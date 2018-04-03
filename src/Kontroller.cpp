/*
 * Copyright (c) 2015, 2016, 2017, 2018, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>

#include "geopm_env.h"
#include "geopm_signal_handler.h"
#include "geopm_message.h"
#include "geopm_time.h"
#include "Kontroller.hpp"
#include "ApplicationIO.hpp"
#include "Reporter.hpp"
#include "Tracer.hpp"
#include "Exception.hpp"
#include "Comm.hpp"
#include "PlatformTopo.hpp"
#include "PlatformIO.hpp"
#include "Agent.hpp"
#include "TreeComm.hpp"
#include "config.h"

extern "C"
{
    static void *geopm_threaded_run(void *args)
    {
        long err = 0;
        geopm::Kontroller *ctl = (geopm::Kontroller *)args;
        try {
            ctl->run();
        }
        catch (...) {
            err = geopm::exception_handler(std::current_exception());
        }
        return (void *)err;
    }
}

namespace geopm
{
    Kontroller::Kontroller(std::shared_ptr<IComm> ppn1_comm,
               const std::string &global_policy_path)
        : Kontroller(platform_topo(),
                     platform_io(),
                     geopm_env_agent(),
                     IAgent::num_send_up(agent_factory().dictionary(m_agent_name)),
                     IAgent::num_send_down(agent_factory().dictionary(m_agent_name)),
                     std::unique_ptr<ITreeComm>(new TreeComm(ppn1_comm, m_num_send_up, m_num_send_down)),
                     m_tree_comm->num_level_controlled(),
                     m_tree_comm->root_level(),
                     std::unique_ptr<IApplicationIO>(new ApplicationIO(geopm_env_shmkey())),
                     std::unique_ptr<IReporter>(new Reporter(geopm_env_report(), geopm_env_report_verbosity())),
                     std::unique_ptr<ITracer>(new Tracer(geopm_env_trace())),
                     std::vector<std::unique_ptr<IAgent> >{},
                     {})
    {
        for (int level = 0; level != m_num_level_ctl; ++level) {
            m_agent.push_back(agent_factory().make_plugin(m_agent_name));
            m_agent.back()->init(level);
        }

    }

    Kontroller::Kontroller(IPlatformTopo &plat_topo,
                           IPlatformIO &plat_io,
                           const std::string &agent_name,
                           int num_send_down,
                           int num_send_up,
                           std::unique_ptr<ITreeComm> tree_comm,
                           int num_level_ctl,
                           int root_level,
                           std::unique_ptr<IApplicationIO> application_io,
                           std::unique_ptr<IReporter> reporter,
                           std::unique_ptr<ITracer> tracer,
                           std::vector<std::unique_ptr<IAgent> > level_agent,
                           std::map<std::string, double> manager_values)
        : m_platform_topo(plat_topo)
        , m_platform_io(plat_io)
        , m_agent_name(agent_name)
        , m_num_send_down(num_send_down)
        , m_num_send_up(num_send_up)
        , m_tree_comm(std::move(tree_comm))
        , m_num_level_ctl(num_level_ctl)
        , m_root_level(root_level)
        , m_application_io(std::move(application_io))
        , m_reporter(std::move(reporter))
        , m_tracer(std::move(tracer))
        , m_agent(std::move(level_agent))
        , m_is_root(m_num_level_ctl - 1 == m_root_level)
        , m_in_policy(m_num_send_down)
        , m_out_policy(m_num_level_ctl)
        , m_in_sample(m_num_level_ctl)
        , m_out_sample(m_num_send_up)
        , m_manager_values(manager_values)
    {
        for (const auto &name : m_reporter->signal_names()) {
            m_reporter_sample_idx.push_back(m_platform_io.push_signal(name, IPlatformTopo::M_DOMAIN_BOARD, 0));
        }
        m_reporter_sample.resize(m_reporter_sample_idx.size());

        // Three dimensional vector over levels, children, and message
        // index.  These are used as temporary storage when passing
        // messages up and down the tree.
        for (int level = 0; level != m_num_level_ctl; ++level) {
            int num_children = m_tree_comm->level_size(level);
            m_out_policy[level] = std::vector<std::vector<double> >(num_children,
                                                                    std::vector<double>(m_num_send_down));
            m_in_sample[level] = std::vector<std::vector<double> >(num_children,
                                                                   std::vector<double>(m_num_send_up));
        }

        /// @todo move somewhere else: need to happen after Agents are constructed
        // sanity checks
        if (m_agent.size() == 0) {
            throw Exception("Kontroller requires at least one Agent",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if (m_agent.size() != (size_t)m_num_level_ctl) {
            throw Exception("Kontroller must have one agent per level",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if ((size_t)m_num_send_down != m_agent[0]->policy_names().size()) {
            throw Exception("Kontroller: num_send_down does not match num policies provided by Agent.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }
        if ((size_t)m_num_send_up != m_agent[0]->sample_names().size()) {
            throw Exception("Kontroller: num_send_up does not match num samples provided by Agent.",
                            GEOPM_ERROR_INVALID, __FILE__, __LINE__);
        }

        m_agent_policy_names = m_agent[0]->policy_names();
        m_agent_sample_names = m_agent[0]->sample_names();
        for (auto name : m_agent_policy_names) {
            if (m_manager_values.find(name) == m_manager_values.end()) {
                throw Exception("Kontroller: resource manager did not provide required "
                                "agent policy value for " + name,
                                GEOPM_ERROR_INVALID, __FILE__, __LINE__);
            }
        }
    }

    Kontroller::~Kontroller()
    {
        geopm_signal_handler_check();
        geopm_signal_handler_revert();
    }

    void Kontroller::run(void)
    {
        setup_trace();
        while (m_application_io->do_sample()) {
            step();
        }
        generate();
    }

    void Kontroller::generate(void)
    {
        std::string agent_report_header;
        if (m_num_level_ctl == m_root_level) {
            agent_report_header = m_agent[m_root_level - 1]->report_header();
        }
        /// @todo why get node reports from each level of the tree?
        std::ostringstream agent_node_report;
        for (int level = 0; level != m_num_level_ctl; ++level) {
            agent_node_report << m_agent[level]->report_node();
        }

        m_reporter->generate(m_application_io->report_name(),
                             m_application_io->profile_name(),
                             m_agent_name,
                             agent_report_header,
                             agent_node_report.str(),
                             m_agent[0]->report_region(),
                             m_application_io->region_name_set(),
                             nullptr); /// @todo get the IComm from somwhere
        m_tracer->flush();
    }

    void Kontroller::step(void)
    {
        walk_down();
        geopm_signal_handler_check();
        std::vector<uint64_t> short_region = m_application_io->short_region();
        struct geopm_time_s epoch_time;
        bool is_epoch = m_application_io->epoch_time(epoch_time);
        auto reporter_it = m_reporter_sample.begin();
        for (auto pio_idx : m_reporter_sample_idx) {
            *reporter_it = m_platform_io.sample(pio_idx);
            ++reporter_it;
        }
        // Pass short_region and is_epoch to increment the count and
        // epoch_time to track epoch time
        m_reporter->update(m_reporter_sample, short_region, is_epoch, epoch_time);
        auto tracer_it = m_tracer_sample.begin();
        for (auto pio_idx : m_tracer_sample_idx) {
            *tracer_it = m_platform_io.sample(pio_idx);
            ++tracer_it;
        }
        // Pass short_region and is_epoch to add extra entries into
        // trace for each.
        m_tracer->update(m_tracer_sample, short_region, is_epoch);
        walk_up();
        geopm_signal_handler_check();
        m_agent[0]->wait();
        geopm_signal_handler_check();
    }

    void Kontroller::walk_down(void)
    {
        int level = m_num_level_ctl - 1;//m_tree_comm->num_level_controlled() - 1;
        if (m_is_root) {
            std::cout << "KON sample manager" << std::endl;
            /// @todo: no longer using IOgroup for this. will be special object with a similar API
            int policy_idx = 0;
            for (auto key : m_agent_policy_names) {
                m_in_policy[policy_idx] = m_manager_values.at(key);
                ++policy_idx;
            }
        }
        else {
            m_tree_comm->receive_down(level, m_in_policy);
        }
        m_agent[level]->descend(m_in_policy, m_out_policy[level]);
        m_tree_comm->send_down(level, m_out_policy[level]);
        --level;
        for (; level > 1; --level) {
            m_tree_comm->receive_down(level, m_in_policy);
            m_agent[level]->descend(m_in_policy, m_out_policy[level]);
            m_tree_comm->send_down(level, m_out_policy[level]);
        }
        m_tree_comm->receive_down(level, m_in_policy);
        m_agent[level]->adjust_platform(m_in_policy);
        m_platform_io.write_batch();
    }

    void Kontroller::walk_up(void)
    {
        int level = 0;
        m_platform_io.read_batch();
        m_agent[level]->sample_platform(m_out_sample);
        m_tree_comm->send_up(level, m_out_sample);
        ++level;
        for (; level != m_num_level_ctl; ++level) {
            m_tree_comm->receive_up(level, m_in_sample[level]);
            m_agent[level]->ascend(m_in_sample[level], m_out_sample);
            if (level != m_num_level_ctl - 1) {
                std::cout << "KON level " << level << " send up" << std::endl;
                m_tree_comm->send_up(level, m_out_sample);
            }
        }
        if (m_is_root) {
            std::cout << "KON level " << level << " send to manager: ";
            for (auto sample : m_out_sample) {
                std::cout << sample << " ";
            }
            std::cout << std::endl;
            // At the root of the tree, send signals up to the
            // resource manager.
        }
    }

    void Kontroller::pthread(const pthread_attr_t *attr, pthread_t *thread)
    {
        int err = pthread_create(thread, attr, geopm_threaded_run, (void *)this);
        if (err) {
            throw Exception("Controller::pthread(): pthread_create() failed",
                            err, __FILE__, __LINE__);
        }
    }

    void Kontroller::setup_trace(void)
    {
        std::vector<IPlatformIO::m_request_s> columns {{"TIME", PlatformTopo::M_DOMAIN_BOARD, 0},
                                                       {"REGION_ID#", PlatformTopo::M_DOMAIN_BOARD, 0},
                                                       {"ENERGY", PlatformTopo::M_DOMAIN_BOARD, 0},
                                                       {"POWER", PlatformTopo::M_DOMAIN_BOARD, 0},
                                                       {"FREQUENCY_MIN", PlatformTopo::M_DOMAIN_BOARD, 0},
                                                       {"FREQUENCY_MAX", PlatformTopo::M_DOMAIN_BOARD, 0},
                                                       {"FREQUENCY_AVG", PlatformTopo::M_DOMAIN_BOARD, 0}};
        std::vector<IPlatformIO::m_request_s> agent_columns = m_agent[0]->trace_columns();
        columns.insert(columns.end(), agent_columns.begin(), agent_columns.end());
        m_tracer->columns(columns);
        m_tracer_sample.resize(columns.size());
        for (const auto &col : columns) {
            m_tracer_sample_idx.push_back(m_platform_io.push_signal(col.name, col.domain_type, col.domain_idx));
        }
    }
}
