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

#include "PowerGoverningAgent.hpp"
#include "PlatformIO.hpp"
#include "PlatformTopo.hpp"
#include "Helper.hpp"
#include "Exception.hpp"
#include "config.h"

namespace geopm
{
    PowerGoverningAgent::PowerGoverningAgent()
        : PowerGoverningAgent(platform_io(), platform_topo())
    {

    }

    PowerGoverningAgent::PowerGoverningAgent(IPlatformIO &plat_io, IPlatformTopo &topo)
        : m_platform_io(plat_io)
        , m_platform_topo(topo)
        , m_last_wait{{0, 0}}
        , m_num_ascend(0)
        , M_SEND_PERIOD(10)
        , M_WAIT_SEC(0.005)
        , m_curr_budget(NAN)
        , m_local_sample(M_NUM_LOCAL_SAMPLE, NAN)
        , m_local_sample_idx(M_NUM_LOCAL_SAMPLE, -1)
        , m_power_ctl_idx(-1)
    {
        geopm_time(&m_last_wait);
        std::vector<std::string> local_sample_names = {"POWER_PACKAGE", "POWER_DRAM"};
        /// @todo assert local_sample_names.size == M_NUM_LOCAL_SAMPLE
        m_power_ctl_idx = m_platform_io.push_control("POWER", IPlatformTopo::M_DOMAIN_BOARD, 0);
        for (int x = 0; x < M_NUM_LOCAL_SAMPLE; ++x) {
            m_local_sample_idx[x] = m_platform_io.push_signal(local_sample_names[x], IPlatformTopo::M_DOMAIN_BOARD, 0);
            /// @todo agg_func?
        }

        for (auto name : sample_names()) {
            m_sample_idx.push_back(m_platform_io.push_signal(name,
                                                             IPlatformTopo::M_DOMAIN_BOARD,
                                                             0));
            m_agg_func.push_back(m_platform_io.agg_function(name));
        }
        m_num_sample = m_sample_idx.size();
    }

    std::string PowerGoverningAgent::plugin_name(void)
    {
        return "power_governor";
    }

    std::unique_ptr<Agent> PowerGoverningAgent::make_plugin(void)
    {
        return geopm::make_unique<PowerGoverningAgent>();
    }

    void PowerGoverningAgent::init(int level)
    {
        m_level = level;
    }

    bool PowerGoverningAgent::descend(const std::vector<double> &in_policy,
                               std::vector<std::vector<double> >&out_policy)
    {
#ifdef GEOPM_DEBUG
        /// @todo is this check right?
        /// @todo once init is updated, do we check the out_policy size against the next level in fan_in?
        if (in_policy.size() != 1) {
            throw Exception("PowerGoverningAgent::descend(): in_policy vector not correctly sized.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        m_curr_budget = in_policy[0];
        std::fill(out_policy.begin(), out_policy.end(), in_policy);
        return false;/// @todo always return true?
    }

    bool PowerGoverningAgent::ascend(const std::vector<std::vector<double> > &in_sample,
                              std::vector<double> &out_sample)
    {
#ifdef GEOPM_DEBUG
        if (out_sample.size() != m_num_sample) {
            throw Exception("PowerGoverningAgent::ascend(): out_sample vector not correctly sized.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        std::vector<double> child_sample(in_sample.size());
        for (size_t sig_idx = 0; sig_idx < m_num_sample; ++sig_idx) {
            for (size_t child_idx = 0; child_idx < in_sample.size(); ++child_idx) {
                child_sample[child_idx] = in_sample[child_idx][sig_idx];
            }
            out_sample[sig_idx] = m_agg_func[sig_idx](child_sample);
        }
        return false;
    }

    bool PowerGoverningAgent::adjust_platform(const std::vector<double> &in_policy)
    {
        static double last_budget = NAN;
        bool ret_val = false;
        if (last_budget != m_curr_budget) {
            double new_val = m_curr_budget - m_local_sample[M_SAMPLE_DRAM_POWER];
            m_platform_io.adjust(m_power_ctl_idx, new_val);
            ret_val = true;
            last_budget = m_curr_budget;
        }
        return ret_val;
    }

    bool PowerGoverningAgent::sample_platform(std::vector<double> &out_sample)
    {
#ifdef GEOPM_DEBUG
        if (out_sample.size() != m_num_sample) {
            throw Exception("PowerGoverningAgent::sample_platform(): out_sample vector not correctly sized.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        bool result = false;
        if (m_num_ascend == 0) {
            for (size_t sample_idx = 0; sample_idx < m_num_sample; ++sample_idx) {
                out_sample[sample_idx] = m_platform_io.sample(m_sample_idx[sample_idx]);
            }
            result = true;
        }
        ++m_num_ascend;
        if (m_num_ascend == M_SEND_PERIOD) {
           m_num_ascend = 0;
        }
        for (int x = 0; x < M_NUM_LOCAL_SAMPLE; ++x) {
            m_local_sample[x] = m_platform_io.sample(m_local_sample_idx[x]);
        }
        return result;
    }

    void PowerGoverningAgent::wait(void)
    {
        geopm_time_s current_time;
        do {
            geopm_time(&current_time);
        }
        while(geopm_time_diff(&m_last_wait, &current_time) < M_WAIT_SEC);
        geopm_time(&m_last_wait);
    }

    std::vector<std::string> PowerGoverningAgent::policy_names(void)
    {
        return {};
    }

    std::vector<std::string> PowerGoverningAgent::sample_names(void)
    {
        return {};
    }

    std::vector<std::pair<std::string, std::string> > PowerGoverningAgent::report_header(void)
    {
        return {};
    }

    std::vector<std::pair<std::string, std::string> > PowerGoverningAgent::report_node(void)
    {
        return {};
    }

    std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > PowerGoverningAgent::report_region(void)
    {
        return {};
    }

    std::vector<std::string> PowerGoverningAgent::trace_names(void) const
    {
        return {};
    }

    void PowerGoverningAgent::trace_values(std::vector<double> &values)
    {

    }
}
