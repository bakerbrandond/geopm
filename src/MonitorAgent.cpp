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

#include "MonitorAgent.hpp"
#include "PlatformIO.hpp"
#include "PlatformTopo.hpp"
#include "Helper.hpp"
#include "Exception.hpp"
#include "config.h"


namespace geopm
{
    MonitorAgent::MonitorAgent()
        : MonitorAgent(platform_io(), platform_topo())
    {

    }

    MonitorAgent::MonitorAgent(IPlatformIO &plat_io, IPlatformTopo &topo)
        : m_platform_io(plat_io)
        , m_platform_topo(topo)
        , m_sample_idx(M_NUM_SAMPLE_MAILBOX)
        , m_policy_idx(M_NUM_POLICY_MAILBOX)
    {
        // All columns sampled will be in the trace
        auto trace_cols = trace_columns();
        for (int col_idx = 0; col_idx < M_NUM_SAMPLE_MAILBOX; ++col_idx) {
            m_sample_idx[col_idx] = m_platform_io.push_signal(trace_cols[col_idx].name,
                                                              trace_cols[col_idx].domain_type,
                                                              trace_cols[col_idx].domain_idx);
        }
    }

    std::string MonitorAgent::plugin_name(void)
    {
        return "MONITOR_AGENT";
    }

    std::unique_ptr<IAgent> MonitorAgent::make_plugin(void)
    {
        return geopm::make_unique<MonitorAgent>();
    }

    void MonitorAgent::init(int level)
    {
        /// @todo Some way for use to provide list of signals they want to collect through this agent.
        m_level = level;
        std::cout << "agent level: " << m_level << std::endl;
    }

    void MonitorAgent::descend(const std::vector<double> &in_policy,
                               std::vector<std::vector<double> >&out_policy)
    {
        std::cout << "agent " << m_level << " descend()" << std::endl;
        for (auto &pol : out_policy) {
            /// @todo assert out_policy sub vector size matches in_policy
            pol = in_policy;
        }
    }

    void MonitorAgent::ascend(const std::vector<std::vector<double> > &in_signal,
                              std::vector<double> &out_signal)
    {
        std::cout << "agent " << m_level << " ascend()" << std::endl;
        /// @todo assert out_signal.size() == in_signal[x].size()
        for (size_t sig_idx = 0; sig_idx < out_signal.size(); ++sig_idx) {
             std::vector<double> temp(in_signal.size());  // make this a class member
             for (size_t child_idx = 0; child_idx < in_signal.size(); ++child_idx) {
                 temp[child_idx] = in_signal[child_idx][sig_idx];
             }
             /// @todo functions can be saved as member
             auto agg_func = m_platform_io.agg_function(send_up_names()[sig_idx]);
             out_signal[sig_idx] = agg_func(temp);
        }
    }

    void MonitorAgent::adjust_platform(const std::vector<double> &in_policy)
    {
        std::cout << "agent " << m_level << " adjust_platform(): " << std::endl;
    }

    void MonitorAgent::sample_platform(std::vector<double> &out_sample)
    {
        std::cout << "agent " << m_level << " sample_platform(): ";
        /// @todo assert out_sample.size() == M_NUM_SAMPLE_MAILBOX
        for (int sample_idx = 0; sample_idx < M_NUM_SAMPLE_MAILBOX; ++sample_idx) {
            out_sample[sample_idx] = m_platform_io.sample(m_sample_idx[sample_idx]);
            std::cout << out_sample[sample_idx] << " ";
        }
        std::cout << std::endl;
    }

    void MonitorAgent::wait(void)
    {
        std::cout << "agent " << m_level << " wait()" << std::endl;
        /// @todo
    }

    std::string MonitorAgent::report_header(void)
    {
        return "";
    }

    std::string MonitorAgent::report_node(void)
    {
        return "";
    }

    std::map<uint64_t, std::string> MonitorAgent::report_region(void)
    {
        return {};
    }

    std::vector<IPlatformIO::m_request_s> MonitorAgent::trace_columns(void)
    {
        static std::vector<IPlatformIO::m_request_s> columns = {
            {"TIME", IPlatformTopo::M_DOMAIN_BOARD, 0},
            {"POWER_PACKAGE", IPlatformTopo::M_DOMAIN_BOARD, 0},
            {"FREQUENCY", IPlatformTopo::M_DOMAIN_BOARD, 0},
            {"REGION_PROGRESS", IPlatformTopo::M_DOMAIN_BOARD, 0},
        };
        return columns;
    }

    std::vector<std::string> MonitorAgent::send_down_names(void)
    {
        return {};
    }

    std::vector<std::string> MonitorAgent::send_up_names(void)
    {
        return {"TIME", "POWER_PACKAGE", "FREQUENCY", "REGION_PROGRESS"};
    }
}
