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

#ifndef MONITORAGENT_HPP_INCLUDE
#define MONITORAGENT_HPP_INCLUDE

#include <vector>
#include <map>
#include <string>
#include <memory>

#include "Agent.hpp"

namespace geopm
{
    class IPlatformIO;
    class IPlatformTopo;

    class MonitorAgent : public IAgent
    {
        public:
            enum m_policy_mailbox_idx_e {
                M_NUM_POLICY_MAILBOX,
            };
            enum m_sample_mailbox_idx_e {
                M_SAMPLE_MAILBOX_TIME,
                M_SAMPLE_MAILBOX_POWER,
                M_SAMPLE_MAILBOX_FREQUENCY,
                M_SAMPLE_MAILBOX_REGION_PROGRESS,
                M_NUM_SAMPLE_MAILBOX,
            };

            MonitorAgent();
            MonitorAgent(IPlatformIO &plat_io, IPlatformTopo &topo);
            virtual ~MonitorAgent() = default;
            void init(int level) override;
            void descend(const std::vector<double> &in_policy,
                         std::vector<std::vector<double> >&out_policy) override;
            void ascend(const std::vector<std::vector<double> > &in_signal,
                        std::vector<double> &out_signal) override;
            void adjust_platform(const std::vector<double> &in_policy) override;
            void sample_platform(std::vector<double> &out_sample) override;
            void wait(void) override;
            std::string report_header(void) override;
            std::string report_node(void) override;
            std::map<uint64_t, std::string> report_region(void) override;
            std::vector<IPlatformIO::m_request_s> trace_columns(void) override;
            static std::vector<std::string> send_down_names(void);
            static std::vector<std::string> send_up_names(void);
            static std::string plugin_name(void);
            static std::unique_ptr<IAgent> make_plugin(void);
        private:
            IPlatformIO &m_platform_io;
            IPlatformTopo &m_platform_topo;
            int m_level;
            std::vector<int> m_sample_idx;
            std::vector<int> m_policy_idx;
    };
}

#endif
