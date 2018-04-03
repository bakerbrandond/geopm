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

#include "Agent.hpp"
#include "MonitorAgent.hpp"
#include "BalancingAgent.hpp"

namespace geopm
{
    static PluginFactory<IAgent> *g_plugin_factory;
    static pthread_once_t g_register_built_in_once = PTHREAD_ONCE_INIT;
    static void register_built_in_once(void)
    {
        g_plugin_factory->register_plugin(MonitorAgent::plugin_name(),
                                          MontiorAgent::make_plugin,
                                          IAgent::make_dictionary(MonitorAgent::M_NUM_POLICY_MAILBOX,
                                                                  MonitorAgent::M_NUM_SAMPLE_MAILBOX));


        g_plugin_factory->register_plugin(BalancingAgent::plugin_name(),
                                          BalancingAgent::make_plugin,
                                          IAgent::make_dictionary(BalancingAgent::M_NUM_POLICY_MAILBOX,
                                                                  BalancingAgent::M_NUM_SAMPLE_MAILBOX))
    }

    PluginFactory<IAgent> &agent_factory(void)
    {
        static PluginFactory<IAgent> instance;
        g_plugin_factory = &instance;
        pthread_once(&g_register_built_in_once, register_built_in_once);
        return instance;
    }

    const std::string IAgent::m_num_send_up_string = "NUM_SEND_UP";
    const std::string IAgent::m_num_send_down_string = "NUM_SEND_DOWN";

    int IAgent::num_send_up(const std::map<std::string, std::string> &dictionary)
    {
        auto it = dictionary.find(m_num_send_up_string);
        if (it == dictionary.end()) {
            throw Exception("IAgent::num_send_up(): "
                            "Agent was not registered with plugin factory with the correct dictionary.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
        return atoi(it->second.c_str());
    }
    int IAgent::num_send_down(const std::map<std::string, std::string> &dictionary)
    {
        auto it = dictionary.find(m_num_send_down_string);
        if (it == dictionary.end()) {
            throw Exception("IAgent::num_send_down(): "
                            "Agent was not registered with plugin factory with the correct dictionary.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
        return atoi(it->second.c_str());
    }
    std::map<std::string, std::string> IAgent::make_dictionary(int num_send_up, int num_send_down)
    {
        std::map<std::string, std::string> result = {{m_num_send_up_string, std::to_string(num_send_up)},
                                                     {m_num_send_down_string, std::to_string(num_send_down)}};
        return result;
    }
}
