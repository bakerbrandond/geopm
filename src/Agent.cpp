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
                                          MonitorAgent::make_plugin,
                                          IAgent::make_dictionary(MonitorAgent::send_down_names(),
                                                                  MonitorAgent::send_up_names()));


        g_plugin_factory->register_plugin(BalancingAgent::plugin_name(),
                                          BalancingAgent::make_plugin,
                                          IAgent::make_dictionary(BalancingAgent::send_down_names(),
                                                                  BalancingAgent::send_up_names()));
    }

    PluginFactory<IAgent> &agent_factory(void)
    {
        static PluginFactory<IAgent> instance;
        g_plugin_factory = &instance;
        pthread_once(&g_register_built_in_once, register_built_in_once);
        return instance;
    }

    int IAgent::num_send_up(const std::map<std::string, std::string> &dictionary)
    {
        auto it = dictionary.find("NUM_SEND_UP");
        if (it == dictionary.end()) {
            throw Exception("IAgent::num_send_up(): "
                            "Agent was not registered with plugin factory with the correct dictionary.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
        return atoi(it->second.c_str());
    }
    int IAgent::num_send_down(const std::map<std::string, std::string> &dictionary)
    {
        auto it = dictionary.find("NUM_SEND_DOWN");
        if (it == dictionary.end()) {
            throw Exception("IAgent::num_send_down(): "
                            "Agent was not registered with plugin factory with the correct dictionary.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
        return atoi(it->second.c_str());
    }

    std::vector<std::string> IAgent::send_up_names(const std::map<std::string, std::string> &dictionary)
    {
        std::vector<std::string> result;
        for (const auto &it : dictionary) {
            if (it.first.find("SEND_UP_") == 0) {
                result.push_back(it.second);
            }
        }
        return result;
    }

    std::vector<std::string> IAgent::send_down_names(const std::map<std::string, std::string> &dictionary)
    {
        std::vector<std::string> result;
        for (const auto &it : dictionary) {
            if (it.first.find("SEND_DOWN_") == 0) {
                result.push_back(it.second);
            }
        }
        return result;
    }

    std::map<std::string, std::string> IAgent::make_dictionary(std::vector<std::string> send_up_names, std::vector<std::string> send_down_names)
    {
        std::map<std::string, std::string> result;
        for (size_t send_up_idx = 0; send_up_idx != send_up_names.size(); ++send_up_idx) {
            std::string key = "SEND_UP_" + std::to_string(send_up_idx);
            result[key] = send_up_names[send_up_idx];
        }
        result["NUM_SEND_UP"] = std::to_string(send_up_names.size());

        for (size_t send_down_idx = 0; send_down_idx != send_down_names.size(); ++send_down_idx) {
            std::string key = "SEND_DOWN_" + std::to_string(send_down_idx);
            result[key] = send_down_names[send_down_idx];
        }
        result["NUM_SEND_DOWN"] = std::to_string(send_down_names.size());
        return result;
    }
}
