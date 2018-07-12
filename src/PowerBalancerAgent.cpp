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

#include <cfloat>
#include <cmath>
#include <algorithm>

#include "PowerGovernor.hpp"
#include "PowerBalancerAgent.hpp"
#include "PowerBalancer.hpp"
#include "PlatformIO.hpp"
#include "PlatformTopo.hpp"
#include "Exception.hpp"
#include "CircularBuffer.hpp"

#include "Helper.hpp"
#include "config.h"

namespace geopm
{
#if 0
    PowerBalancerAgent::PowerBalancerAgent()
        : m_platform_io(platform_io())
        , m_platform_topo(platform_topo())
        , m_level(-1)
        , m_updates_per_sample(5)
        , m_samples_per_control(10)
        , m_min_power_budget(m_platform_io.read_signal("POWER_PACKAGE_MIN", IPlatformTopo::M_DOMAIN_PACKAGE, 0))
        , m_max_power_budget(m_platform_io.read_signal("POWER_PACKAGE_MAX", IPlatformTopo::M_DOMAIN_PACKAGE, 0))
        , m_power_gov(geopm::make_unique<PowerGovernor> (m_platform_io, m_platform_topo))
        , m_power_balancer(nullptr)
        , m_pio_idx(M_PLAT_NUM_SIGNAL)
        , m_agg_func {
              IPlatformIO::agg_min, // M_SAMPLE_STEP
              IPlatformIO::agg_and, // M_SAMPLE_IS_STEP_COMPLETE
              IPlatformIO::agg_max, // M_SAMPLE_RUNTIME
              IPlatformIO::agg_sum, // M_SAMPLE_POWER_SLACK
          }
        , m_num_children(0)
        , m_is_level_root(false)
        , m_is_tree_root(false)
        , m_last_epoch_count(0)
        , m_step(M_STEP_SEND_DOWN_LIMIT);
    {
#ifdef GEOPM_DEBUG
        if (m_agg_func.size() != M_NUM_SAMPLE) {
            throw Exception("PowerBalancerAgent(): aggregation function vector is not the size of the policy vector",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
    }

    PowerBalancerAgent::~PowerBalancerAgent() = default;

    void PowerBalancerAgent::init(int level, const std::vector<int> &fan_in, bool is_root)
    {
        m_level = level;
        if (m_level == 0) {
            // Only do this at the leaf level.
            init_platform_io();
            m_power_balancer = geopm::make_unique<PowerBalancer>();
            m_num_children = 1;
        }
        m_num_children = fan_in[level - 1];
        m_is_level_root = is_root;
        m_is_tree_root = (level == fan_in.size()) && is_root;
    }

    void PowerBalancerAgent::init_platform_io(void)
    {
        m_power_gov->init_platform_io();
        // Setup signals
        m_pio_idx[M_PLAT_SIGNAL_EPOCH_RUNTIME] = m_platform_io.push_signal("EPOCH_RUNTIME", IPlatformTopo::M_DOMAIN_BOARD, 0);
        m_pio_idx[M_PLAT_SIGNAL_EPOCH_COUNT] = m_platform_io.push_signal("EPOCH_COUNT", IPlatformTopo::M_DOMAIN_BOARD, 0);
    }

    int PowerBalancerAgent::step(size_t step_count)
    {
        return (step_count % M_NUM_STEP);
    }

    int PowerBalancerAgent::step(void)
    {
        return step(m_step_count);
    }

    bool PowerBalancerAgent::descend(const std::vector<double> &policy_in, std::vector<std::vector<double> > &policy_out)
    {
#ifdef GEOPM_DEBUG
        if (policy_in.size() != M_NUM_POLICY ||
            policy_out.size() != m_num_children) {
            throw Exception("PowerBalancerAgent::" + std::string(__func__) + "(): policy vectors are not correctly sized.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        bool result = (policy_in == m_policy);
        for (auto &po : policy_out) {
            po = policy_in;
        }
        m_policy = policy_in;

        return result;
    }


    bool PowerBalancerAgent::ascend(const std::vector<std::vector<double> > &in_sample, std::vector<double> &out_sample)
    {
#ifdef GEOPM_DEBUG
        if (in_sample.size() != m_num_children ||
            out_sample.size() != M_NUM_SAMPLE) {
            throw Exception("PowerBalancerAgent::ascend(): sample vectors not correctly sized.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        bool result = false;
        for (int child_idx = 0; child_idx != m_num_children; ++child_idx) {
            for (sample_idx = 0; sample_idx != M_NUM_SAMPLE; ++sample_idx) {
                out_sample[sample_idx] = m_agg_function[sample_idx](in_sample[sample_idx]);
            }
        }
        if (m_is_root) {
            /// @todo If this agent is the tree root, extra stuff needs to happen here.
        }
        return out_sample[M_SAMPLE_IS_STEP_COMPLETE];
    }

    bool PowerBalancerAgent::adjust_platform(const std::vector<double> &in_policy)
    {
#ifdef GEOPM_DEBUG
        if (in_policy.size() != M_NUM_POLICY) {
            throw Exception("PowerBalancerAgent::" + std::string(__func__) + "(): policy vector incorrectly sized.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        if (step(in_policy[M_POLICY_STEP]) != step()) {
            if (in_policy[M_POLICY_POWER_CAP != 0.0) {
                // New power cap from resource manager, reset
                // algorithm.
                m_step_count = M_STEP_SEND_DOWN_LIMIT;
            }
            else if (m_is_step_complete &&
                     in_policy[M_POLICY_STEP) == m_policy_count + 1)) {
                // Advance a step
                ++m_step_count;
                m_is_step_complete = false;
                switch(step()) {
                    case M_STEP_MEASURE_RUNTIME:
                        break;
                    case M_STEP_SEND_UP_RUNTIME:
                        break;
                    case M_STEP_SEND_DOWN_RUNTIME:
                        break;
                    case M_STEP_REDUCE_LIMIT:
                        break;
                    case M_STEP_SEND_UP_EXCESS:
                        break;
                    case M_STEP_SEND_DOWN_LIMIT:
                        break;
                    case M_STEP_INCREASE_CAP:
                        break;
                }
            }
            else {
                // Throw, agent is out of step with the policy
            }
        }
        double actual_limit = 0.0;
        double request_limit = m_power_balancer->power_limit();
        bool result = m_power_gov->adjust_platform(request_limit, actual_limit);
        if (actual_limit != request_limit) {
            /// @todo Depending on step, need to do something here.

        }
        return result;
    }

    bool PowerBalancerAgent::sample_platform(std::vector<double> &out_sample)
    {
#ifdef GEOPM_DEBUG
        if (out_sample.size() != M_NUM_SAMPLE) {
            throw Exception("PowerBalancerAgent::" + std::string(__func__)  + "(): out_sample vector not correctly sized.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        int epoch_count = m_platform_io->sample(m_pio_idx[M_PLAT_SIGNAL_EPOCH_COUNT]);
        // If all of the ranks have observed a new epoch then update
        // the power_balancer.
        if (epoch_count != m_last_epoch_count) {
            double epoch_runtime = m_platform_io->sample(m_pio_idx[M_PLAT_SIGNAL_EPOCH_RUNTIME]);
            switch (m_step) {
                case M_STEP_MEASURE_RUNTIME:
                    m_is_step_complete = m_power_balancer->is_runtime_stable(epoch_runtime);
                    m_runtime = m_power_balancer->runtime_sample();
                    result = m_is_step_complete;
                    break;
                case M_STEP_REDUCE_LIMIT:
                    m_is_step_complete = m_power_balancer->is_target_met(epoch_runtime);
                    m_power_slack = m_power_balancer->power_cap() - m_power_balancer->power_limit();
                    result = m_is_step_complete;
                    break;
                default:
                    break;
            }
            m_last_epoch_count = epoch_count;
        }
        m_power_gov->sample_platform();
        out_sample[M_SAMPLE_STEP] = m_step_count;
        out_sample[M_SAMPLE_IS_STEP_COMPLETE] = m_is_step_complete;
        out_sample[M_SAMPLE_RUNTIME] = m_runtime;
        out_sample[M_SAMPLE_POWER_SLACK] = m_power_slack;
        return m_is_step_complete;
    }

    void PowerBalancerAgent::wait()
    {
        // Wait for updates to the energy status register
        double curr_energy_status = 0;

        for (int i = 0; i < m_updates_per_sample; ++i) {
            do  {
                curr_energy_status = m_platform_io.read_signal("ENERGY_PACKAGE", IPlatformTopo::M_DOMAIN_PACKAGE, 0);
            }
            while (m_last_energy_status == curr_energy_status);
            m_last_energy_status = curr_energy_status;
        }
    }

    std::vector<std::pair<std::string, std::string> > PowerBalancerAgent::report_header(void) const
    {
        return {};
    }

    std::vector<std::pair<std::string, std::string> > PowerBalancerAgent::report_node(void) const
    {
        return {};
    }

    std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > PowerBalancerAgent::report_region(void) const
    {
        return {};
    }

    std::vector<std::string> PowerBalancerAgent::trace_names(void) const
    {
        return {"epoch_runtime",
                "power_package",
                "power_dram",
                "is_converged",
                "power_budget"};
    }

    void PowerBalancerAgent::trace_values(std::vector<double> &values)
    {
#ifdef GEOPM_DEBUG
        if (values.size() != M_TRACE_NUM_SAMPLE) { // Everything sampled from the platform plus convergence (and the power budget soon...)
            throw Exception("PowerBalancerAgent::" + std::string(__func__) + "(): values vector not correctly sized.",
                            GEOPM_ERROR_LOGIC, __FILE__, __LINE__);
        }
#endif
        values[M_TRACE_SAMPLE_EPOCH_RUNTIME] = m_sample[M_PLAT_SIGNAL_EPOCH_RUNTIME];
        values[M_TRACE_SAMPLE_PKG_POWER] = m_sample[M_PLAT_SIGNAL_PKG_POWER];
        values[M_TRACE_SAMPLE_DRAM_POWER] = m_sample[M_PLAT_SIGNAL_DRAM_POWER];
        values[M_TRACE_SAMPLE_IS_CONVERGED] = m_is_converged;
        values[M_TRACE_SAMPLE_PWR_BUDGET] = m_last_power_budget_out;
    }

    std::string PowerBalancerAgent::plugin_name(void)
    {
        return "power_balancer";
    }

    std::unique_ptr<Agent> PowerBalancerAgent::make_plugin(void)
    {
        return geopm::make_unique<PowerBalancerAgent>();
    }

    std::vector<std::string> PowerBalancerAgent::policy_names(void)
    {
        return {"POWER"};
    }

    std::vector<std::string> PowerBalancerAgent::sample_names(void)
    {
        return {"EPOCH_RUNTIME", "POWER", "IS_CONVERGED"};
    }
#endif
}
