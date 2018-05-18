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

#include "EnergyEfficientRegion.hpp"
#include "PlatformIO.hpp"
#include "PlatformTopo.hpp"
#include "Exception.hpp"
#include "config.h"

///@todo
#include <iostream>

namespace geopm
{

    EnergyEfficientRegion::EnergyEfficientRegion(IPlatformIO &platform_io,
                                                 double freq_min, double freq_max,
                                                 double freq_step, int num_domain,
                                                 int runtime_idx,
                                                 int pkg_energy_idx,
                                                 int dram_energy_idx)
        : m_platform_io(platform_io)
        , m_curr_freq(NAN)
        , m_num_domain(num_domain)
        , m_runtime_idx(runtime_idx)
        , m_pkg_energy_idx(pkg_energy_idx)
        , m_dram_energy_idx(dram_energy_idx)
    {
        update_freq_range(freq_min, freq_max, freq_step);
    }

    void EnergyEfficientRegion::update_freq_range(const double freq_min, const double freq_max, const double freq_step)
    {
        /// @todo m_freq_step == freq_step else we have to re-key our map
        ///       or make m_freq_step const
        const struct m_freq_ctx_s freq_ctx_stub = {.is_learning = true,
                                                   .num_increase = 0,
                                                   .perf_max = 0.0,
                                                   .energy_min = 0.0,
                                                   .num_sample = 0,};
        // set up allowed frequency range
        m_freq_step = freq_step;
        double num_freq_step = 1 + (size_t)(ceil((freq_max - freq_min) / m_freq_step));
        m_allowed_freq.clear();
        double freq = 0.0;
        std::cout << __func__ << " emplacing: {" << std::endl;
        for (double step = 0; step < num_freq_step; ++step) {
            freq = freq_min + (step * m_freq_step);
            m_allowed_freq.insert(freq);
            m_freq_ctx_map.emplace(std::piecewise_construct,
                                   std::make_tuple(freq / m_freq_step),
                                   std::make_tuple(freq_ctx_stub));
            std::cout << "emplaced: " << freq << ", num_step: " << freq / m_freq_step << std::endl;
        }
        m_curr_freq_max = freq;
        std::cout <<"}" << std::endl << __func__ << " m_curr_freq_max(" << m_curr_freq_max << ")" << std::endl;
        if (isnan(m_curr_freq)) {
            m_curr_freq = m_curr_freq_max;
        } else if (m_curr_freq < *m_allowed_freq.begin()) {
            m_curr_freq = *m_allowed_freq.begin();
        } else if (m_curr_freq > m_curr_freq_max) {
            m_curr_freq = m_curr_freq_max;
        }
        std::cout << __func__ << " m_curr_freq(" << m_curr_freq << ")" << std::endl;
    }

    double EnergyEfficientRegion::perf_metric()
    {
        double runtime = m_platform_io.sample(m_runtime_idx);
        if (isnan(runtime)) {
            runtime = 0.0;
        }
        std::cout << __func__ << " return(" << -1.0 * runtime << ")" << std::endl;
        // Higher is better for performance, so negate
        return -1.0 * runtime;
    }

    double EnergyEfficientRegion::energy_metric()
    {
        double total_energy = 0.0;
        total_energy += m_platform_io.sample(m_pkg_energy_idx);
        total_energy += m_platform_io.sample(m_dram_energy_idx);
        std::cout << __func__ << " return(" << total_energy << ")" << std::endl;
        return total_energy;
    }

    double EnergyEfficientRegion::freq(void) const
    {
        std::cout << __func__ << " return(" << m_curr_freq << ")" << std::endl;
        return m_curr_freq;
    }

    void EnergyEfficientRegion::update_entry()
    {
        m_start_energy = energy_metric();
    }

    void EnergyEfficientRegion::update_exit()
    {
        auto &curr_freq_ctx = m_freq_ctx_map[m_curr_freq];
        auto step_up_freq_ctx_it = m_freq_ctx_map.find(m_curr_freq + m_freq_step);
        std::cout << __func__ << " m_curr_freq(" << m_curr_freq << ")";
        if (curr_freq_ctx.is_learning) {
            double perf = perf_metric();
            double energy = energy_metric() - m_start_energy;
            if (!isnan(perf) && !isnan(energy)) {
                if (curr_freq_ctx.num_sample == 0 ||
                    curr_freq_ctx.perf_max < perf) {
                    curr_freq_ctx.perf_max = perf;
                std::cout << "\t.perf_max(" << curr_freq_ctx.perf_max << ")" << std::endl;
                }
                if (curr_freq_ctx.num_sample == 0 ||
                    curr_freq_ctx.energy_min > energy) {
                    curr_freq_ctx.energy_min = energy;
                }
                ++curr_freq_ctx.num_sample;
                std::cout << "\t.num_sample(" << curr_freq_ctx.num_sample << ")" << std::endl;
            }

            if (curr_freq_ctx.num_sample > 0) {
                if (curr_freq_ctx.num_sample >= M_MIN_BASE_SAMPLE &&
                    m_target == 0.0 &&
                    m_curr_freq == m_curr_freq_max) {

                    if (curr_freq_ctx.perf_max > 0.0) {
                        m_target = (1.0 - M_PERF_MARGIN) * curr_freq_ctx.perf_max;
                        std::cout << "m_target(" << m_target << ")" << std::endl;
                    }
                    else {
                        m_target = (1.0 + M_PERF_MARGIN) * curr_freq_ctx.perf_max;
                        std::cout << "m_target(" << m_target << ")" << std::endl;
                    }
                }

                bool do_increase = false;
                // assume best min energy is at highest freq if energy follows cpu-bound
                // pattern; otherwise, energy should decrease with frequency.
                if (step_up_freq_ctx_it != m_freq_ctx_map.end()) {
                    auto step_up_freq_ctx = step_up_freq_ctx_it->second;
                    if (m_curr_freq != m_curr_freq_max &&
                        step_up_freq_ctx.energy_min < (1.0 - M_ENERGY_MARGIN) * curr_freq_ctx.energy_min) {
                        do_increase = true;
                    }
                    else if (m_target != 0.0) {
                        if (curr_freq_ctx.perf_max > m_target) {
                            double next_freq = m_curr_freq - m_freq_step;
                            if (m_allowed_freq.find(next_freq) != m_allowed_freq.end()) {
                                // Performance is in range; lower frequency
                                m_curr_freq = next_freq;
                            }
                        }
                        else {
                            double next_freq = m_curr_freq + m_freq_step;
                            if (m_allowed_freq.find(next_freq) != m_allowed_freq.end()) {
                                do_increase = true;
                            }
                        }
                    }
                }
                std::cout << "do_increase(" << do_increase << ")" << std::endl;
                if (do_increase) {
                    // Performance degraded too far; increase freq
                    ++curr_freq_ctx.num_increase;
                    // If the frequency has been lowered too far too
                    // many times, stop learning
                    if (curr_freq_ctx.num_increase == M_MAX_INCREASE) {
                        curr_freq_ctx.is_learning = false;
                    }
                    m_curr_freq += m_freq_step;
                    std::cout << "m_curr_freq(" << m_curr_freq << ")" << std::endl;
                }
            }
        }
    }
}
