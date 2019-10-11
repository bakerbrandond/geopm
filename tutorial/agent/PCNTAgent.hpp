/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation
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

#ifndef PCNTAGENT_HPP_INCLUDE
#define PCNTAGENT_HPP_INCLUDE

#include <vector>

#include "geopm/Agent.hpp"
#include "geopm_time.h"

namespace geopm
{
    class PlatformTopo;
    class PlatformIO;
}

/// @brief Agent
class PcntAgent : public geopm::Agent
{
    public:
        PcntAgent();
        virtual ~PcntAgent() = default;
        void init(int level, const std::vector<int> &fan_in, bool is_level_root) override;
        void validate_policy(std::vector<double> &in_policy) const override;
        void split_policy(const std::vector<double> &in_policy,
                          std::vector<std::vector<double> > &out_policy) override;
        bool do_send_policy(void) const override;
        void aggregate_sample(const std::vector<std::vector<double> > &in_sample,
                              std::vector<double> &out_sample) override;
        bool do_send_sample(void) const override;
        void adjust_platform(const std::vector<double> &in_policy) override;
        bool do_write_batch(void) const override;
        void sample_platform(std::vector<double> &out_sample) override;
        void wait(void) override;
        std::vector<std::pair<std::string, std::string> > report_header(void) const override;
        std::vector<std::pair<std::string, std::string> > report_host(void) const override;
        std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > report_region(void) const override;
        std::vector<std::string> trace_names(void) const override;
        void trace_values(std::vector<double> &values) override;
        std::vector<std::function<std::string(double)> > trace_formats(void) const override;

        static std::string plugin_name(void);
        static std::unique_ptr<geopm::Agent> make_plugin(void);
        static std::vector<std::string> policy_names(void);
        static std::vector<std::string> sample_names(void);
    private:
        void load_trace_columns(void);

        // Policy indices; must match policy_names()
        enum m_policy_e {
            M_POLICY_LOW_THRESH,
            M_POLICY_HIGH_THRESH,
            M_POLICY_START_FREQ,
            M_NUM_POLICY
        };
        // Sample indices; must match sample_names()
        enum m_sample_e {
            //M_SAMPLE_USER_PCT,
            //M_SAMPLE_SYSTEM_PCT,
            //M_SAMPLE_IDLE_PCT,

            M_SAMPLE_FREQ,
            M_SAMPLE_SCAL,

            M_NUM_SAMPLE
        };

        // Signals read in sample_platform()
        //enum m_plat_signal_e {
        //    M_PLAT_SIGNAL_USER,
        //    M_PLAT_SIGNAL_SYSTEM,
        //    M_PLAT_SIGNAL_IDLE,
        //    M_PLAT_SIGNAL_NICE,
        //    M_NUM_PLAT_SIGNAL
        //};

        enum m_plat_pp_signal_e {
            //M_PLAT_SIGNAL_UNCORE_FREQUENCY,
            M_PLAT_PP_SIGNAL_UNCORE_F_MAX,
            M_PLAT_PP_SIGNAL_UNCORE_F_MIN,
            M_NUM_PLAT_PP_SIGNAL
        };

        enum m_plat_pc_signal_e {
            M_PLAT_PC_SIGNAL_CORE_FREQUENCY,
            M_PLAT_PC_SIGNAL_SCALABILITY,
            M_NUM_PLAT_PC_SIGNAL
        };

        // Controls written in adjust_platform()
        enum m_plat_control_e {
            M_PLAT_CONTROL_STDOUT,
            M_PLAT_CONTROL_STDERR,
            M_NUM_PLAT_CONTROL
        };

        enum m_plat_pc_control_e {
            M_PLAT_PC_CONTROL_CORE_FREQUENCY,
            M_NUM_PLAT_PC_CONTROL
        };

        enum m_plat_pp_control_e {
            M_PLAT_PP_CONTROL_UNCORE_F_MAX,
            M_PLAT_PP_CONTROL_UNCORE_F_MIN,
            M_NUM_PLAT_PP_CONTROL
        };

        geopm::PlatformIO &m_platform_io;
        const geopm::PlatformTopo &m_platform_topo;

        std::vector<int> m_signal_idx;
        std::vector<int> m_control_idx;
        std::vector<double> m_last_sample;
        std::vector<double> m_last_signal;

        geopm_time_s m_last_wait;
        const double M_WAIT_SEC;

        double m_min_idle;
        double m_max_idle;

        bool m_do_write_batch;



        int m_num_cpu;
        int m_num_package;

        std::vector<std::vector<int> > m_pc_signal_idx;
        std::vector<std::vector<int> > m_pc_control_idx;
        std::vector<std::vector<unsigned long long> > m_pc_last_signal;
        std::vector<std::vector<double> > m_pc_last_sample;

        std::vector<std::vector<int> > m_pp_signal_idx;
        std::vector<std::vector<int> > m_pp_control_idx;
        std::vector<std::vector<int> > m_pp_last_signal;
        std::vector<std::vector<double> > m_pp_last_sample;
        std::vector<std::vector<double> > m_pp_last_scal;

        double m_min_scal;
        double m_max_scal;
        geopm_time_s m_scal_wait;
};

#endif
