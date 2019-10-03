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

#include "PCNTAgent.hpp"

#include <climits>//for string_format_double
#include <cmath>
#include <cassert>
#include <algorithm>
#include <string>

#include "geopm/PluginFactory.hpp"
#include "geopm/PlatformIO.hpp"
#include "geopm/PlatformTopo.hpp"
#include "geopm/Helper.hpp"
#include "geopm/Agg.hpp"

using geopm::Agent;
using geopm::PlatformIO;
using geopm::PlatformTopo;

// Registers this Agent with the Agent factory, making it visible
// to the Controller when the plugin is first loaded.
static void __attribute__((constructor)) example_agent_load(void)
{
    geopm::agent_factory().register_plugin(PcntAgent::plugin_name(),
                                           PcntAgent::make_plugin,
                                           Agent::make_dictionary(PcntAgent::policy_names(),
                                                                  PcntAgent::sample_names()));
}

PcntAgent::PcntAgent()
    : m_platform_io(geopm::platform_io())
    , m_platform_topo(geopm::platform_topo())
    //, m_signal_idx(M_NUM_PLAT_SIGNAL, -1)
    //, m_control_idx(M_NUM_PLAT_CONTROL, -1)
    //, m_last_sample(M_NUM_SAMPLE, NAN)
    //, m_last_signal(M_NUM_PLAT_SIGNAL, NAN)
    , m_last_wait{{0, 0}}
    //, M_WAIT_SEC(1.0)
    , M_WAIT_SEC(0.005)
    , m_do_write_batch(false)
    //, m_min_idle(NAN)
    //, m_max_idle(NAN)


    //Additions made for PCNT
    , m_num_cpu(geopm::platform_topo().num_domain(GEOPM_DOMAIN_CPU))
    , m_num_core(geopm::platform_topo().num_domain(GEOPM_DOMAIN_CORE))
    , m_num_package(geopm::platform_topo().num_domain(GEOPM_DOMAIN_PACKAGE))
    //Per core signals
    , m_pc_signal_idx(M_NUM_PLAT_PC_SIGNAL, std::vector<int>(m_num_cpu))
    , m_pc_control_idx(M_NUM_PLAT_PC_CONTROL, std::vector<int>(m_num_cpu))
    , m_pc_last_signal(M_NUM_PLAT_PC_SIGNAL, std::vector<unsigned long long>(m_num_cpu))
    , m_pc_last_sample(M_NUM_SAMPLE, std::vector<double>(m_num_cpu))
    //Per package signals
    , m_pp_signal_idx(M_NUM_PLAT_PP_SIGNAL, std::vector<int>(m_num_package))
    , m_pp_control_idx(M_NUM_PLAT_PP_CONTROL, std::vector<int>(m_num_package))
    , m_pp_last_signal(M_NUM_PLAT_PP_SIGNAL, std::vector<int>(m_num_package))
    , m_pp_last_sample(M_NUM_SAMPLE, std::vector<double>(m_num_package))
{
    geopm_time(&m_last_wait);
}

// Push signals and controls for future batch read/write
void PcntAgent::init(int level, const std::vector<int> &fan_in, bool is_level_root)
{
    // all signals and controls will be at board domain
    int board = GEOPM_DOMAIN_BOARD;
    // push signals
    //m_signal_idx[M_PLAT_SIGNAL_USER] = m_platform_io.push_signal("USER_TIME", board, 0);
    //m_signal_idx[M_PLAT_SIGNAL_SYSTEM] = m_platform_io.push_signal("SYSTEM_TIME", board, 0);
    //m_signal_idx[M_PLAT_SIGNAL_IDLE] = m_platform_io.push_signal("IDLE_TIME", board, 0);
    //m_signal_idx[M_PLAT_SIGNAL_NICE] = m_platform_io.push_signal("NICE_TIME", board, 0);
    // push controls
    //m_control_idx[M_PLAT_CONTROL_STDOUT] = m_platform_io.push_control("STDOUT", board, 0);
    //m_control_idx[M_PLAT_CONTROL_STDERR] = m_platform_io.push_control("STDERR", board, 0);


    //Additions made for PCNT
    int package = GEOPM_DOMAIN_PACKAGE;
    int cpu = GEOPM_DOMAIN_CPU;
    int core = GEOPM_DOMAIN_CORE;
    //per package
    for(int i=0; i<m_num_package; i++){
        // push signals
        m_pp_signal_idx[M_PLAT_PP_SIGNAL_UNCORE_F_MAX][i] = m_platform_io.push_signal("MSR::UNCORE_RATIO_LIMIT:MAX_RATIO", package, i);
        m_pp_signal_idx[M_PLAT_PP_SIGNAL_UNCORE_F_MIN][i] = m_platform_io.push_signal("MSR::UNCORE_RATIO_LIMIT:MIN_RATIO", package, i);
        //m_pp_signal_idx[M_PLAT_PP_SIGNAL_UNCORE_FREQUENCY][i] = m_platform_io.push_signal("MSR::RESEARCH_UNCORE_PERF_STATUS:CURRENT_CLM_RATIO", package, i);
        // push controls
        //m_pp_control_idx[M_PLAT_PP_CONTROL_UNCORE_F_MAX][i] = m_platform_io.push_signal("MSR::UNCORE_RATIO_LIMIT:MAX_RATIO", package, i);
        //m_pp_control_idx[M_PLAT_PP_CONTROL_UNCORE_F_MIN][i] = m_platform_io.push_signal("MSR::UNCORE_RATIO_LIMIT:MIN_RATIO", package, i);
    }

    //per cpu
    for(int i=0; i<m_num_cpu; i++){
        // push signals

        //Replace this with using ACNT and MCNT to figure out average frequency over a period of time.  Or perf_status.  But this will work for the quick version
        m_pc_signal_idx[M_PLAT_PC_SIGNAL_CORE_FREQUENCY][i] = m_platform_io.push_signal("FREQUENCY", cpu, i);
        m_pc_signal_idx[M_PLAT_PC_SIGNAL_PCNT][i] = m_platform_io.push_signal("MSR::RESEARCH_PPERF:PCNT", cpu, i);
        m_pc_signal_idx[M_PLAT_PC_SIGNAL_ACNT][i] = m_platform_io.push_signal("MSR::APERF:ACNT", cpu, i);
    }

    //per core
    for(int i=0; i<m_num_core; i++){
        // push controls
        m_pc_control_idx[M_PLAT_PC_CONTROL_CORE_FREQUENCY][i] = m_platform_io.push_control("FREQUENCY", core, i);
    }

}

// Validate incoming policy and configure default policy requests.
void PcntAgent::validate_policy(std::vector<double> &in_policy) const
{
    assert(in_policy.size() == M_NUM_POLICY);
    if (std::isnan(in_policy[M_POLICY_LOW_THRESH])) {
        in_policy[M_POLICY_LOW_THRESH] = 0.80;
    }
    if (std::isnan(in_policy[M_POLICY_HIGH_THRESH])) {
        in_policy[M_POLICY_HIGH_THRESH] = 0.90;
    }
    if (std::isnan(in_policy[M_POLICY_START_FREQ])) {
        in_policy[M_POLICY_START_FREQ] = 2000000000.000000; // todo sticker
    }
}

// Distribute incoming policy to children
void PcntAgent::split_policy(const std::vector<double>& in_policy,
                                std::vector<std::vector<double> >& out_policy)
{
    assert(in_policy.size() == M_NUM_POLICY);
    for (auto &child_pol : out_policy) {
        child_pol = in_policy;
    }
}

// Indicate whether to send the policy down to children
bool PcntAgent::do_send_policy(void) const
{
    return m_do_write_batch;
}

// Aggregate average utilization samples from children
void PcntAgent::aggregate_sample(const std::vector<std::vector<double> > &in_sample,
                                    std::vector<double>& out_sample)
{
    //assert(out_sample.size() == M_NUM_SAMPLE);
    //std::vector<double> child_sample(in_sample.size());
    //for (size_t sample_idx = 0; sample_idx < M_NUM_SAMPLE; ++sample_idx) {
    //    for (size_t child_idx = 0; child_idx < in_sample.size(); ++child_idx) {
    //        child_sample[child_idx] = in_sample[child_idx][sample_idx];
    //    }
    //    out_sample[sample_idx] = geopm::Agg::average(child_sample);
    //}
}

// Indicate whether to send samples up to the parent
bool PcntAgent::do_send_sample(void) const
{
    return true;
}

// Set up idle percentage to print to either standard out or standard error
void PcntAgent::adjust_platform(const std::vector<double>& in_policy)
{
    assert(in_policy.size() == M_NUM_POLICY);
    // Check for NAN to set default values for policy
    double low_thresh = in_policy[M_POLICY_LOW_THRESH];
    double high_thresh = in_policy[M_POLICY_HIGH_THRESH];
    double start_freq = in_policy[M_POLICY_START_FREQ];
    if (std::isnan(low_thresh)) {
        low_thresh = 0.80;
    }
    if (std::isnan(high_thresh)) {
        high_thresh = 0.90;
    }
    if (std::isnan(start_freq)) {
        start_freq = 2000000000.000000;
    }

    //Added for PCNT
    double scalability = 0.0;
    double freq_actual = -1;
    double freq_request = -1;
    double curr_freq = -1;
    double m_freq_max = 2800000000.000000;
    double m_freq_min = 1000000000.000000;
    for(int c=0; c<m_num_core; c++){
        scalability = m_pc_last_sample[M_SAMPLE_SCAL][c];
        curr_freq = m_pc_last_sample[M_SAMPLE_FREQ][c];

        if(curr_freq == 0.0){
            curr_freq = start_freq;
            scalability = 1.0;
        }

        if (!std::isnan(scalability)) {
            if (scalability < low_thresh) {
                //decrease core freq by 100MHz
                freq_request = curr_freq - 100000000.000000;
                //printf("scalability %f is < %f.  Decrement freq\n", scalability, low_thresh);
            }
            else if (scalability > high_thresh) {
                //increase core freq by 100MHz
                freq_request = curr_freq + 100000000.000000;
                //printf("scalability %f is > %f.  Increment freq\n", scalability, high_thresh);
            }
            else {
                freq_request = curr_freq;
            }
        }

        //printf("freq_req for core%d is: %f, previously: %f\n", c, freq_request, m_pc_last_sample[M_SAMPLE_FREQ][c]);
        double clamp_freq = NAN;
        if (freq_request > m_freq_max) {
            freq_actual = m_freq_max;
        }
        else if (freq_request < m_freq_min) {
            freq_actual = m_freq_min;
        }
        else {
            freq_actual = freq_request;
        }

        if (freq_actual != curr_freq) {
            //printf("\t\tFreq request for core%d: new:%f, curr: %f. Scalability: %f, thresh low:%f, thresh high:%f\n", c, freq_actual, curr_freq, scalability, low_thresh, high_thresh);
            m_platform_io.adjust(m_pc_control_idx[M_PLAT_PC_CONTROL_CORE_FREQUENCY][c], freq_actual);
            m_do_write_batch = true;
        } else {
            m_do_write_batch = false;
        }

    }

}

// If idle percent had a valid value, execute the print
bool PcntAgent::do_write_batch(void) const
{
    return m_do_write_batch;
}

// Read signals from the platform and calculate samples to be sent up
void PcntAgent::sample_platform(std::vector<double> &out_sample)
{
    assert(out_sample.size() == M_NUM_SAMPLE);
    // Collect latest times from platform signals
    double total = 0.0;
    //for (auto signal_idx : m_signal_idx) {
    //    m_last_signal[signal_idx] = m_platform_io.sample(signal_idx);
    //    total += m_last_signal[signal_idx];
    //}

    for(int s=0; s<M_NUM_PLAT_PP_SIGNAL; s++){
        for(int c=0; c<m_num_package; c++){
            m_pp_last_signal[s][c] = m_platform_io.sample(m_pp_signal_idx[s][c]);
        }
    }

    // Update samples
    for(int s=0; s<M_NUM_PLAT_PC_SIGNAL; s++){
        for(int c=0; c<m_num_cpu; c++){
            if(s == M_PLAT_PC_SIGNAL_ACNT) {
                //ANCT Delta = (acnt - prev acnt)
                m_pc_last_signal[s][c] = m_platform_io.sample(m_pc_signal_idx[s][c]);

                m_pc_last_sample[M_SAMPLE_ACNT_DELTA][c] = m_pc_last_signal[s][c] - m_pc_last_sample[M_SAMPLE_ACNT][c];
                m_pc_last_sample[M_SAMPLE_ACNT][c] = m_pc_last_signal[s][c];

            } else if(s == M_PLAT_PC_SIGNAL_PCNT) {
                //PCNT Delta = (pcnt - prev pcnt).
                m_pc_last_signal[s][c] = m_platform_io.sample(m_pc_signal_idx[s][c]);

                m_pc_last_sample[M_SAMPLE_PCNT_DELTA][c] = m_pc_last_signal[s][c] - m_pc_last_sample[M_SAMPLE_PCNT][c];
                m_pc_last_sample[M_SAMPLE_PCNT][c] = m_pc_last_signal[s][c];
            } else {
                m_pc_last_sample[M_SAMPLE_FREQ][c] = m_platform_io.sample(m_pc_signal_idx[s][c]);
            }
        }
    }

    unsigned long long acnt;
    unsigned long long pcnt;
    for(int c=0; c<m_num_cpu; c++){
        //acnt = m_pc_last_signal[M_PLAT_PC_SIGNAL_ACNT][c]; //m_platform_io.sample(m_pc_signal_idx[M_PLAT_PC_SIGNAL_ACNT][c]);
        //pcnt = m_pc_last_signal[M_PLAT_PC_SIGNAL_PCNT][c]; //m_platform_io.sample(m_pc_signal_idx[M_PLAT_PC_SIGNAL_PCNT][c]);

        acnt = m_pc_last_sample[M_SAMPLE_ACNT_DELTA][c]; //m_platf(rm_io.sample(m_pc_signal_idx[M_PLAT_PC_SIGNAL_ACNT][c]);
        pcnt = m_pc_last_sample[M_SAMPLE_PCNT_DELTA][c]; //m_platform_io.sample(m_pc_signal_idx[M_PLAT_PC_SIGNAL_PCNT][c]);

        m_pc_last_sample[M_SAMPLE_SCAL][c] = (double)pcnt/(double)acnt;

        // Update min and max for the report
        if (std::isnan(m_min_scal) || m_pc_last_sample[M_SAMPLE_SCAL][c] < m_min_scal) {
            m_min_scal = m_pc_last_sample[M_SAMPLE_SCAL][c];
        } else if (std::isnan(m_max_scal) || m_max_scal < m_pc_last_sample[M_SAMPLE_SCAL][c]) {
            m_max_scal = m_pc_last_sample[M_SAMPLE_SCAL][c];
        }
    }

}

// Wait for the remaining cycle time to keep Controller loop cadence at 1 second
void PcntAgent::wait(void)
{
    geopm_time_s current_time;
    do {
        geopm_time(&current_time);
    }
    while(geopm_time_diff(&m_last_wait, &current_time) < M_WAIT_SEC);
    geopm_time(&m_last_wait);
}

// Adds the wait time to the top of the report
std::vector<std::pair<std::string, std::string> > PcntAgent::report_header(void) const
{
    return {{"Wait time (sec)", std::to_string(M_WAIT_SEC)}};
}

// Adds min and max idle percentage to the per-node section of the report
std::vector<std::pair<std::string, std::string> > PcntAgent::report_host(void) const
{
    return {
        {"Lowest scalability %", std::to_string(m_min_scal)},
        {"Highest scalability %", std::to_string(m_max_scal)}
    };
}

// This Agent does not add any per-region details
std::map<uint64_t, std::vector<std::pair<std::string, std::string> > > PcntAgent::report_region(void) const
{
    return {};
}

// Adds trace columns samples and signals of interest
std::vector<std::string> PcntAgent::trace_names(void) const
{
    //return {"user_percent", "system_percent", "idle_percent",
    //        "user", "system", "idle", "nice"};

    std::vector<std::string> names;
    std::string freq_s, scal_s;

    for(int i=0; i<m_num_core; i++){
        //sprintf(freq_s, "frequency_core%d", i);
        freq_s = "FREQUENCY_CORE" + std::to_string(i);
        names.push_back(freq_s);
    }

    for(int i=0; i<m_num_core; i++){
        //sprintf(scal_s, "scalability_core%d", i);
        scal_s = "SCALABILITY_CORE" + std::to_string(i);
        names.push_back(scal_s);
    }

    //TODO: add socket power dump

    return names;

}

// Updates the trace with values for samples and signals from this Agent
void PcntAgent::trace_values(std::vector<double> &values)
{
    for(int i=0; i<m_num_core; i++){
        values[i] = m_pc_last_sample[M_SAMPLE_FREQ][i];
    }
    int c = 0;
    for(int i=m_num_core; i<m_num_core*2; i++){
        values[i] = m_pc_last_sample[M_SAMPLE_SCAL][c];
        c++;
    }
}

// Name used for registration with the Agent factory
std::string PcntAgent::plugin_name(void)
{
    return "pcntAgent";
}

// Used by the factory to create objects of this type
std::unique_ptr<Agent> PcntAgent::make_plugin(void)
{
    return geopm::make_unique<PcntAgent>();
}

// Describes expected policies to be provided by the resource manager or user
std::vector<std::string> PcntAgent::policy_names(void)
{
    return {"LOW_THRESHOLD", "HIGH_THRESHOLD", "START_FREQUENCY"};
}

// Describes samples to be provided to the resource manager or user
std::vector<std::string> PcntAgent::sample_names(void)
{
    //return {"USER_PERCENT", "SYSTEM_PERCENT", "IDLE_PERCENT", "CORE FREQUENCY", "SCALABILITY"};
    return {"CORE FREQUENCY", "CORE PCNT", "CORE ACNT", "PCNT DELTA", "ACNT DELTA", "SCALABILITY"};
}

// todo address
std::string string_format_double(double signal)
{
    char result[NAME_MAX];
    snprintf(result, NAME_MAX, "%.16g", signal);
    return result;
}

std::vector<std::function<std::string(double)> > PcntAgent::trace_formats(void) const
{
    std::vector<std::function<std::string(double)> > ret(trace_names().size(), /*geopm::*/string_format_double);
    return ret;
}
