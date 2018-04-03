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

#ifndef REPORTER_HPP_INCLUDE
#define REPORTER_HPP_INCLUDE

#include <stdint.h>

#include <map>
#include <set>
#include <string>
#include <memory>
#include <vector>

#include "geopm_time.h"

namespace geopm
{
    class IComm;

    class IReporter
    {
        public:
            IReporter() = default;
            virtual ~IReporter() = default;
            virtual std::vector<std::string> signal_names(void) = 0;
            virtual void update(std::vector<double> signal,
                                std::vector<uint64_t> short_region,
                                bool is_epoch,
                                struct geopm_time_s &epoch_time) = 0;
            virtual void generate(const std::string &report_name,
                                  const std::string &profile_name,
                                  const std::string &agent_name,
                                  const std::string &agent_report_header,
                                  const std::string &agent_node_report,
                                  const std::map<uint64_t, std::string> &agent_region_report,
                                  const std::set<std::string> &region_name_set,
                                  std::shared_ptr<IComm> comm) = 0;
    };

    class Reporter : public IReporter
    {
        public:
            Reporter(const std::string &report_name, int verbosity);
            virtual ~Reporter() = default;
            std::vector<std::string> signal_names(void) override;
            void update(std::vector<double> signal,
                        std::vector<uint64_t> short_region,
                        bool is_epoch,
                        struct geopm_time_s &epoch_time) override;
            void generate(const std::string &report_name,
                          const std::string &profile_name,
                          const std::string &agent_name,
                          const std::string &agent_report_header,
                          const std::string &agent_node_report,
                          const std::map<uint64_t, std::string> &agent_region_report,
                          const std::set<std::string> &region_name_set,
                          std::shared_ptr<IComm> comm) override;

        private:
            std::string get_max_memory(void);

            std::string m_report_name;
            double m_mpi_agg_time; // app total mpi-runtime
            double m_ignore_agg_time; // app total ignore-time
            struct geopm_time_s m_app_start_time; // for app total runtime
            double m_counter_energy_start; // for app total energy
            uint64_t m_sample_count;
            uint64_t m_throttle_count; // for app total throttle time



            double m_mpi_sync_time;
            double m_epoch_time;

            double m_hint_ignore_time;



    };
}

#endif
