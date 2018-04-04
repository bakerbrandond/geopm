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

#ifndef APPLICATIONIO_HPP_INCLUDE
#define APPLICATIONIO_HPP_INCLUDE

#include <cstdint>
#include <set>
#include <string>
#include <memory>
#include <vector>
#include <map>

#include "geopm_message.h"
#include "geopm_time.h"

namespace geopm
{
    class IComm;

    class IApplicationIO
    {
        public:
            IApplicationIO() = default;
            virtual ~IApplicationIO() = default;
            virtual bool do_shutdown(void) = 0;
            virtual std::string report_name(void) = 0;
            virtual std::string profile_name(void) = 0;
            virtual std::set<std::string> region_name_set(void) = 0;
            virtual std::vector<std::pair<uint64_t, double> > short_region(void) = 0;
            virtual bool epoch_time(struct geopm_time_s &epoch_time) = 0;
            virtual void update(std::shared_ptr<IComm> comm) = 0;
    };

    class IProfileSampler;
    class ISampleRegulator;
    class RuntimeRegulator;
    class IProfileIOSample;
    class IProfileIORuntime;

    class ApplicationIO : public IApplicationIO
    {
        public:
            ApplicationIO(const std::string &shm_key);
            virtual ~ApplicationIO();
            bool do_shutdown(void) override;
            std::string report_name(void) override;
            std::string profile_name(void) override;
            std::set<std::string> region_name_set(void) override;
            std::vector<std::pair<uint64_t, double> > short_region(void) override;
            bool epoch_time(struct geopm_time_s &epoch_time) override;
            void update(std::shared_ptr<IComm> comm) override;
        private:
            static constexpr size_t M_SHMEM_REGION_SIZE = 12288;

            void connect(void);
            void update_short_regions(std::vector<std::pair<uint64_t, struct geopm_prof_message_s> >::const_iterator prof_sample_begin,
                                      std::vector<std::pair<uint64_t, struct geopm_prof_message_s> >::const_iterator prof_sample_end);
            void update_epoch(std::vector<std::pair<uint64_t, struct geopm_prof_message_s> >::const_iterator prof_sample_begin,
                              std::vector<std::pair<uint64_t, struct geopm_prof_message_s> >::const_iterator prof_sample_end);

            std::unique_ptr<IProfileSampler> m_sampler;
            std::unique_ptr<ISampleRegulator> m_sample_regulator;
            std::vector<std::pair<uint64_t, struct geopm_prof_message_s> > m_prof_sample;
            std::vector<uint64_t> m_region_id;
            // Per rank vector counting number of entries into MPI.
            std::vector<uint64_t> m_num_mpi_enter;
            std::vector<bool> m_is_epoch_changed;
            bool m_do_shutdown;
            bool m_is_connected;
            int m_rank_per_node;
            std::map<uint64_t, std::unique_ptr<RuntimeRegulator> > m_rid_regulator_map;
            std::shared_ptr<IProfileIOSample> m_profile_io_sample;
            std::shared_ptr<IProfileIORuntime> m_profile_io_runtime;
            std::vector<std::pair<uint64_t, double> > m_short_region;
    };
}

#endif
