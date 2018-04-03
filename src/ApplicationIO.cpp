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

#include "ApplicationIO.hpp"
#include "PlatformIO.hpp"
#include "ProfileSampler.hpp"
#include "SampleRegulator.hpp"
#include "RuntimeRegulator.hpp"
#include "ProfileIOSample.hpp"
#include "ProfileIORuntime.hpp"
#include "ProfileIOGroup.hpp"
#include "Helper.hpp"
#include "config.h"

#ifdef GEOPM_HAS_XMMINTRIN
#include <xmmintrin.h>
#endif

namespace geopm
{
    ApplicationIO::ApplicationIO(const std::string &shm_key)
        : m_sampler(new ProfileSampler(M_SHMEM_REGION_SIZE))
        , m_sample_regulator(nullptr)
        , m_do_shutdown(false)
        , m_is_connected(false)
        , m_rank_per_node(-1)  /// @todo
    {

    }

    ApplicationIO::~ApplicationIO()
    {

    }

    void ApplicationIO::connect(void)
    {
        if (!m_is_connected) {
            m_sampler->initialize();
            m_rank_per_node = m_sampler->rank_per_node();
            m_prof_sample.resize(m_sampler->capacity());
            std::vector<int> cpu_rank = m_sampler->cpu_rank();
            m_profile_io_sample = std::make_shared<ProfileIOSample>(cpu_rank);
            m_profile_io_runtime = std::make_shared<ProfileIORuntime>(cpu_rank);
            platform_io().register_iogroup(geopm::make_unique<ProfileIOGroup>(m_profile_io_sample,
                                                                              m_profile_io_runtime));
            m_is_connected = true;
        }
    }

    bool ApplicationIO::do_sample(void)
    {
        /// @todo implement me
        return false;
    }

    bool ApplicationIO::do_shutdown(void)
    {
        return m_sampler->do_shutdown();
    }

    std::string ApplicationIO::report_name(void)
    {
        /// @todo implement me
        return "";
    }

    std::string ApplicationIO::profile_name(void)
    {
        /// @todo implement me
        return "";
    }

    std::set<std::string> ApplicationIO::region_name_set(void)
    {
        /// @todo implement me
        return {};
    }

    void ApplicationIO::update_short_regions(std::vector<std::pair<uint64_t, struct geopm_prof_message_s> >::const_iterator prof_sample_begin,
                                             std::vector<std::pair<uint64_t, struct geopm_prof_message_s> >::const_iterator prof_sample_end)
    {
        /// @todo implement me
    }

    void ApplicationIO::update_epoch(std::vector<std::pair<uint64_t, struct geopm_prof_message_s> >::const_iterator prof_sample_begin,
                                     std::vector<std::pair<uint64_t, struct geopm_prof_message_s> >::const_iterator prof_sample_end)
    {
        /// @todo implement me
    }

    std::vector<uint64_t> ApplicationIO::short_region(void)
    {
        /// @todo implement me
        return {};
    }

    bool ApplicationIO::epoch_time(struct geopm_time_s &time)
    {
        /// @todo implement me
        return false;
    }

    void ApplicationIO::update(std::shared_ptr<IComm> comm)
    {
        size_t length = 0; /// @todo fix
        m_sampler->sample(m_prof_sample, length, comm);
        m_profile_io_sample->update(m_prof_sample.cbegin(), m_prof_sample.cbegin() + length);
        //m_profile_io_runtime->update(m_prof_sample.cbegin(), m_prof_sample.cbegin() + length);
        update_short_regions(m_prof_sample.cbegin(), m_prof_sample.cbegin() + length);
        update_epoch(m_prof_sample.cbegin(), m_prof_sample.cbegin() + length);
    }
}
