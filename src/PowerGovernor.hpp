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

#ifndef POWERGOVERNOR_HPP_INCLUDE
#define POWERGOVERNOR_HPP_INCLUDE

#include <memory>
#include <vector>

namespace geopm
{
    class IPlatformIO;
    class IPlatformTopo;
    template <class type>
    class ICircularBuffer;

    class PowerGovernor
    {
        public:
            PowerGovernor(IPlatformIO &platform_io, IPlatformTopo &platform_topo);
            virtual ~PowerGovernor();
            void init_platform_io(void);
            void sample_platform(void);
            void adjust_platform(double node_power_setting);
            void set_power_bounds(double min_node_power, double max_node_power);
        private:
            IPlatformIO &m_platform_io;
            IPlatformTopo &m_platform_topo;
            int m_pkg_pwr_domain_type;
            int m_num_pkg;
            double m_min_node_power_setting;
            double m_max_node_power_setting;
            int m_dram_sig_idx;
            std::vector<int> m_control_idx;
            std::unique_ptr<ICircularBuffer<double> > m_dram_power_buf;
    };
}

#endif
