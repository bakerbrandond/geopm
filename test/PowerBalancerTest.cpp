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

#include <memory>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "PowerBalancer.hpp"
#include "Helper.hpp"

using geopm::PowerBalancer;
using ::testing::_;
using ::testing::Return;
using ::testing::Sequence;

class PowerBalancerTest : public ::testing::Test
{
    public:
        void SetUp(void);

    protected:
        double calc_rt(void);
        double calc_rt(double power_limit);

        const double M_POWER_CAP = 300;
        const double M_TARGET_EPSILON = 0.03;
        const double M_TRIAL_DELTA = 1.0;
        const size_t M_NUM_SAMPLE = 3;
        std::unique_ptr<PowerBalancer> m_balancer;
};

void PowerBalancerTest::SetUp(void)
{
    m_balancer = geopm::make_unique<PowerBalancer>(M_TARGET_EPSILON, M_TRIAL_DELTA, M_NUM_SAMPLE);
    m_balancer->power_cap(M_POWER_CAP);
}

double PowerBalancerTest::calc_rt(void)
{
    return calc_rt(m_balancer->power_limit());
}

double PowerBalancerTest::calc_rt(double power_limit)
{
    return 1 / power_limit * 1e3;
}


TEST_F(PowerBalancerTest, power_cap)
{
    double cap = 999;
    m_balancer->power_cap(cap);
    EXPECT_EQ(cap, m_balancer->power_cap());
    EXPECT_EQ(cap, m_balancer->power_limit());
}

TEST_F(PowerBalancerTest, is_runtime_stable)
{
    size_t count = 1;
    while (!m_balancer->is_runtime_stable(calc_rt())) {
        ++count;
    }
    EXPECT_EQ(M_NUM_SAMPLE, count);
}

TEST_F(PowerBalancerTest, is_target_met)
{
    while (!m_balancer->is_runtime_stable(calc_rt())) {
    }

    int exp_step = 20;/// add another test where step is not a whole number (target_power ! multiple of delta)
    double target_power = M_POWER_CAP - exp_step * M_TRIAL_DELTA;
    double target_runtime = calc_rt(target_power);
    m_balancer->target_runtime(target_runtime);

    bool is_target_met = false;
    std::cout << "target_power: " << target_power << " target_runtime: " << target_runtime << std::endl;
    while (!is_target_met) {
        double curr_rt = calc_rt();
        is_target_met = m_balancer->is_target_met(curr_rt);
        std::cout << "power_limit: " << m_balancer->power_limit() << " curr_rt: " << curr_rt << std::endl;
        if (!is_target_met) {
            while (!m_balancer->is_runtime_stable(calc_rt())) {
            }
        }
    }
    //// new test expect power_limit to be > target_power and power_limit < target_power + delta
    EXPECT_EQ(target_power, m_balancer->power_limit());
}
