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
#include <cmath>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "PowerBalancerAgent.hpp"
#include "ReplayPlatformIO.hpp"
#include "MockPlatformIO.hpp"
#include "MockPlatformTopo.hpp"
#include "Helper.hpp"

using geopm::PowerBalancerAgent;
using geopm::IPlatformTopo;
using testing::_;
using testing::Return;

/// Tests of the power balancer algorithms in a tree with 2 levels and
/// fan out of 2 at each level. Nodes are numbered as follows:
///
///      0
///    /   \     level 1
///   0     2
///  / \   / \   level 0
/// 0   1 2   3  leaf
///
/// Different set up fixures are provided for different levels/nodes to set up
/// expectations for platform_io.
class PowerBalancerAgentTest : public ::testing::Test
{
    protected:
        void SetUp(void);
        /// set expectations for platform io at the leaf
        void leaf_setup(void);
        void set_sample_values(double runtime, double count,
                               double power_pkg, double power_dram);
        void check_result(const std::vector<double> &expected, const std::vector<double> &result);

        enum m_platform_idx_e {
            M_SIGNAL_EPOCH_RUNTIME,
            M_SIGNAL_EPOCH_COUNT,
            M_SIGNAL_POWER_PACKAGE,
            M_SIGNAL_POWER_DRAM,
        };

        MockPlatformIO m_platform_io;
        MockPlatformTopo m_platform_topo;
        std::unique_ptr<PowerBalancerAgent> m_agent;
        std::vector<int> m_fan_in;
        int m_num_policy = 0;
        int m_num_sample = 0;
        int m_min_num_converged = 15;  // this is hard coded in the agent; determines how many times we need to sample
        int m_ascend_period = 10;      // also hardcoded; determines how many times we need to ascend
        double m_min_power = 50;
        double m_max_power = 300;
};

void PowerBalancerAgentTest::SetUp()
{
    m_fan_in = {2, 2};

    EXPECT_CALL(m_platform_io, read_signal("POWER_PACKAGE_MIN", IPlatformTopo::M_DOMAIN_PACKAGE, 0))
        .WillOnce(Return(m_min_power));
    EXPECT_CALL(m_platform_io, read_signal("POWER_PACKAGE_MAX", IPlatformTopo::M_DOMAIN_PACKAGE, 0))
        .WillOnce(Return(m_max_power));
    m_agent = geopm::make_unique<PowerBalancerAgent>(m_platform_io, m_platform_topo);
    m_num_policy = PowerBalancerAgent::policy_names().size();
    m_num_sample = PowerBalancerAgent::sample_names().size();
}

void PowerBalancerAgentTest::leaf_setup(void)
{
    // signals
    EXPECT_CALL(m_platform_io, push_signal("EPOCH_RUNTIME", IPlatformTopo::M_DOMAIN_BOARD, 0))
        .WillOnce(Return(M_SIGNAL_EPOCH_RUNTIME));
    EXPECT_CALL(m_platform_io, push_signal("EPOCH_COUNT", IPlatformTopo::M_DOMAIN_BOARD, 0))
        .WillOnce(Return(M_SIGNAL_EPOCH_COUNT));
    EXPECT_CALL(m_platform_io, push_signal("POWER_PACKAGE", IPlatformTopo::M_DOMAIN_BOARD, 0))
        .WillOnce(Return(M_SIGNAL_POWER_PACKAGE));
    EXPECT_CALL(m_platform_io, push_signal("POWER_DRAM", IPlatformTopo::M_DOMAIN_BOARD, 0))
        .WillOnce(Return(M_SIGNAL_POWER_DRAM));

    // controls
    int num_power_domain = 2;
    int power_domain_type = IPlatformTopo::M_DOMAIN_PACKAGE;
    EXPECT_CALL(m_platform_io, control_domain_type("POWER_PACKAGE"))
        .WillOnce(Return(power_domain_type));
    EXPECT_CALL(m_platform_topo, num_domain(power_domain_type))
        .WillOnce(Return(num_power_domain));
    EXPECT_CALL(m_platform_io, push_control("POWER_PACKAGE", power_domain_type, _)).Times(num_power_domain);
}

// set the values to be returned for sample
void PowerBalancerAgentTest::set_sample_values(double runtime, double count,
                                               double power_pkg, double power_dram)
{
    EXPECT_CALL(m_platform_io, sample(M_SIGNAL_EPOCH_RUNTIME))
        .WillOnce(Return(runtime));
    EXPECT_CALL(m_platform_io, sample(M_SIGNAL_EPOCH_COUNT))
        .WillOnce(Return(count));
    EXPECT_CALL(m_platform_io, sample(M_SIGNAL_POWER_PACKAGE))
        .WillOnce(Return(power_pkg));
    EXPECT_CALL(m_platform_io, sample(M_SIGNAL_POWER_DRAM))
        .WillOnce(Return(power_dram));
}

// check if containers are equal, including NAN
void PowerBalancerAgentTest::check_result(const std::vector<double> &expected,
                                          const std::vector<double> &result)
{
    ASSERT_EQ(expected.size(), result.size());
    for (size_t ii = 0; ii < expected.size(); ++ii) {
        if (std::isnan(expected[ii])) {
            EXPECT_TRUE(std::isnan(result[ii]));
        }
        else {
            EXPECT_EQ(expected[ii], result[ii]);
        }
    }
}

TEST_F(PowerBalancerAgentTest, sample_platform)
{
    leaf_setup();
    m_agent->init(0, m_fan_in, false);

    // todo: make sure this is safe to call on its own and doesn't depend on state
    // being set in other calls
    // policy shouldn't matter for sampling?

    std::vector<double> result(m_num_sample, NAN);
    std::vector<double> expected(m_num_sample, NAN);

    double epoch_time = 12;
    double epoch_count = 0.0;
    double pkg = 9.9;
    double dram = 8.8;

    // step 0 - no update
    {
        set_sample_values(epoch_time, epoch_count, pkg, dram);
        EXPECT_FALSE(m_agent->sample_platform(result));
        check_result(expected, result);
    }
    // step 1 - new epoch count, not converged
    {
        epoch_count += 1.0;
        set_sample_values(epoch_time, epoch_count, pkg, dram);
        EXPECT_FALSE(m_agent->sample_platform(result));
        check_result(expected, result);
    }

    // take more samples; still no return values
    for (int num_sample = 2; num_sample <= m_min_num_converged; ++num_sample) {
        epoch_count += 1.0;
        set_sample_values(epoch_time, epoch_count, pkg, dram);
        EXPECT_FALSE(m_agent->sample_platform(result)) << " failed when num sample was " << num_sample;
        check_result(expected, result);
    }

    // step 2 - new epoch count, converged
    {
        /// todo: check that this is the right return value for power (idx 1)
        expected = {epoch_time, pkg + dram, true}; // todo: make this more flexible in case num samples changes
        epoch_count += 1.0;
        set_sample_values(epoch_time, epoch_count, pkg, dram);
        EXPECT_TRUE(m_agent->sample_platform(result));
        check_result(expected, result);
    }

    // step 3 - new epoch count, stays converged
    {
        expected = {epoch_time, pkg + dram, true};
        epoch_count += 1.0;
        set_sample_values(epoch_time, epoch_count, pkg, dram);
        EXPECT_TRUE(m_agent->sample_platform(result));
        check_result(expected, result);
    }

}

// multiple ascend and descend where sample for is_converged is always true
TEST_F(PowerBalancerAgentTest, ascend_descend)
{
    int level = 1;
    m_agent->init(level, m_fan_in, false);

    std::vector<double> in_budget = {200};
    ASSERT_EQ(in_budget.size(), (size_t) m_num_policy);
    std::vector<std::vector<double> > expected;
    std::vector<std::vector<double> >result(m_fan_in[level], std::vector<double>(m_num_policy, NAN));
    // todo fix values here
    std::vector<double> expected_sample {4, 3.5, true};
    std::vector<double> out_sample(m_num_sample, NAN);

    // initial, split budget evenly
    // the input is average, not total, so all children will just get the same budget as in the input
    expected = { in_budget, in_budget };
    ASSERT_EQ(expected.size(), (size_t) m_fan_in[level]);

    EXPECT_TRUE(m_agent->descend(in_budget, result));
    for (size_t child = 0; child < result.size(); ++child) {
        check_result(expected[child], result[child]);
    }
    EXPECT_TRUE(m_agent->ascend({ { 1,2,true}, {4,5,true} }, out_sample));
    check_result(expected_sample, out_sample);

    for (int aa = 1; aa <= m_min_num_converged; ++aa) {
        m_agent->ascend({ { (double)aa,2,true}, {4,5,true} }, out_sample);
        EXPECT_FALSE(m_agent->descend(in_budget, result));
        // expected remains the same; if returned false; output should not have changed
        for (size_t child = 0; child < result.size(); ++child) {
            check_result(expected[child], result[child]);
        }
        check_result(expected_sample, out_sample);
    }

    // TODO: only hit updated runtimes branch every m_ascend_period calls to ascend
    // only hit out of range branch in decend_updated_runtimes after m_min_num_converged calls to descend_updated_runtimes
    // not sure why condition is m_ascend_count == 1
    // TODO: need to call sample_platform also; but this is supposed to be non-leaf node

    m_agent->ascend({ { 1,2,true}, {4,5,true} }, out_sample);
    check_result(expected_sample, out_sample);

    // updated runtimes
    expected = { {190}, {210} };
    ASSERT_EQ(expected.size(), (size_t) m_fan_in[level]);
    EXPECT_TRUE(m_agent->descend(in_budget, result));
    for (size_t child = 0; child < result.size(); ++child) {
        check_result(expected[child], result[child]);
    }


    for (int aa = 1; aa <= m_min_num_converged +1; ++aa) {
        std::cout << aa << std::endl;
        m_agent->ascend({ { 6,7,true}, {8,9,true} }, out_sample);
        EXPECT_FALSE(m_agent->descend(in_budget, result));
        // expected remains the same; if returned false; output should not have changed
        for (size_t child = 0; child < result.size(); ++child) {
            check_result(expected[child], result[child]);
        }
        //check_result(expected_sample, out_sample);
    }


    expected = { {189}, {211} };
    ASSERT_EQ(expected.size(), (size_t) m_fan_in[level]);
    EXPECT_TRUE(m_agent->descend(in_budget, result));

    for (size_t child = 0; child < result.size(); ++child) {
        check_result(expected[child], result[child]);
    }

    /*
    expected_sample = {8, 8, true};

    expected = { {189}, {211} };
    for (size_t child = 0; child < result.size(); ++child) {
        check_result(expected[child], result[child]);
    }
    */

    //expected_sample = {3, 3.5, true};
    //EXPECT_TRUE(m_agent->ascend({ { 2,3,true}, {3,4,true} }, out_sample));
    //check_result(expected_sample, out_sample);

    //EXPECT_TRUE(m_agent->descend(in_budget, result));
    //for (size_t child = 0; child < result.size(); ++child) {
    //    check_result(expected[child], result[child]);
    // }

}

TEST_F(PowerBalancerAgentTest, descend_updated_budget)
{

}
// TEST LIST
// equal epoch keeps budget equal
// longer epoch nodes get more power
// changing epoch re-adjusts budget

// Q: does is_root parameter refer to root of this level or root of tree?
// A: this is whether num level controlled is > level
// what does is_root mean at level 0? - this is about the links about the node with its parent
TEST_F(PowerBalancerAgentTest, node_2_level_0_setup)
{
    // tree should not use platform io
    EXPECT_CALL(m_platform_io, push_signal(_, _, _)).Times(0);
    EXPECT_CALL(m_platform_io, push_control(_, _, _)).Times(0);

    m_agent->init(1, m_fan_in, false);
}

TEST_F(PowerBalancerAgentTest, node_0_level_1_setup)
{
    // tree should not use platform io
    EXPECT_CALL(m_platform_io, push_signal(_, _, _)).Times(0);
    EXPECT_CALL(m_platform_io, push_control(_, _, _)).Times(0);

    m_agent->init(1, m_fan_in, true);
}

TEST_F(PowerBalancerAgentTest, split_budget) {
    // tree should not use platform io
    EXPECT_CALL(m_platform_io, push_signal(_, _, _)).Times(0);
    EXPECT_CALL(m_platform_io, push_control(_, _, _)).Times(0);

    m_fan_in = {3, 3};
    m_agent->init(1, m_fan_in, true);

    // initial split
    std::vector<double> expected {200, 200, 200};
    std::vector<double> result;
    result = m_agent->split_budget(200);
    EXPECT_EQ(expected, result);

    // first
    m_agent->inject_runtimes({20, 15, 30}, {NAN, NAN, NAN});
    m_agent->inject_budgets({200, 200, 200}, {NAN, NAN, NAN});
    expected = {210, 190, 210};
    result = m_agent->split_budget(200);
    EXPECT_EQ(expected, result);

    // last
    for (int i = 0; i < 15; ++i) {
        m_agent->inject_runtimes({25, 15, 30}, {24, 16, 28});
    }
    m_agent->inject_runtimes({24, 16, 28}, {23, 17, 26});
    m_agent->inject_budgets({200, 200, 200}, {225, 150, 225});
    expected = {185, 215, 160}; // ?
    result = m_agent->split_budget(200);
    EXPECT_EQ(expected, result);

    m_agent->inject_runtimes({23, 17, 26}, {22, 18, 24});
    m_agent->inject_budgets({225, 150, 225}, { 220, 160, 220});
    expected = {185, 215, 160}; // ?
    result = m_agent->split_budget(200);
    EXPECT_EQ(expected, result);


}

TEST_F(PowerBalancerAgentTest, something)
{
    ReplayPlatformIO replayer("pio_record-mr-fusion8");
}
