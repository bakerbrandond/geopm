/*
 * Copyright (c) 2015, 2016, 2017, Intel Corporation
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

#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "geopm.h"
#include "geopm_hash.h"
#include "geopm_time.h"
#include "geopm_sched.h"
#include "Controller.hpp"
#include "MockComm.hpp"
#include "MockGlobalPolicy.hpp"
#include "MockProfileSampler.hpp"
#include "MockControlMessage.hpp"

ACTION_P(SetArg0ToVoidP, value)
{
    arg0 = static_cast<void  *>(value);
}

void MockComm::config_in_comm(MockComm *ppn1)
{
    EXPECT_CALL(*this, split(testing::Matcher<const std::string&>(testing::_), testing::_))
        .WillOnce(testing::Return(ppn1));
}

void MockComm::config_ppn1(std::string header, int num_rank, int rank)
{
    m_num_rank = num_rank;
    m_rank = rank;
    m_len = header.length();
    header.copy(m_header, m_len, 0);
    EXPECT_CALL(*this, num_rank()).WillRepeatedly(testing::Return(m_num_rank));
    EXPECT_CALL(*this, rank()).WillRepeatedly(testing::Return(m_rank));
    EXPECT_CALL(*this, barrier()).WillRepeatedly(testing::Return());
    EXPECT_CALL(*this, broadcast(testing::_, testing::_, testing::_))
        .WillOnce(testing::Return())
        .WillOnce(SetArg0ToVoidP(&m_len))
        .WillOnce(SetArg0ToVoidP(m_header));
}

void MockGlobalPolicy::config(int mode, int budget, std::string tree_decider, std::string leaf_decider)
{
    m_mode = mode;
    m_power_budget_watts = budget;
    m_tree_decider = tree_decider;
    m_leaf_decider = leaf_decider;

    switch (m_mode) {
        case GEOPM_POLICY_MODE_TDP_BALANCE_STATIC:
            m_mode_str = "TDP_BALANCE_STATIC";
            break;
        case GEOPM_POLICY_MODE_FREQ_UNIFORM_STATIC:
            m_mode_str = "FREQ_UNIFORM_STATIC";
            break;
        case GEOPM_POLICY_MODE_FREQ_HYBRID_STATIC:
            m_mode_str = "FREQ_HYBRID_STATIC";
            break;
        case GEOPM_POLICY_MODE_PERF_BALANCE_DYNAMIC:
            m_mode_str = "PERF_BALANCE_DYNAMIC";
            break;
        case GEOPM_POLICY_MODE_FREQ_UNIFORM_DYNAMIC:
            m_mode_str = "FREQ_UNIFORM_DYNAMIC";
            break;
        case GEOPM_POLICY_MODE_FREQ_HYBRID_DYNAMIC:
            m_mode_str = "FREQ_HYBRID_DYNAMIC";
            break;
        case GEOPM_POLICY_MODE_STATIC:
            m_mode_str = "STATIC";
            break;
        case GEOPM_POLICY_MODE_DYNAMIC:
            m_mode_str = "DYNAMIC";
            break;
        case GEOPM_POLICY_MODE_SHUTDOWN:
            m_mode_str = "SHUTDOWN";
            break;
        default:
            m_mode_str = "INVALID";
    }

    geopm_policy_message_s pol_mess = {m_mode, 0, 1, (double) m_power_budget_watts};
    geopm_policy_message_s shutdown_mess = {GEOPM_POLICY_MODE_SHUTDOWN, 0, 1, (double) m_power_budget_watts};
    m_pol_mess = pol_mess;
    m_shutdown_mess = shutdown_mess;

    m_header = std::string("# \"geopm_version\" : \"tag+branchNNgHASH\",\n# \"profile_name\" : \"controller_mock_test\",\n# \"power_budget\" : 600,\n# \"tree_decider\" : \"power_balancing\",\n# \"leaf_decide");

    EXPECT_CALL(*this, mode()).WillOnce(testing::Return(m_mode));
    EXPECT_CALL(*this, mode_string()).WillOnce(testing::Return(m_mode_str));
    EXPECT_CALL(*this, header()).WillOnce(testing::Return(m_header));
    EXPECT_CALL(*this, policy_message(testing::_))
        .WillOnce(testing::SetArgReferee<0>(m_pol_mess))
        .WillOnce(testing::SetArgReferee<0>(m_pol_mess))
        .WillRepeatedly(testing::SetArgReferee<0>(m_shutdown_mess));
    EXPECT_CALL(*this, tree_decider()).WillRepeatedly(testing::ReturnRef(m_tree_decider));
    EXPECT_CALL(*this, leaf_decider()).WillRepeatedly(testing::ReturnRef(m_leaf_decider));
    EXPECT_CALL(*this, budget_watts()).WillRepeatedly(testing::Return(m_power_budget_watts));
}

MockProfileSampler::MockProfileSampler(int num_cpu)
    : m_table_size(num_cpu * 64)
{
    m_ctl_msg = new MockControlMessage();
    m_tbuf = malloc(m_table_size);
    m_tprof_table = new geopm::ProfileThreadTable(m_table_size, m_tbuf);
    m_cpu_rank.resize(num_cpu);
}

MockProfileSampler::~MockProfileSampler()
{
    delete m_ctl_msg;
    delete m_tprof_table;
    free(m_tbuf);
}

void MockProfileSampler::config(std::string report_name, std::string profile_name) {
    m_report_name = report_name;
    m_profile_name = profile_name;

    std::string region_name = "app_region";
    uint64_t region_id = geopm_crc32_str(0, region_name.c_str());
    m_name_set = {region_name};
    std::fill(m_cpu_rank.begin(), m_cpu_rank.end(), -1);
    for (size_t i = 0; i < m_cpu_rank.size(); i++) {
        m_cpu_rank[i] = i;
    }
    int ranks_per_node = 4;
    geopm_time_s ts_epoch;
    geopm_time(&ts_epoch);
    geopm_time_s ts_entry;
    geopm_time(&ts_entry);
    sleep(1);
    geopm_time_s ts_exit;
    geopm_time(&ts_exit);
    geopm_prof_message_s prof_mess_epoch_entry = {0, GEOPM_REGION_ID_EPOCH, ts_epoch, 0.0};
    geopm_prof_message_s prof_mess_epoch_exit = {0, GEOPM_REGION_ID_EPOCH, ts_epoch, 1.0};
    geopm_prof_message_s prof_mess_entry = {0, region_id, ts_entry, 0.0};
    geopm_prof_message_s prof_mess_exit = {0, region_id, ts_exit, 1.0};
    std::pair<uint64_t, struct geopm_prof_message_s> epoch_entry_sample(GEOPM_REGION_ID_EPOCH, prof_mess_epoch_entry);
    std::pair<uint64_t, struct geopm_prof_message_s> epoch_exit_sample(GEOPM_REGION_ID_EPOCH, prof_mess_epoch_exit);
    std::pair<uint64_t, struct geopm_prof_message_s> entry_sample(region_id, prof_mess_entry);
    std::pair<uint64_t, struct geopm_prof_message_s> exit_sample(region_id, prof_mess_exit);
    std::vector<std::pair<uint64_t, struct geopm_prof_message_s> > samples {epoch_entry_sample, entry_sample, exit_sample, epoch_exit_sample};

    EXPECT_CALL(*this, initialize(testing::_)).WillOnce(testing::SetArgReferee<0>(ranks_per_node));
    EXPECT_CALL(*this, do_report()).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(*this, tprof_table()).WillRepeatedly(testing::Return(m_tprof_table));
    EXPECT_CALL(*this, cpu_rank(testing::_)).WillRepeatedly(testing::SetArgReferee<0>(m_cpu_rank));
    EXPECT_CALL(*this, name_set(testing::_)).WillRepeatedly(testing::SetArgReferee<0>(m_name_set));
    EXPECT_CALL(*this, sample(testing::_, testing::_, testing::_)).WillRepeatedly(testing::DoAll(testing::SetArgReferee<0>(samples), testing::SetArgReferee<1>(samples.size())));
    EXPECT_CALL(*this, report_name(testing::_)).WillRepeatedly(testing::SetArgReferee<0>(m_report_name));
    EXPECT_CALL(*this, profile_name(testing::_)).WillRepeatedly(testing::SetArgReferee<0>(m_profile_name));
}

namespace geopm
{
class ControllerTestHelper : public Controller
{
    public:
        ControllerTestHelper(IGlobalPolicy *global_policy, const IComm *comm, MockProfileSampler *swap);
};

ControllerTestHelper::ControllerTestHelper(IGlobalPolicy *global_policy, const IComm *comm, MockProfileSampler *swap)
    : Controller(global_policy, comm)
{
    delete m_sampler;
    m_sampler = swap;
}

class ControllerTest : public :: testing :: Test
{
    protected:
        void SetUp();
        void TearDown();
};

void ControllerTest::SetUp()
{
}

void ControllerTest::TearDown()
{
}

TEST_F(ControllerTest, dummy)
{
    return;
    MockComm mock_in_comm;
    MockComm mock_ppn1;
    MockComm mock_cart;
    MockComm mock_level;
    MockGlobalPolicy mock_policy;
    EXPECT_CALL(mock_policy,mode()).WillOnce(testing::Return(GEOPM_POLICY_MODE_PERF_BALANCE_DYNAMIC));
    EXPECT_CALL(mock_in_comm, split(testing::Matcher<const std::string&>(testing::_), testing::_)).WillOnce(testing::Return(&mock_ppn1));
    EXPECT_CALL(mock_ppn1, split(testing::_, testing::_, testing::_)).WillOnce(testing::Return(&mock_cart));
    EXPECT_CALL(mock_ppn1, num_rank()).WillRepeatedly(testing::Return(2));// TODO return greater than M_MAX_FAN_OUT to call dim_create
    EXPECT_CALL(mock_ppn1, rank()).WillOnce(testing::Return(0));
    std::vector<int> fan_out = {1, 1};
    //EXPECT_CALL(mock_ppn1, dimension_create(testing::_, testing::_)).WillRepeatedly(testing::SetArgReferee<1>(fan_out));
    EXPECT_CALL(mock_cart, rank()).WillOnce(testing::Return(0));
    std::vector<int> coord = {0, 0};
    EXPECT_CALL(mock_cart, coordinate(testing::_, testing::_)).WillOnce(testing::SetArgReferee<1>(coord));
    EXPECT_CALL(mock_cart, split(testing::Matcher<const std::string&>(testing::_), testing::_)).WillOnce(testing::Return(&mock_level));
    EXPECT_CALL(mock_level, window_create(testing::_, testing::_)).Times(2);
    Controller con(&mock_policy, &mock_in_comm);
}

TEST_F(ControllerTest, stc)
{
    std::string report_name = "controller_mock_test.report";// TODO make empty string to execute branch
    std::string profile_name = "controller_mock_test.profile";
    MockProfileSampler *mock_sampler = new MockProfileSampler(geopm_sched_num_cpu());
    mock_sampler->config(report_name, profile_name);

    MockGlobalPolicy mock_policy;
    std::string tree_decider = "power_balancing";
    std::string leaf_decider = "static_policy";
    mock_policy.config(GEOPM_POLICY_MODE_PERF_BALANCE_DYNAMIC, 600, tree_decider, leaf_decider);

    MockComm mock_in_comm;
    MockComm *mock_ppn1 = new MockComm();
    mock_in_comm.config_in_comm(mock_ppn1);
    mock_ppn1->config_ppn1(mock_policy.get_header(), 1, 0);

    ControllerTestHelper ctl(&mock_policy, &mock_in_comm, mock_sampler);

    EXPECT_EQ(ctl.is_node_root(), true);

    ctl.run();
}
}
