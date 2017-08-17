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

ACTION_P(SetArg0ToVoidP, value)
{
    arg0 = static_cast<void  *>(value);
}

TEST_F(ControllerTest, stc)
{
    MockComm mock_in_comm;
    MockComm *mock_ppn1 = new MockComm();
    MockGlobalPolicy mock_policy;
    MockProfileSampler *mock_sampler = new MockProfileSampler();
    MockControlMessage *mock_ctl_msg = new MockControlMessage();
    mock_sampler->m_ctl_msg = mock_ctl_msg;
    std::string tree_decider = "power_balancing";
    std::string leaf_decider = "static_policy";
    std::string report_name = "controller_mock_test.report";// TODO make empty string to execute branch
    std::string profile_name = "controller_mock_test.profile";
    int budget = 150;
    char header[] = "# \"geopm_version\" : \"tag+branchNNgHASH\",\n# \"profile_name\" : \"controller_mock_test\",\n# \"power_budget\" : 150,\n# \"tree_decider\" : \"power_balancing\",\n# \"leaf_decide";
    int len = sizeof(header);
    std::string region_name = "app_region";
    uint64_t region_id = geopm_crc32_str(0, region_name.c_str());
    std::set<std::string> region_name_set = {region_name};
    geopm_time_s ts_epoch;
    geopm_time(&ts_epoch);
    geopm_time_s ts_entry;
    geopm_time(&ts_entry);
    sleep(1);
    geopm_time_s ts_exit;
    geopm_time(&ts_exit);
    geopm_policy_message_s pol_mess = {GEOPM_POLICY_MODE_PERF_BALANCE_DYNAMIC, 0, 1, 600};
    geopm_policy_message_s shutdown_mess = {GEOPM_POLICY_MODE_SHUTDOWN, 0, 1, 600};
    geopm_prof_message_s prof_mess_epoch_entry = { 0, GEOPM_REGION_ID_EPOCH, ts_epoch, 0.0};
    geopm_prof_message_s prof_mess_epoch_exit = { 0, GEOPM_REGION_ID_EPOCH, ts_epoch, 1.0};
    geopm_prof_message_s prof_mess_entry = { 0, region_id, ts_entry, 0.0};
    geopm_prof_message_s prof_mess_exit = { 0, region_id, ts_exit, 1.0};
    std::pair<uint64_t, struct geopm_prof_message_s> epoch_entry_sample(GEOPM_REGION_ID_EPOCH, prof_mess_epoch_entry);
    std::pair<uint64_t, struct geopm_prof_message_s> epoch_exit_sample(GEOPM_REGION_ID_EPOCH, prof_mess_epoch_exit);
    std::pair<uint64_t, struct geopm_prof_message_s> entry_sample(region_id, prof_mess_entry);
    std::pair<uint64_t, struct geopm_prof_message_s> exit_sample(region_id, prof_mess_exit);
    std::vector<std::pair<uint64_t, struct geopm_prof_message_s> > samples {epoch_entry_sample, entry_sample, exit_sample, epoch_exit_sample};
    int ranks_per_node = 4;
    geopm_policy_message_s mess = { GEOPM_POLICY_MODE_PERF_BALANCE_DYNAMIC, 0, 1, (double) budget};
    int num_cpu = geopm_sched_num_cpu();
    std::vector<int> cpu_rank(num_cpu);
    std::fill(cpu_rank.begin(), cpu_rank.end(), -1);
    size_t tbuf_size = num_cpu * 64;
    void *tbuf = malloc(tbuf_size);
    ProfileThreadTable *tprof_table = new ProfileThreadTable(tbuf_size, tbuf);
    cpu_rank[224] = 0;
    cpu_rank[225] = 1;
    cpu_rank[226] = 2;
    cpu_rank[227] = 3;
    EXPECT_CALL(*mock_sampler, initialize(testing::_)).WillOnce(testing::SetArgReferee<0>(ranks_per_node));
    EXPECT_CALL(*mock_sampler, do_report()).WillRepeatedly(testing::Return(true));
    EXPECT_CALL(*mock_sampler, tprof_table()).WillRepeatedly(testing::Return(tprof_table));
    EXPECT_CALL(*mock_sampler, cpu_rank(testing::_)).WillRepeatedly(testing::SetArgReferee<0>(cpu_rank));
    EXPECT_CALL(*mock_sampler, name_set(testing::_)).WillRepeatedly(testing::SetArgReferee<0>(region_name_set));
    EXPECT_CALL(*mock_sampler, sample(testing::_, testing::_, testing::_)).WillRepeatedly(testing::DoAll(testing::SetArgReferee<0>(samples), testing::SetArgReferee<1>(samples.size())));
    EXPECT_CALL(*mock_sampler, report_name(testing::_)).WillRepeatedly(testing::SetArgReferee<0>(report_name));
    EXPECT_CALL(*mock_sampler, profile_name(testing::_)).WillRepeatedly(testing::SetArgReferee<0>(profile_name));
    EXPECT_CALL(mock_policy, mode()).WillOnce(testing::Return(mess.mode));
    EXPECT_CALL(mock_policy, header()).WillOnce(testing::Return(header));
    EXPECT_CALL(mock_policy, policy_message(testing::_))
        .WillOnce(testing::SetArgReferee<0>(pol_mess))
        .WillOnce(testing::SetArgReferee<0>(pol_mess))
        .WillRepeatedly(testing::SetArgReferee<0>(shutdown_mess));
    EXPECT_CALL(mock_policy, tree_decider()).WillRepeatedly(testing::ReturnRef(tree_decider));
    EXPECT_CALL(mock_policy, leaf_decider()).WillRepeatedly(testing::ReturnRef(leaf_decider));
    EXPECT_CALL(mock_policy, budget_watts()).WillRepeatedly(testing::Return(budget));
    //EXPECT_CALL(mock_policy, policy_message(testing::_)).WillRepeatedly(testing::SetArgReferee<0>(pol_mess));
    EXPECT_CALL(mock_in_comm, split(testing::Matcher<const std::string&>(testing::_), testing::_))
        .WillOnce(testing::Return(mock_ppn1));
    EXPECT_CALL(*mock_ppn1, num_rank()).WillRepeatedly(testing::Return(1));
    EXPECT_CALL(*mock_ppn1, rank()).WillRepeatedly(testing::Return(0));
    EXPECT_CALL(*mock_ppn1, barrier()).WillRepeatedly(testing::Return());
    EXPECT_CALL(*mock_ppn1, broadcast(testing::_, testing::_, testing::_))
        .WillOnce(testing::Return())
        .WillOnce(SetArg0ToVoidP(&len))
        .WillOnce(SetArg0ToVoidP(header));
    ControllerTestHelper ctl(&mock_policy, &mock_in_comm, mock_sampler);
    int err = 0;
    ctl.run();
    EXPECT_EQ(err, 0);
    free(tbuf);
}
}
