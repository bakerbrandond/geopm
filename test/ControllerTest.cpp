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
#include "Controller.hpp"
#include "MockComm.hpp"
#include "MockGlobalPolicy.hpp"
#include "MockProfileSampler.hpp"

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
    MockProfileSampler *mock_samp = new MockProfileSampler();
    char header[] = "mock policy header";
    int len = sizeof(header);
    geopm_policy_message_s mess = { GEOPM_POLICY_MODE_PERF_BALANCE_DYNAMIC, 0xDEADBEEF, 1, 140.0};// compiler warn-err
    EXPECT_CALL(mock_policy, mode()).WillOnce(testing::Return(mess.mode));
    EXPECT_CALL(mock_policy, header()).WillOnce(testing::Return(header));
    EXPECT_CALL(mock_policy, policy_message(testing::_)).WillRepeatedly(testing::SetArgReferee<0>(mess));
    EXPECT_CALL(mock_in_comm, split(testing::Matcher<const std::string&>(testing::_), testing::_))
        .WillOnce(testing::Return(mock_ppn1));
    EXPECT_CALL(*mock_ppn1, num_rank()).WillRepeatedly(testing::Return(1));
    EXPECT_CALL(*mock_ppn1, rank()).Times(2).WillRepeatedly(testing::Return(0));
    EXPECT_CALL(*mock_ppn1, barrier()).WillRepeatedly(testing::Return());
    EXPECT_CALL(*mock_ppn1, broadcast(testing::_, testing::_, testing::_))
        .WillOnce(testing::Return())
        .WillOnce(SetArg0ToVoidP(&len))
        .WillOnce(SetArg0ToVoidP(header));
    ControllerTestHelper ctl(&mock_policy, &mock_in_comm, mock_samp);
    int err = 0;
    ctl.run();
    EXPECT_EQ(err, 0);
}
}
