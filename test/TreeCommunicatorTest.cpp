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

#include <iostream> // DEBUG

#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "geopm_env.h"
#include "TreeCommunicator.hpp"
#include "Comm.hpp"
#include "MockComm.hpp"
#include "Controller.hpp"
#include "MockGlobalPolicy.hpp"
#include "geopm_policy.h"
#include "Exception.hpp"

#ifndef NAME_MAX
#define NAME_MAX 256
#endif

#define WORLD_SIZE 16

struct level_context_s
{
    int level;
    bool pol_sender;
    bool samp_snder;
    struct geopm_policy_message_s policy;
    std::vector<struct geopm_sample_message_s> samples;
};

class TreeCommunicatorTest: public :: testing :: Test
{
    public:
        TreeCommunicatorTest();
        ~TreeCommunicatorTest();
    protected:
        void config_ppn1_comm(int size, int rank);
        void config_cart_comm(int size, int rank, int cart_rank);
        void config_level_comms(std::vector<int> sizes, std::vector<int> ranks);
        const std::vector<int> m_coordinates[WORLD_SIZE];
        geopm::TreeCommunicator *m_tcomm;
        MockGlobalPolicy *m_polctl;
        MockComm *m_ppn1_comm;
        MockComm *m_cart_comm;
        std::vector<MockComm *> m_level_comms;
        std::vector<struct level_context_s> m_level_ctxs;
};

void TreeCommunicatorTest::config_ppn1_comm(int size, int rank)
{
    EXPECT_CALL(*m_ppn1_comm, num_rank()).WillRepeatedly(testing::Return(size));
    EXPECT_CALL(*m_ppn1_comm, rank()).WillRepeatedly(testing::Return(rank));
    EXPECT_CALL(*m_ppn1_comm, split(testing::_, testing::_, testing::Matcher<bool> (testing::_)))
        .WillOnce(testing::Return(m_cart_comm));
    EXPECT_CALL(*m_ppn1_comm, barrier()).WillRepeatedly(testing::Return());
}

static const std::vector<int> g_coordinates[WORLD_SIZE] =
{
    {0, 0},
    {0, 1},
    {0, 2},
    {0, 3},
    {0, 4},
    {0, 5},
    {0, 6},
    {0, 7},
    {1, 0},
    {1, 2},
    {1, 3},
    {1, 4},
    {1, 5},
    {1, 6},
    {1, 7},
};

ACTION_P(CalcCoord, coordinates)
{
    testing::SetArgReferee<1>(coordinates[arg0]);
}

void TreeCommunicatorTest::config_cart_comm(int size, int rank, int cart_rank)
{
    EXPECT_CALL(*m_cart_comm, num_rank()).WillRepeatedly(testing::Return(size));
    EXPECT_CALL(*m_cart_comm, rank()).WillRepeatedly(testing::Return(rank));
    EXPECT_CALL(*m_cart_comm, cart_rank(testing::_))
        .WillRepeatedly(testing::Return(cart_rank));
    EXPECT_CALL(*m_cart_comm, coordinate(testing::_, testing::_))
        .WillOnce(CalcCoord(m_coordinates));
    EXPECT_CALL(*m_cart_comm, barrier()).WillRepeatedly(testing::Return());
    testing::Sequence s2;
    for (auto level_comm : m_level_comms) {
        EXPECT_CALL(*m_cart_comm, split(testing::Matcher<int>(testing::_), testing::Matcher<int>(testing::_)))
            .InSequence(s2)
            .WillOnce(testing::Return(level_comm));
    }
}

void TreeCommunicatorTest::config_level_comms(std::vector<int> sizes, std::vector<int> ranks)
{
    ASSERT_EQ(m_level_comms.size(), sizes.size());
    ASSERT_EQ(m_level_comms.size(), ranks.size());
    m_level_ctxs.resize(m_level_comms.size());
    for (size_t level = 0; level < m_level_comms.size(); level++) {
        MockComm *level_comm = m_level_comms[level];
        m_level_ctxs[level].level = level;
        m_level_ctxs[level].samples.resize(sizes[level]);
        struct level_context_s *s_lcs = &m_level_ctxs[level];
        EXPECT_CALL(*level_comm, num_rank())
            .WillRepeatedly(testing::Return(sizes[level]));
        EXPECT_CALL(*level_comm, rank())
            .WillRepeatedly(testing::Return(ranks[level]));
        m_level_ctxs.resize(sizes[level]);
        EXPECT_CALL(*level_comm, alloc_mem(testing::_, testing::_))
            .WillRepeatedly(testing::Invoke([this, level] (size_t size, void **base)
                            {
                            struct level_context_s *lcs = &this->m_level_ctxs[level];
                            *base = static_cast<void *> (lcs->samples.data());
                            }));
        EXPECT_CALL(*level_comm, free_mem(testing::_))
            .WillRepeatedly(testing::Return());
        EXPECT_CALL(*level_comm, barrier()).WillRepeatedly(testing::Return());
        testing::Sequence s1;
        EXPECT_CALL(*level_comm, window_create(testing::_, testing::_))
            .InSequence(s1)
            .WillOnce(testing::Invoke([this, level] (size_t size, void *base)
                            {
                            struct level_context_s *lcs = &this->m_level_ctxs[level];
                            //policy window
                            lcs->pol_sender = size == 0;
                            return 0;
                            }))
            .WillOnce(testing::Invoke([this, level] (size_t size, void *base)
                            {
                            struct level_context_s *lcs = &this->m_level_ctxs[level];
                            //sample window
                            lcs->samp_snder = size == 0;
                            return 1;
                            }));
        EXPECT_CALL(*level_comm, window_destroy(testing::_))
            .WillRepeatedly(testing::Return());
        EXPECT_CALL(*level_comm, window_lock(testing::_, testing::_, testing::_, testing::_))
            .WillRepeatedly(testing::Invoke([this, level] (size_t window_id, bool is_exclusive, int rank, int assert)
                            {
                            struct level_context_s *lcs = &this->m_level_ctxs[level];
                            switch (window_id) {
                                case 0:  //policy window:
                                    if (lcs->pol_sender) {
                                    } else {
                                         this->m_polctl->policy_message(lcs->policy);
                                    }
                                    break;
                                case 1:  //sample window:
                                    if (lcs->samp_snder) {
                                    } else {
                                    std::fill(lcs->samples.begin(), lcs->samples.end(), GEOPM_SAMPLE_INVALID);
                                    }
                                    break;
                            }
                            }));
    }
}

TreeCommunicatorTest::TreeCommunicatorTest()
    : m_coordinates(g_coordinates)
    , m_tcomm(NULL)
    , m_polctl(new MockGlobalPolicy())
    , m_ppn1_comm(NULL)
    , m_cart_comm(NULL)
{
    struct geopm_policy_message_s pm = {GEOPM_POLICY_MODE_PERF_BALANCE_DYNAMIC, 0xDEADBEEF, 5, 75500};
    EXPECT_CALL(*m_polctl, policy_message(testing::_)).WillRepeatedly(testing::SetArgReferee<0>(pm));
}

TreeCommunicatorTest::~TreeCommunicatorTest()
{
    delete m_polctl;
}

TEST_F(TreeCommunicatorTest, hello)
{
    std::vector<int> ranks = {0, 8, 15};
    std::vector<int> cart_ranks = {0, 0, 15};
    std::vector<std::vector<int> > all_level_ranks = {{0, 0}, {0, 1}, {7, 0}};
    std::vector<int> level_sizes = {8, 2};
    std::vector<int> factor = {2, 8};

    ASSERT_EQ(ranks.size(), cart_ranks.size());
    ASSERT_EQ(ranks.size(), all_level_ranks.size());
    for (int i = 0; i < ranks.size(); i++) {
        int rank = ranks[i];
        int cart_rank = cart_ranks[i];
        std::vector<int> level_ranks = all_level_ranks[i];
        m_ppn1_comm = new MockComm();
        m_cart_comm = new MockComm();
        m_level_comms.resize(2);
        m_level_ctxs.resize(2);
        for (size_t level = 0; level < m_level_comms.size(); level++) {
            m_level_comms[level] = new MockComm();
        }

        config_ppn1_comm(WORLD_SIZE, rank);
        config_cart_comm(WORLD_SIZE, rank, cart_rank);
        config_level_comms(level_sizes, level_ranks);

        if (!rank) {
            m_tcomm = new geopm::TreeCommunicator(factor, m_polctl, m_ppn1_comm);
        } else {
            m_tcomm = new geopm::TreeCommunicator(factor, NULL, m_ppn1_comm);
        }

        EXPECT_EQ(1, m_tcomm->num_level() > 0 && m_tcomm->num_level() <= 3) << "for rank: " << rank << ".";
        EXPECT_EQ(2, m_tcomm->root_level()) << "for rank: " << rank << ".";
        EXPECT_EQ(8, m_tcomm->level_size(0)) << "for rank: " << rank << ".";
        EXPECT_EQ(2, m_tcomm->level_size(1)) << "for rank: " << rank << ".";
        EXPECT_EQ(1, m_tcomm->level_size(2)) << "for rank: " << rank << ".";

        delete m_tcomm;
        delete m_ppn1_comm;
    }
}

TEST_F(TreeCommunicatorTest, send_policy_down_rank_0)
{
    int success;
    struct geopm_policy_message_s policy = {0};
    std::vector <struct geopm_policy_message_s> send_policy;
    size_t level;
    int rank = 0;
    int cart_rank = 0;
    std::vector<int> level_ranks = {0, 0};
    std::vector<int> level_sizes = {8, 2};
    std::vector<int> factor(2);
    std::string td = "power_balancing";
    std::string ld = "power_governing";
    factor[0] = 2;
    factor[1] = 8;

    m_ppn1_comm = new MockComm();
    m_cart_comm = new MockComm();
    m_level_comms.resize(2);
    m_level_ctxs.resize(2);
    for (level = 0; level < m_level_comms.size(); level++) {
        m_level_comms[level] = new MockComm();
    }

    config_ppn1_comm(WORLD_SIZE, rank);
    config_cart_comm(WORLD_SIZE, rank, cart_rank);
    config_level_comms(level_sizes, level_ranks);

    struct geopm_policy_message_s pm = {GEOPM_POLICY_MODE_PERF_BALANCE_DYNAMIC, 0xDEADBEEF, 5, 75500};
    //EXPECT_CALL(*m_polctl, mode()).WillRepeatedly(testing::Return(GEOPM_POLICY_MODE_PERF_BALANCE_DYNAMIC));// not called...
    //EXPECT_CALL(*m_polctl, frequency_mhz()).WillRepeatedly(testing::Return(1200));
    //EXPECT_CALL(*m_polctl, tree_decider()).WillRepeatedly(testing::ReturnRef(td));
    //EXPECT_CALL(*m_polctl, leaf_decider()).WillRepeatedly(testing::ReturnRef(ld));
    //EXPECT_CALL(*m_polctl, budget_watts()).WillRepeatedly(testing::Return(75500));
    EXPECT_CALL(*m_polctl, policy_message(testing::_)).WillRepeatedly(testing::SetArgReferee<0>(pm));

    m_tcomm = new geopm::TreeCommunicator(factor, m_polctl, m_ppn1_comm);

    for (level = m_tcomm->num_level() - 1; level > 0; --level) {
        if (level == m_tcomm->root_level()) {
            m_tcomm->get_policy(level, policy);
            policy.flags = m_tcomm->root_level();
        }
        else {
            success = 0;
            while (!success) {
                try {
                    m_tcomm->get_policy(level, policy);
                    EXPECT_EQ(m_tcomm->root_level(), (int)policy.flags);
                    success = 1;
                }
                catch (geopm::Exception ex) {
                    if (ex.err_value() != GEOPM_ERROR_POLICY_UNKNOWN) {
                        throw ex;
                    }
                }
            }
        }
        if (level) {
            send_policy.resize(m_tcomm->level_size(level - 1));
            fill(send_policy.begin(), send_policy.end(), policy);
            m_tcomm->send_policy(level - 1, send_policy);
        }
    }

    delete m_tcomm;
    delete m_ppn1_comm;
}

TEST_F(TreeCommunicatorTest, send_policy_down_rank_8)
{
    int success;
    struct geopm_policy_message_s policy = {0};
    std::vector <struct geopm_policy_message_s> send_policy;
    size_t level;
    int rank = 8;
    int cart_rank = 0;
    std::vector<int> level_ranks = {0, 1};
    std::vector<int> level_sizes = {8, 2};
    std::vector<int> factor(2);
    factor[0] = 2;
    factor[1] = 8;

    m_ppn1_comm = new MockComm();
    m_cart_comm = new MockComm();
    m_level_comms.resize(2);
    for (level = 0; level < m_level_comms.size(); level++) {
        m_level_comms[level] = new MockComm();
    }

    config_ppn1_comm(WORLD_SIZE, rank);
    config_cart_comm(WORLD_SIZE, rank, cart_rank);
    config_level_comms(level_sizes, level_ranks);

    m_tcomm = new geopm::TreeCommunicator(factor, NULL, m_ppn1_comm);

    for (level = m_tcomm->num_level() - 1; level > 0; --level) {
        if (level == m_tcomm->root_level()) {
            m_tcomm->get_policy(level, policy);
            policy.flags = m_tcomm->root_level();
        }
        else {
            success = 0;
            while (!success) {
                try {
                    m_tcomm->get_policy(level, policy);
                    EXPECT_EQ(m_tcomm->root_level(), (int)policy.flags);
                    success = 1;
                }
                catch (geopm::Exception ex) {
                    if (ex.err_value() != GEOPM_ERROR_POLICY_UNKNOWN) {
                        throw ex;
                    }
                }
            }
        }
        if (level) {
            send_policy.resize(m_tcomm->level_size(level - 1));
            fill(send_policy.begin(), send_policy.end(), policy);
            m_tcomm->send_policy(level - 1, send_policy);
        }
    }

    delete m_tcomm;
    delete m_ppn1_comm;
}

TEST_F(TreeCommunicatorTest, send_sample_up_rank_8)
{
    int success;
    std::vector <struct geopm_sample_message_s> samples;
    struct geopm_sample_message_s send_sample = {0};
    struct geopm_policy_message_s policy = {0};
    std::vector <struct geopm_policy_message_s> send_policy;
    size_t level;
    int rank = 8;
    int cart_rank = 0;
    std::vector<int> level_ranks = {0, 1};
    std::vector<int> level_sizes = {8, 2};
    std::vector<int> factor(2);
    factor[0] = 2;
    factor[1] = 8;

    m_ppn1_comm = new MockComm();
    m_cart_comm = new MockComm();
    m_level_comms.resize(2);
    for (size_t level = 0; level < m_level_comms.size(); level++) {
        m_level_comms[level] = new MockComm();
    }

    config_ppn1_comm(WORLD_SIZE, rank);
    config_cart_comm(WORLD_SIZE, rank, cart_rank);
    config_level_comms(level_sizes, level_ranks);

    m_tcomm = new geopm::TreeCommunicator(factor, NULL, m_ppn1_comm);

    int num_level = m_tcomm->num_level();
    if (m_tcomm->root_level() == num_level - 1) {
        num_level--;
    }
    for (size_t level = 0; level < num_level; ++level) {
        send_sample.region_id = 1;
        send_sample.signal[0] = m_tcomm->level_rank(level) * (level + 1);
        m_tcomm->send_sample(level, send_sample);
        if (level && m_tcomm->level_rank(level) == 0) {
            samples.resize(m_tcomm->level_size(level - 1));
            success = 0;
            while (!success) {
                try {
                    m_tcomm->get_sample(level, samples);
                    for (size_t rank = 0; rank < m_tcomm->level_size(level); ++rank) {
                        EXPECT_EQ((uint64_t)rank * level, samples[rank].signal[0]);
                    }
                    success = 1;
                }
                catch (geopm::Exception ex) {
                    if (ex.err_value() != GEOPM_ERROR_SAMPLE_INCOMPLETE) {
                        throw ex;
                    }
                }
            }
        }
    }

    delete m_tcomm;
    delete m_ppn1_comm;
}
