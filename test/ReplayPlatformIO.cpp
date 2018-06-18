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

#include "ReplayPlatformIO.hpp"
#include "Helper.hpp"

using testing::Return;
using testing::Sequence;

static void parse_signal_names(std::string input_str, std::string output_str, ReplayMockPlatformIO &pio);
static void parse_push_signal_line(std::string input_str, std::string output_str, ReplayMockPlatformIO &pio);

ReplayPlatformIO::ReplayPlatformIO(const std::string &record_path)
    : m_func_map({std::make_pair("push_signal", parse_push_signal_line),
                  std::make_pair("signal_names", parse_signal_names)})
    , m_non_ret_funcs({"adjust", "write_control", "push_region_signal_total"})
    , m_pio_mock(geopm::make_unique<ReplayMockPlatformIO>())
    , m_record_file(record_path)
{
    std::string line;
    while (std::getline(m_record_file, line)) {
        /// find start of parameter list
        size_t del_pos = line.find("(");
        std::string func_name = line.substr(0, del_pos);

        line = line.substr(del_pos + 1, std::string::npos);

        /// find the end of the parameter list
        del_pos = line.find(")");
        std::string input_str = line.substr(0, del_pos);

        std::string output_str = "";
        if (m_non_ret_funcs.find(func_name) != m_non_ret_funcs.end()) {
            output_str = line.substr(del_pos + 1, std::string::npos);
        }
        auto parser = m_func_map.find(func_name);
        ///@todo
        //ASSERT_NE(m_func_map.end(), parser);
        //ASSERT_TRUE(false);// << "parser for PlatformIO function: [" << func_name << "] unimplemented";
        // for now, until I get all parsers into map
        if (m_func_map.end() != parser) {
            parser->second(input_str, output_str, *m_pio_mock);
        }
    }
}

void parse_signal_names(std::string input_str, std::string output_str, ReplayMockPlatformIO &pio)
{
    std::set<std::string> ret;
    /// find the first of the signal names set
    size_t del_pos = output_str.find("{");
    while (del_pos != std::string::npos) {
        std::string signal = "";
        size_t start_pos = del_pos + 1;
        del_pos = output_str.find(",");
        /// last signal in set
        if (del_pos == std::string::npos) {
            del_pos = output_str.find("}");
            if (del_pos == std::string::npos) {
                ///ASSERT
            }
            else {
            signal = output_str.substr(start_pos, del_pos);
            }
        }
        else {
            signal = output_str.substr(start_pos, del_pos);
        }
        std::cout << signal << ",";
        ret.insert(signal);
        output_str = output_str.substr(del_pos + 1, std::string::npos);
        del_pos = output_str.find(",");
    }
    std::cout << std::endl;
    Sequence seq = pio.get_sequence();
    EXPECT_CALL(pio, signal_names())
        .InSequence(seq)
        .WillOnce(Return(ret));
}

void parse_push_signal_line(std::string input_str, std::string output_str, ReplayMockPlatformIO &pio)
{
    const int num_input_params = 3;
    std::string signal_name = "";
    int domain_type = 0, domain_idx = 0, ret = 0;
    for (int x = 0; x < num_input_params; ++x) {
        size_t del_pos = input_str.find(",");
        std::string substr = input_str.substr(0, del_pos);
        switch(x) {
            case 0:
                signal_name = substr;
                break;
            case 1:
                domain_type = atoi(substr.c_str());
                break;
            case 2:
                domain_idx = atoi(substr.c_str());
                break;
            default:
                break;
        }
        input_str = input_str.substr(del_pos + 1, std::string::npos);
    }

    Sequence seq = pio.get_sequence();
    EXPECT_CALL(pio, push_signal(signal_name, domain_type, domain_idx))
        .InSequence(seq)
        .WillOnce(Return(ret));
}

geopm::IPlatformIO &ReplayPlatformIO::platform_io()
{
    return *m_pio_mock;
}
