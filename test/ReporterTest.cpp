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

#include <sstream>
#include <fstream>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Reporter.hpp"

using geopm::Reporter;
using testing::HasSubstr;

class ReporterTest : public testing::Test
{
    protected:
        ReporterTest();
        void TearDown(void);
        std::string m_report_name = "test_reporter.out";

        Reporter m_reporter;
};

ReporterTest::ReporterTest()
    : m_reporter(m_report_name, 0)
{

}

void ReporterTest::TearDown(void)
{
    //std::remove(m_report_name.c_str());
}

void check_report(const std::string &expected, const std::string &result)
{
    auto exp_it = 0;
    auto res_it = 0;
    auto exp_end = expected.find("\n", exp_it);
    auto res_end = result.find("\n", res_it);
    int line = 1;
    while (exp_end != std::string::npos && res_end != std::string::npos) {
        std::string exp = expected.substr(exp_it, exp_end - exp_it);
        std::string res = result.substr(res_it, res_end - res_it);
        ASSERT_EQ(exp, res) << " on line " << line;
        exp_it = exp_end + 1;
        res_it = res_end + 1;
        exp_end = expected.find("\n", exp_it);
        res_end = result.find("\n", res_it);
        ++line;
    }

    if (exp_end != res_end) {
        FAIL() << "different length strings";
    }
}

void check_report2(std::istream &expected, std::istream &result)
{
    char exp_line[1024];
    char res_line[1024];
    expected.getline(exp_line, 1024);
    result.getline(res_line, 1024);
    int line = 1;
    while (expected.good() && result.good()) {
        ASSERT_THAT(std::string(res_line), HasSubstr(exp_line)) << " on line " << line;
        expected.getline(exp_line, 1024);
        result.getline(res_line, 1024);
        ++line;
    }
    if (expected.good() != result.good()) {
        std::ostringstream message;
        message << "Different length strings." << std::endl;
        message << "Remaining expected:" << std::endl;
        message << "--------" << std::endl;
        while (expected.good()) {
            expected.getline(exp_line, 1024);
            message << exp_line << std::endl;
        }
        message << "--------" << std::endl;
        message << "Remaining result:" << std::endl;
        message << "--------" << std::endl;
        while (result.good()) {
            result.getline(res_line, 1024);
            message << res_line << std::endl;
        }
        message << "--------" << std::endl;
        FAIL() << message.str();
    }
}

TEST_F(ReporterTest, generate)
{
    // Check for labels at start of line but ignore numbers
    // Note that region lines start with tab
    std::string expected = R"raw(#####
Profile: my_prof
Agent: my_agent
Policy Mode:
Tree Decider:
Leaf Decider:
Power Budget:

Host:
Region all2all (
	runtime (sec):
	energy (joules):
	frequency (%):
	mpi-runtime (sec):
	count:
Region model-init (
	runtime (sec):
	energy (joules):
	frequency (%):
	mpi-runtime (sec):
	count:
Application Totals:
	runtime (sec):
	energy (joules):
	mpi-runtime (sec):
	ignore-time (sec):
	throttle time (%):
	geopmctl memory HWM:
	geopmctl network BW (B/sec):

)raw";
    std::istringstream exp_stream(expected);

    m_reporter.generate(m_report_name, "my_profile", "my_agent", "agent_header",
                        "agent_node", {}, {"all2all", "model-init"}, nullptr);
    std::ifstream report(m_report_name);
    check_report2(exp_stream, report);
}

/*
std::string example_report = R"raw(##### geopm 0.4.0+dev103g7639afc #####
Profile: test_plugin_efficient_freq_offline_offline
Policy Mode: DYNAMIC
Tree Decider: static_policy
Leaf Decider: efficient_freq
Power Budget: 400

Host: mr-fusion2
Region all2all (35397599679):
        runtime (sec): 44.891
        energy (joules): 4767.17
        frequency (%): 100
        mpi-runtime (sec): 54.9077
        count: 60
Region model-init (5977905031):
        runtime (sec): 4.90595
        energy (joules): 692.315
        frequency (%): 77.23
        mpi-runtime (sec): 0
        count: 1
Region dgemm (11396693813):
        runtime (sec): 264.722
        energy (joules): 43340.5
        frequency (%): 84.813
        mpi-runtime (sec): 1.17775
        count: 60
Region stream (20779751936):
        runtime (sec): 48.7621
        energy (joules): 8748.3
        frequency (%): 99.8746
        mpi-runtime (sec): 0
        count: 60
Region unmarked-region (2305843009213693952):
        runtime (sec): 54.9608
        energy (joules): 9277.61
        frequency (%): 97.3734
        mpi-runtime (sec): 0
        count: 0
Region epoch (9223372036854775808):
        runtime (sec): 413.234
        energy (joules): 66120.7
        frequency (%): 89.3575
        mpi-runtime (sec): 56.0855
        count: 60
Application Totals:
        runtime (sec): 418.321
        energy (joules): 66833.2
        mpi-runtime (sec): 56.0855
        ignore-time (sec): 0
        throttle time (%): 0
        geopmctl memory HWM: 57136 kB
        geopmctl network BW (B/sec): 0
)raw";

*/
