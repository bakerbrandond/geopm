/*
 * Copyright (c) 2015, 2016, 2017, 2018, 2019, Intel Corporation
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

#include "StubIOGroup.hpp"

#include <cpuid.h>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <utility>
#include <iomanip>
#include <iostream>

#include "geopm/IOGroup.hpp"
#include "geopm_topo.h"
#include "geopm/Exception.hpp"
#include "geopm/Agg.hpp"
#include "geopm/Helper.hpp"

#define STUBIO_GROUP_PLUGIN_NAME "STUBIO"

// Registers this IOGroup with the IOGroup factory, making it visible
// to PlatformIO when the plugin is first loaded.
static void __attribute__((constructor)) example_iogroup_load(void)
{
    geopm::iogroup_factory().register_plugin(StubIOGroup::plugin_name(),
                                             StubIOGroup::make_plugin);
}

StubIOGroup::StubIOGroup()
    : m_signal_names({"TIMESTAMP_COUNTER",
                      "FREQUENCY",
                      "FREQUENCY_MAX",
                      "ENERGY_PACKAGE",
                      "ENERGY_DRAM",
                      "INSTRUCTIONS_RETIRED",
                      "CYCLES_THREAD",
                      "CYCLES_REFERENCE",
                      "POWER_PACKAGE_MIN",
                      "POWER_PACKAGE_MAX",
                      "POWER_PACKAGE_TDP",
                      "TEMPERATURE_CORE_UNDER",
                      "TEMPERATURE_PKG_UNDER",
                      "TEMPERATURE_MAX"})
    , m_control_names({"POWER_PACKAGE_LIMIT",
                       "FREQUENCY",
                       "POWER_PACKAGE_TIME_WINDOW"})
{
}

StubIOGroup::~StubIOGroup()
{

}

std::set<std::string> StubIOGroup::signal_names(void) const
{
    return m_signal_names;
}

std::set<std::string> StubIOGroup::control_names(void) const
{
    return m_control_names;
}

bool StubIOGroup::is_valid_signal(const std::string &signal_name) const
{
    return m_signal_names.find(signal_name) != m_signal_names.end();
}

bool StubIOGroup::is_valid_control(const std::string &control_name) const
{
    return m_control_names.find(control_name) != m_control_names.end();
}

int StubIOGroup::signal_domain_type(const std::string &signal_name) const
{
    int result = GEOPM_DOMAIN_INVALID;
    if (is_valid_signal(signal_name)) {
        result = GEOPM_DOMAIN_BOARD;
    }
    return result;
}

int StubIOGroup::control_domain_type(const std::string &control_name) const
{
    int result = GEOPM_DOMAIN_INVALID;
    if (is_valid_control(control_name)) {
        result = GEOPM_DOMAIN_BOARD;
    }
    return result;
}

int StubIOGroup::push_signal(const std::string &signal_name, int domain_type, int domain_idx)
{
    //if (m_is_active) {
        //throw geopm::Exception("StubIOGroup::push_signal(): cannot push a signal after read_batch() or adjust() has been called.",
                        //GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    //}
    // these if are check_signal
    if (!is_valid_signal(signal_name)) {
        throw geopm::Exception("StubIOGroup::push_signal(): signal_name " + signal_name +
                        " not valid for StubIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    if (signal_domain_type(signal_name) != domain_type) {
        throw geopm::Exception("StubIOGroup::push_signal(): signal_name " + signal_name +
                        " domain " + std::to_string(domain_type) + " not valid for StubIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    if (domain_idx != 0) {
        throw geopm::Exception("StubIOGroup::push_signal(): domain_idx out of range",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    return 1;
}

int StubIOGroup::push_control(const std::string &control_name, int domain_type, int domain_idx)
{
    //if (m_is_active) {
        //throw geopm::Exception("StubIOGroup::push_control(): cannot push a control after read_batch() or adjust() has been called.",
                        //GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    //}
    // these if are check_control
    if (!is_valid_control(control_name)) {
        throw geopm::Exception("StubIOGroup::push_control(): control_name " + control_name +
                        " not valid for StubIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    if (control_domain_type(control_name) != domain_type) {
        throw geopm::Exception("StubIOGroup::push_control(): control_name " + control_name +
                        " domain " + std::to_string(domain_type) + " not valid for StubIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    if (domain_idx != 0) {
        throw geopm::Exception("StubIOGroup::push_signal(): domain_idx out of range",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    return 1;
}

void StubIOGroup::read_batch(void)
{
    if (!m_is_active) {
        activate();
    }
}

void StubIOGroup::write_batch(void)
{
}

double StubIOGroup::sample(int signal_idx)
{
    if (signal_idx != 1) {
        throw geopm::Exception("StubIOGroup::sample(): signal_idx out of range",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }

    return 0.0;
}

void StubIOGroup::adjust(int control_idx, double setting)
{
    if (control_idx != 1) {
        throw geopm::Exception("StubIOGroup::adjust(): control_idx out of range",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    if (!m_is_active) {
        activate();
    }
}

double StubIOGroup::read_signal(const std::string &signal_name, int domain_type, int domain_idx)
{
    // implement and call check_signal
    return 0.0;
}

void StubIOGroup::write_control(const std::string &control_name, int domain_type, int domain_idx, double setting)
{
    // implement and call check_control
}

void StubIOGroup::save_control(void)
{
}

void StubIOGroup::restore_control(void)
{
}

void StubIOGroup::activate(void)
{
    m_is_active = true;
}

std::string StubIOGroup::plugin_name(void)
{
    return STUBIO_GROUP_PLUGIN_NAME;
}

std::unique_ptr<geopm::IOGroup> StubIOGroup::make_plugin(void)
{
    return geopm::make_unique<StubIOGroup>();
}

double sum(const std::vector<double> &operand)
{
    double result = NAN;
    if (operand.size()) {
        result = std::accumulate(operand.begin(), operand.end(), 0.0);
    }
    return result;
}

std::function<double(const std::vector<double> &)> StubIOGroup::agg_function(const std::string &signal_name) const
{
    if (!is_valid_signal(signal_name)) {
        throw geopm::Exception("StubIOGroup::agg_function(): signal_name " + signal_name +
                        " not valid for StubIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    //return geopm::Agg::sum;
    return sum;
}

#include <climits>
std::string string_format_double(double signal)
{
    char result[NAME_MAX];
    snprintf(result, NAME_MAX, "%.16g", signal);
    return result;
}

std::function<std::string(double)> StubIOGroup::format_function(const std::string &signal_name) const
{
    if (!is_valid_signal(signal_name)) {
        throw geopm::Exception("StubIOGroup::format_function(): signal_name " + signal_name +
                        " not valid for StubIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    //return geopm::string_format_double;
    return string_format_double;
}

std::string StubIOGroup::signal_description(const std::string &signal_name) const
{
    if (!is_valid_signal(signal_name)) {
        throw geopm::Exception("StubIOGroup::signal_description(): signal_name " + signal_name +
                        " not valid for StubIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    return "Stub for signal " + signal_name + " not a real system signal, constant value of 0.0";
}

std::string StubIOGroup::control_description(const std::string &control_name) const
{
    if (!is_valid_control(control_name)) {
        throw geopm::Exception("StubIOGroup::control_description(): control_name " + control_name +
                        " not valid for StubIOGroup",
                        GEOPM_ERROR_INVALID, __FILE__, __LINE__);
    }
    return "Stub for control " + control_name + " does not have any system effect";
}
