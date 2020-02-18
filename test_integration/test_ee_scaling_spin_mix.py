#!/usr/bin/env python
#
#  Copyright (c) 2015, 2016, 2017, 2018, 2019, 2020, Intel Corporation
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#      * Redistributions of source code must retain the above copyright
#        notice, this list of conditions and the following disclaimer.
#
#      * Redistributions in binary form must reproduce the above copyright
#        notice, this list of conditions and the following disclaimer in
#        the documentation and/or other materials provided with the
#        distribution.
#
#      * Neither the name of Intel Corporation nor the names of its
#        contributors may be used to endorse or promote products derived
#        from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY LOG OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

"""Test the energy efficient agent by doing a frequency scaling/net_spin mix.

"""

from __future__ import absolute_import

import sys
import re
import unittest
import os
import glob

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from test_integration import geopm_context
import geopmpy.io
from test_integration import geopm_test_launcher
from test_integration import util

_g_skip_launch = False


class AppConf(object):
    """Class that is used by the test launcher as a geopmpy.io.BenchConf
    when running the script as a benchmark.

    """
    def write(self):
        """No configuration files are required.

        """
        pass

    def get_exec_path(self):
        """Path to bencmark

        """
        script_dir = os.path.dirname(os.path.realpath(__file__))
        return os.path.join(script_dir, '.libs', 'test_ee_scaling_spin_mix')

    def get_exec_args(self):
        return []


@util.skip_unless_cpufreq()
@util.skip_unless_optimized()
@util.skip_unless_run_long_tests()
class TestIntegrationEEScalingSpinMix(unittest.TestCase):
    @classmethod
    def get_run_subtest_name(cls, test_min_freq, test_max_freq):
        return '{}_{}_{}'.format(cls._test_name, test_min_freq, test_max_freq)
    @classmethod
    def get_run_report(cls, test_min_freq, test_max_freq):
        cls._agent = 'energy_efficient'
        cls._options = {'FREQ_MIN': test_min_freq,
                         'FREQ_MAX': test_max_freq}
        subtest_name = cls.get_run_subtest_name(test_min_freq, test_max_freq)
        report_path = subtest_name + '.report'
        agent_conf = geopmpy.io.AgentConf(subtest_name + '_agent.config', cls._agent, cls._options)
        cls._tmp_files = []
        cls._tmp_files.append(report_path)
        cls._tmp_files.append(agent_conf.get_path())
        launcher = geopm_test_launcher.TestLauncher(AppConf(), agent_conf, report_path)
        launcher.set_num_node(1)
        launcher.set_num_rank(1)
        launcher.run(subtest_name)
        return geopmpy.io.RawReport(report_path)

    @classmethod
    # todo decorate with batch, do not use test launcher for geopmread
    # todo use ptopo python wrapper for geopmread -d
    # todo explore ptopo based dynamic num_cpu detection
    # todo need to make sure both trials land on the same host
    def setUpClass(cls):
        """
        Test that the energy_efficient agent can save energy on short-running regions.
        """
        sys.stdout.write('(' + os.path.basename(__file__).split('.')[0] +
                         '.' + cls.__name__ + ') ...')
        cls._test_name = 'test_ee_scaling_spin_mix'
        cls._report_path = cls._test_name + '.report'
        cls._trace_path = cls._test_name + '.trace'
        cls._skip_launch = _g_skip_launch
        cls._keep_files = os.getenv('GEOPM_KEEP_FILES') is not None
        cls._agent_conf_path = cls._test_name + '-agent-config.json'
        min_freq = geopm_test_launcher.geopmread("CPUINFO::FREQ_MIN board 0")
        sticker_freq = geopm_test_launcher.geopmread("CPUINFO::FREQ_STICKER board 0")
        if not cls._skip_launch:
            cls._ee_report = cls.get_run_report(min_freq, sticker_freq)
            cls._baseline_report = cls.get_run_report(sticker_freq, sticker_freq)
        else:
            subtest_name = cls.get_run_subtest_name(min_freq, sticker_freq)
            report_path = subtest_name + '.report'
            cls._ee_report = geopmpy.io.RawReport(report_path)
            subtest_name = cls.get_run_subtest_name(sticker_freq, sticker_freq)
            report_path = subtest_name + '.report'
            cls._baseline_report = geopmpy.io.RawReport(report_path)

    @classmethod
    def tearDownClass(cls):
        """If we are not handling an exception and the GEOPM_KEEP_FILES
        environment variable is unset, clean up output.

        """
        if (sys.exc_info() == (None, None, None) and not
            cls._keep_files and not cls._skip_launch):
            os.unlink(cls._agent_conf_path)
            os.unlink(cls._report_path)
            for ff in glob.glob(cls._trace_path + '*'):
                os.unlink(ff)

    def test_monotone_frequency(self):
        """Test that agent selects lower frequencies for regions with more
        stream.

        """
        for host_name in self._ee_report.host_names():
            ee_app_totals = self._ee_report.raw_totals(host_name)
            baseline_app_totals = self._baseline_report.raw_totals(host_name)
            msg = 'Save energy on the total application'
            self.assertLess(ee_app_totals['package-energy (joules)'], baseline_app_totals['package-energy (joules)'], msg=msg)
            for region_name in self._ee_report.region_names(host_name):
                if region_name == 'scaling':
                    ee_scaling_region = self._ee_report.raw_region(host_name, region_name)
                    baseline_scaling_region = self._baseline_report.raw_region(host_name, region_name)
                    self.assertTrue('requested-online-frequency' in ee_scaling_region,
                                    msg='Learning for region {} did not complete'.format(region_name))
                    # todo assert ee runtime not more than 10% longer than baseline
                    # The small scaling region should be compute-bound. Expect that it
                    # is greater than min and less than max?.
                if region_name == 'network_spin':
                    ee_spin_region = self._ee_report.raw_region(host_name, region_name)
                    baseline_spin_region = self._baseline_report.raw_region(host_name, region_name)
                    self.assertFalse('requested-online-frequency' in ee_spin_region,
                                     msg='Learning for region {} did not complete'.format(region_name))
                    msg = 'Save energy on the network-hinted spin region'
                    self.assertLess(ee_spin_region['package-energy (joules)'],
                                    baseline_spin_region['package-energy (joules)'], msg=msg)


if __name__ == '__main__':
    try:
        sys.argv.remove('--skip-launch')
        _g_skip_launch = True
    except ValueError:
        pass
    unittest.main()
