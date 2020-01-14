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

"""Test the energy efficient agent against short running regions.

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
        return os.path.join(script_dir, '.libs', 'test_ee_short_regions')

    def get_exec_args(self):
        return []


@util.skip_unless_cpufreq()
@util.skip_unless_optimized()
@util.skip_unless_run_long_tests()
class TestIntegrationEEShortRegions(unittest.TestCase):
    @classmethod
    def get_run_subtest_name(cls, rank_per_node, test_min_freq, test_max_freq):
        return '{}_{}_{}_{}'.format(cls._test_name, rank_per_node, test_min_freq, test_max_freq)

    @classmethod
    def get_run_report(cls, rank_per_node, test_min_freq, test_max_freq):
        cls._agent = 'energy_efficient'
        cls._options = {'FREQ_MIN': test_min_freq,
                        'FREQ_MAX': test_max_freq}
        subtest_name = cls.get_run_subtest_name(rank_per_node, test_min_freq, test_max_freq)
        report_path = subtest_name + '.report'
        agent_conf = geopmpy.io.AgentConf(subtest_name + '_agent.config', cls._agent, cls._options)
        if not cls._skip_launch:
            launcher = geopm_test_launcher.TestLauncher(AppConf(), agent_conf, report_path)
            launcher.set_num_node(rank_per_node)
            launcher.set_num_rank(rank_per_node)
            launcher.run(subtest_name)
        return geopmpy.io.RawReport(report_path)

    @classmethod
    # todo decorate with batch, do not use test launcher for geopmread
    # todo need to make sure both trials land on the same host
    def setUpClass(cls):
        """
        Test that the energy_efficient agent can save energy on short-running regions.
        """
        sys.stdout.write('(' + os.path.basename(__file__).split('.')[0] +
                         '.' + cls.__name__ + ') ...')
        cls._test_name = 'test_ee_scaling_spin_mix'
        cls._skip_launch = _g_skip_launch
        cls._keep_files = os.getenv('GEOPM_KEEP_FILES') is not None
        cls._agent_conf_path = cls._test_name + '-agent-config.json'

        min_freq = geopm_test_launcher.geopmread("CPUINFO::FREQ_MIN board 0")
        sticker_freq = geopm_test_launcher.geopmread("CPUINFO::FREQ_STICKER board 0")
        cls._ee_report = cls.get_run_report(1, min_freq, sticker_freq)
        cls._baseline_report = cls.get_run_report(1, sticker_freq, sticker_freq)

    @classmethod
    def tearDownClass(cls):
        """If we are not handling an exception and the GEOPM_KEEP_FILES
        environment variable is unset, clean up output.

        """
        if (sys.exc_info() == (None, None, None) and not
            cls._keep_files and not cls._skip_launch):
            os.unlink(cls._agent_conf_path)
            os.unlink(cls._ee_report)
            os.unlink(cls._baseline_report)

    def test_total_app_energy(self):
        """ Test that total app energy consumption is reasonable.

        """
        for host_name in self._ee_report.host_names():
            ee_app_totals = self._ee_report.raw_totals(host_name)
            baseline_app_totals = self._baseline_report.raw_totals(host_name)
            msg = 'Save energy on the total application'
            self.assertLess(ee_app_totals['package-energy (joules)'], baseline_app_totals['package-energy (joules)'], msg=msg)

    def test_spin_region(self):
        """ Test that netork hinted region does not get learned for.

        """
        for host_name in self._ee_report.host_names():
            for region_name in self._ee_report.region_names(host_name):
                if region_name.startswith('network_spin'):
                    ee_spin_region = self._ee_report.raw_region(host_name, region_name)
                    baseline_spin_region = self._baseline_report.raw_region(host_name, region_name)
                    self.assertFalse('requested-online-frequency' in ee_spin_region,
                                     msg='Learning for region {} did not complete'.format(region_name))
                    msg = 'Save energy on the network-hinted spin region'
                    self.assertLess(ee_spin_region['package-energy (joules)'],
                                    baseline_spin_region['package-energy (joules)'], msg=msg)

    def test_scaling_region_learned(self):
        """ Test that scaling regions have learned frequecies.

        """
        for host_name in self._ee_report.host_names():
            for region_name in self._ee_report.region_names(host_name):
                if region_name.startswith('scaling'):
                    ee_scaling_region = self._ee_report.raw_region(host_name, region_name)
                    baseline_scaling_region = self._baseline_report.raw_region(host_name, region_name)
                    self.assertTrue('requested-online-frequency' in ee_scaling_region,
                                    msg='Learning for region {} did not complete'.format(region_name))

    def test_scaling_region_perf_margin(self):
        """ Test that scaling regions runtime does not degrade more than 10%.

        """
        for host_name in self._ee_report.host_names():
            for region_name in self._ee_report.region_names(host_name):
                if region_name.startswith('scaling'):
                    ee_scaling_region = self._ee_report.raw_region(host_name, region_name)
                    baseline_scaling_region = self._baseline_report.raw_region(host_name, region_name)
                    ee_rt = ee_scaling_region['runtime (sec)']
                    baseline_rt = baseline_scaling_region['runtime (sec)']
                    msg = 'Region {} runtime degraded more that 10%'.format(region_name)
                    self.assertLess(ee_rt,
                                    baseline_rt + (baseline_rt * 0.1), msg=msg)

    def test_based_on_big_os(self):
        """TODO

        """
        big_os = [0.001, 0.002, 0.004, 0.008, 0.016, 0.032, 0.064, 0.128]


if __name__ == '__main__':
    try:
        sys.argv.remove('--skip-launch')
        _g_skip_launch = True
    except ValueError:
        pass
    unittest.main()
