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

from __future__ import absolute_import
from __future__ import division

import sys
import os
import time
import datetime
import numpy
from numpy import arange


import geopmpy.io
import geopmpy.launcher
import geopmpy.analysis

class EEAgentPolicyAnalysis(object):
    def __init__(self):
        self._host_file = None
        self._time_limit = 600
        self._node_list = None
        self._pmpi_ctl = 'process'
        self._timeout = 30
        self._num_rank = 1
        self._num_node = 1
        self._cpu_per_rank = 86
        self._tmp_files = []

    def tearDown(self):
        if sys.exc_info() == (None, None, None) and os.getenv('GEOPM_KEEP_FILES') is None:
            if self._output is not None:
                self._output.remove_files()
            for ff in self._tmp_files:
                try:
                    os.remove(ff)
                except OSError:
                    pass

    def launch(self):
        self._app_conf.write()
        self._agent_conf.write()
        with open(self._profile_name + '.log', 'a') as outfile:
            # todo append profile_name.log
            #self._tmp_files
            outfile.write(str(datetime.datetime.now()) + '\n')
            outfile.flush()
            argv = ['dummy', 'srun', '--geopm-ctl', self._pmpi_ctl,
                                                '--geopm-agent', self._agent_conf.get_agent(),
                                                '--geopm-profile', self._profile_name]

            argv.extend(['--geopm-policy', self._agent_conf.get_path()])

            if self._report_path is not None:
                argv.extend(['--geopm-report', self._report_path])
            argv.extend(['--'])
            # Use app config to get path and arguements
            argv.append(self._app_conf.get_exec_path())
            argv.append('--verbose')
            argv.extend(self._app_conf.get_exec_args())
            launcher = geopmpy.launcher.Factory().create(argv, self._num_rank, self._num_node, self._cpu_per_rank, self._timeout,
                                                         self._time_limit, self._profile_name, self._node_list, self._host_file)

            try:
                launcher.run(stdout=outfile, stderr=outfile)
            except (AttributeError, TypeError):
                if self._msr_save_path is not None:
                    self.msr_restore()
                raise

    def get_agent_energy_efficient_conf(self):
        agent = "energy_efficient"
        options = {'FREQ_MIN': self._min_freq,
                   'FREQ_MAX': self._max_freq}
        if self._perf_margin is not None:
            options['PERF_MARGIN'] = self._perf_margin
        return geopmpy.io.AgentConf(self._profile_name + '_agent.config', agent, options)

    def _get_agent_energy_efficient_single_region_report(self):
        self._agent_conf = self.get_agent_energy_efficient_conf()
        # todo append agent_conf
        #self._tmp_files
        self._app_conf = geopmpy.io.BenchConf(self._profile_name + '_app.config')
        self._app_conf.set_loop_count(self._loop_count)
        self._app_conf.append_region(self._region_name, self._big_o)
        # todo append app_conf
        #self._tmp_files
        self.launch()
        return geopmpy.io.AppOutput(self._report_path)

    def get_agent_energy_efficient_single_region_report(self):
        self._min_freq = geopmpy.launcher.geopmread("CPUINFO::FREQ_MIN board 0")
        self._max_freq = geopmpy.launcher.geopmread("CPUINFO::FREQ_STICKER board 0")
        self._loop_count = 100

        # todo create app sweep
        #spin_id = 0
        for region_name in ['timed_scaling']:#, 'scaling', 'spin', 'dgemm']
            #for big_o in arange(0.001, 0.1, 0.001):
            for big_o in [0.001, 1.0, 0.01, 0.1]:
                app_conf = geopmpy.io.BenchConf('region_{}_app.config'.format(region_name))
                app_conf.set_loop_count(self._loop_count)
                app_conf.append_region('{}_{}'.format(region_name, big_o), big_o)
                #app_conf.append_region('spin_{}'.format(++spin_id), 0.005)

                self._app_conf = app_conf
                app_conf.write()

                self._min_freq = self._max_freq
                self._profile_name = 'node_{}_rank_{}_tpr_{}_region_{}_{}'.format(
                                             self._num_node, self._num_rank, self._cpu_per_rank,
                                             region_name, big_o)
                self._report_path = self._profile_name + '.report'
                self._agent_conf = self.get_agent_energy_efficient_conf()
                self.launch()
                # todo create perf margin sweep analysis
                self._min_freq = geopmpy.launcher.geopmread("CPUINFO::FREQ_MIN board 0")
                for perf_margin in arange(0.005, 0.1, 0.01):
                    self._perf_margin = perf_margin
                    self._profile_name = 'perf_{}_node_{}_rank_{}_tpr_{}_region_{}_{}'.format(perf_margin,
                                                 self._num_node, self._num_rank, self._cpu_per_rank,
                                                 region_name, big_o)
                    self._report_path = self._profile_name + '.report'
                    self._agent_conf = self.get_agent_energy_efficient_conf()
                    self.launch()

if __name__=='__main__':
    EEAgentPolicyAnalysis().get_agent_energy_efficient_single_region_report()
