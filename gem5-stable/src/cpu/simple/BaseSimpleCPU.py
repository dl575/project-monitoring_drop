# Copyright (c) 2008 The Hewlett-Packard Development Company
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Gabe Black

from m5.defines import buildEnv
from m5.params import *
from BaseCPU import BaseCPU
from DummyChecker import DummyChecker

class BaseSimpleCPU(BaseCPU):
    type = 'BaseSimpleCPU'
    abstract = True

    # Monitoring parameters
    fifo_enabled = Param.Bool(False, "monitoring fifo port enabled")
    monitoring_enabled = Param.Bool(False, "monitoring enabled")
    timer_enabled = Param.Bool(False, "timer enabled")
    flagcache_enabled = Param.Bool(False, "flag cache enabled")
    invalidation_file = Param.String("", "invalidation file name")
    filter_file_1 = Param.String("", "filtering file")
    filter_file_2 = Param.String("", "filtering file")
    filter_ptr_file = Param.String("", "filtering ptr file")
    # Monitoring filter parameters
    monitoring_filter_load = Param.Bool(False, "monitoring load instructions")
    monitoring_filter_store = Param.Bool(False, "monitoring store instructions")
    monitoring_filter_call = Param.Bool(False, "monitoring call instructions")
    monitoring_filter_ret = Param.Bool(False, "monitoring return instructions")
    monitoring_filter_intalu = Param.Bool(False, "monitoring integer ALU instructions")
    monitoring_filter_intand = Param.Bool(False, "monitoring integer AND instructions")
    monitoring_filter_intmov = Param.Bool(False, "monitoring integer move instructions")
    monitoring_filter_intadd = Param.Bool(False, "monitoring integer add instructions")
    monitoring_filter_intsub = Param.Bool(False, "monitoring integer subtract instructions")
    monitoring_filter_intmul = Param.Bool(False, "monitoring integer multiply instructions")
    monitoring_filter_indctrl = Param.Bool(False, "monitoring indirect control instructions")
    # Check definition parameters
    check_load = Param.Bool(False, "check load instructions")
    check_store = Param.Bool(False, "check store instructions")
    check_indctrl = Param.Bool(False, "check indirect control instructions")
    # Coverage Settings
    target_coverage = Param.Float(1, "Target coverage rate")
    check_frequency = Param.Unsigned(0, "How often to evaluate coverage")
    # Check if WCET violated
    hard_wcet = Param.Bool(True, "check if hard real-time deadline met")
    emulate_filtering = Param.Bool(False, "emulate filtering but don't perform")
    backtrack = Param.Bool(False, "backtrack to find out important instructions")
    backtrack_read_table = Param.Bool(False, "read backtrack table at start of simulation")
    backtrack_write_table = Param.Bool(False, "write backtrack table at end of simulation")
    backtrack_table_dir = Param.String("", "directory to read/write backtrack table")

    if fifo_enabled:
      fifo_port = MasterPort("Fifo Port")

    if timer_enabled:
      timer_port = MasterPort("Timer Port")
    
    if flagcache_enabled:
      flagcache_port = MasterPort("Flagcache Port")

    def addCheckerCpu(self):
        if buildEnv['TARGET_ISA'] in ['arm']:
            from ArmTLB import ArmTLB

            self.checker = DummyChecker(workload = self.workload)
            self.checker.itb = ArmTLB(size = self.itb.size)
            self.checker.dtb = ArmTLB(size = self.dtb.size)
        else:
            print "ERROR: Checker only supported under ARM ISA!"
            exit(1)
