# Copyright (c) 2012 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2007 The Regents of The University of Michigan
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
# Authors: Nathan Binkert

from m5.params import *
from BaseSimpleCPU import BaseSimpleCPU

class DropSimpleCPU(BaseSimpleCPU):
    type = 'DropSimpleCPU'
    width = Param.Int(1, "CPU width")
    simulate_data_stalls = Param.Bool(False, "Simulate dcache stall cycles")
    fastmem = Param.Bool(False, "Access memory directly")
    
    forward_fifo_enabled = Param.Bool(False, "monitoring forward fifo port enabled")
    full_clock = Param.Clock('500MHz', "Full monitoring clock frequency")
    
    monitor_port = SlavePort("Monitor Port")
    monitor_type = Param.Int(0, "Type of monitor")
    
    if forward_fifo_enabled:
      forward_fifo_port = MasterPort("Forward Fifo Port")

    mpt_size = Param.Int(0x100000, "Size of memory producer tracking table")
    ipt_impl = Param.Int(0, "IPT implementation")
    ipt_tagged = Param.Bool(True, "Whether IPT is tagged")
    ipt_size = Param.Int(0x100000, "Size of instruction priority table")
    ipt_entry_size = Param.Int(16, "Size of instruction priority table entry")
    ipt_false_positive_rate = Param.Float(0.01, "False positive rate of IPT bloom filter")

    compute_check_sets = Param.Bool(False, "Compute check sets")
    max_check_set_size = Param.Int(0, "Maximum check set size to be computed")
    read_check_sets = Param.Bool(False, "Read check sets from file")
    compute_optimal_dropping = Param.Bool(False, "Compute optimal dropping points")
    read_optimal_dropping = Param.Bool(False, "Read optimal dropping points from file")