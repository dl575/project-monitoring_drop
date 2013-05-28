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
# Copyright (c) 2006-2008 The Regents of The University of Michigan
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
# Authors: Steve Reinhardt

# Simple test script
#
# "m5 test.py"

import optparse
import sys
import os

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../common')
addToPath('../ruby')

import Options
import Ruby
import Simulation
import CacheConfig
from Caches import *
from cpu2000 import *

parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addSEOptions(parser)

# Monitor
parser.add_option("--monitor", type="string", default="umc")
# Monitor frequency
parser.add_option("--monfreq", type="string", default="0.5GHz")
# Modeling atomic cache stalls
parser.add_option("--simulatestalls", action="store_true")

available_monitors = {
  "none" : 0,
  "umc"  : 1,
  "dift" : 2,
  "bc"   : 3,
  "sec"  : 4,
  "hb"   : 5
}


if '--ruby' in sys.argv:
    Ruby.define_options(parser)

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# Number of threads per CPU
numThreads = 1

# Create new CPU type for main core
(MainCPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)
MainCPUClass.numThreads = numThreads;
MainCPUClass.fifo_enabled = True
MainCPUClass.monitoring_enabled = False
# Enable slack timer so it can write to it
MainCPUClass.timer_enabled = True
# Don't need flag cache for main core
MainCPUClass.flagcache_enabled = False
# Not hard real-time
MainCPUClass.hard_wcet = False
if (options.cpu_type == 'atomic'):
    # Simulate cache stalls in atomic
    MainCPUClass.simulate_inst_stalls = True
    MainCPUClass.simulate_data_stalls = True

# Create new CPU type for monitoring core
(MonCPUClass, test_mem_mode, FutureClass) = (AtomicSimpleMonitor, test_mem_mode, None)
MonCPUClass.numThreads = numThreads;
# Has port to access fifo, but does not enqueue monitoring events
MonCPUClass.fifo_enabled = True
MonCPUClass.monitoring_enabled = False
# Enable slack timer so it can read from it
MonCPUClass.timer_enabled = True
# Need flag cache for monitoring core
MonCPUClass.flagcache_enabled = False
MonCPUClass.monitor_type = available_monitors[options.monitor]

if (options.cpu_type == 'atomic'):
    # Simulate d cache stalls
    MonCPUClass.simulate_data_stalls = True

# Create drop core
options.cpu_type = 'drop'
(DropCPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)
DropCPUClass.numThreads = numThreads;
# Has port to access fifo, but does not enqueue monitoring events
DropCPUClass.fifo_enabled = True
DropCPUClass.monitoring_enabled = False
# Enable slack timer so it can read from it
DropCPUClass.timer_enabled = True
# Need flag cache for monitoring core
DropCPUClass.flagcache_enabled = True
# Simulate d cache stalls
DropCPUClass.simulate_data_stalls = True

invalidation_cpu = DropCPUClass

# Set up monitoring filter
execfile( os.path.dirname(os.path.realpath(__file__)) + "/monitors.py" )

MonCPUClass.clock = options.monfreq
DropCPUClass.clock = MainCPUClass.clock
DropCPUClass.full_clock = MonCPUClass.clock
# Enable output to second fifo
DropCPUClass.forward_fifo_enabled = True

# Create system, CPUs, bus, and memory
system = System(cpu = [MainCPUClass(cpu_id=0), MonCPUClass(cpu_id=1), DropCPUClass(cpu_id=2)],
                physmem = SimpleMemory(range=AddrRange("512MB"), latency=mem_latency),
                membus = CoherentBus(), mem_mode = test_mem_mode)

# Connect port between drop and monitoring cpu
system.cpu[1].monitor_port = system.cpu[2].monitor_port

# Number of CPUs
options.num_cpus = 3

# Create a "fifo" memory
fifo_main_to_dc = Fifo(range=AddrRange(start=0x30000000, size="64kB")) 
# fifo.fifo_size = 32
system.fifo_main_to_dc = fifo_main_to_dc
# Create a second fifo
fifo_dc_to_mon = Fifo(range=AddrRange(start=0x30030000, size="64kB")) 
fifo_dc_to_mon.fifo_size = 2
system.fifo_dc_to_mon = fifo_dc_to_mon
# Create timer
timer = PerformanceTimer(range=AddrRange(start=0x30010000, size="64kB"))
timer.percent_overhead = 0.2
system.timer = timer
# Create flag cache
flagcache = FlagCache(range=AddrRange(start=0x30020000, size="64kB"))
system.flagcache = flagcache

# Connect CPU to fifo
if system.cpu[0].fifo_enabled:
  system.cpu[0].fifo_port = system.fifo_main_to_dc.port
if system.cpu[1].fifo_enabled:
  system.cpu[1].fifo_port = system.fifo_dc_to_mon.port
if system.cpu[2].fifo_enabled:
  system.cpu[2].fifo_port = system.fifo_main_to_dc.port
if system.cpu[2].forward_fifo_enabled:
  system.cpu[2].forward_fifo_port = system.fifo_dc_to_mon.port
  

for i in range(options.num_cpus):
  # Connect CPU to timer
  if system.cpu[i].timer_enabled:
    system.cpu[i].timer_port = system.timer.port
  # Connect CPU to flag cache
  if system.cpu[i].flagcache_enabled:
    system.cpu[i].flagcache_port = system.flagcache.port

# Assign programs
process0 = LiveProcess()
# If passed a program from script options
if options.cmd:
  process0.executable = options.cmd
  process0.cmd = [options.cmd] + options.options.split()
else:
  #process0.executable = os.environ["GEM5"] + "/tests/malarden_monitor/malarden_wcet.arm"
  process0.executable = os.environ["GEM5"] + "/tests/monitoring/timer_monitor.arm"
  # process0.executable = os.environ["GEM5"] + "../../papabench/sw/airborne/autopilot/autopilot.elf"
  process0.cmd = ""
system.cpu[0].workload = process0

process1 = LiveProcess()
# load a dummy executable file to avoid polluting tag memory space
process1.executable = os.environ["GEM5"] + "/tests/test-progs/dummy/dummy.arm"
process1.cmd = ""
system.cpu[1].workload = process1

process2 = LiveProcess()
process2.executable = process1.executable
process2.cmd = process1.cmd
system.cpu[2].workload = process2

# Connect system to the bus
system.system_port = system.membus.slave
# Connect memory to bus
system.physmem.port = system.membus.master
# Set up caches if enabled, connect to memory bus, and set up interrupts
options.num_cpus = 2
CacheConfig.config_cache(options, system)

if options.caches:
    icache = L1Cache(size = options.l1i_size,
                     assoc = options.l1i_assoc,
                     block_size=options.cacheline_size,
                     latency = options.l1i_latency)
    dcache = L1Cache(size = '2kB',
                     assoc = options.l1d_assoc,
                     block_size=options.cacheline_size,
                     latency = options.l1d_latency)
    system.cpu[2].addPrivateSplitL1Caches(icache, dcache)
system.cpu[2].connectAllPorts(system.membus)
system.cpu[2].createInterruptController()

# Run simulation
root = Root(full_system = False, system = system)
Simulation.run(options, root, system, FutureClass)
