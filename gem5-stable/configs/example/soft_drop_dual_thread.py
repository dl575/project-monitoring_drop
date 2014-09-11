# Copyright (c) 2013 ARM Limited
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
import random

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
parser.add_option("--monfreq", type="string", default="250MHz")
# Size of fifo connecting main core to dropping core
parser.add_option("--fifo_size", type="int", default=16)
# Modeling atomic cache stalls
parser.add_option("--simulatestalls", action="store_true")
# Allow invalidations
parser.add_option("--invalidation", action="store_true")
# Set the desired overhead
parser.add_option("--overhead", type="float", default=0.0)
# Emulate filtering
parser.add_option("--emulate_filtering", action="store_true")
# Invalidation cache size
parser.add_option("--invalidation_cache_size", type="string", default="2kB")
# Headstart (initial) slack
parser.add_option("--headstart_slack", type="int", default=2000000)

# backtracking
parser.add_option("--backtrack", action="store_true")
# instruction priority table implementation
parser.add_option("--ipt_impl", type="string", default="table")
# Whether IPT is tagged
parser.add_option("--ipt_tagless", action="store_true")
# instruction priority table false positive rate (bloom filter implementation)
parser.add_option("--ipt_fpr", type="float", default=0.0)
# instruction priority table size
parser.add_option("--ipt_size", type="int", default=0x100000)
# instruction priority table entry size
parser.add_option("--ipt_entry_size", type="int", default=1)
# memory producer tracking table size
parser.add_option("--mpt_size", type="int", default=0x100000)
parser.add_option("--backtrack_read_table", action="store_true")
parser.add_option("--backtrack_write_table", action="store_true")
# backtracking table directory
parser.add_option("--backtrack_table_dir", type="string", default="m5out")
# Policy for forwarding important instructions
parser.add_option("--important_policy", type="string", default="always")
# Additional slack for important instructions
parser.add_option("--important_slack", type="int", default=2000000)
# Additional slack for important instructions as percent of total cycles
parser.add_option("--important_percent", type="float", default=0.0)
# Increment slack on important instructions only
parser.add_option("--increment_important_only", action="store_true")
# Read slack multiplier from file
parser.add_option("--read_slack_multiplier", action="store_true")
# read optimal dropping points from file
parser.add_option("--read_optimal_dropping", action="store_true")
# perform optimal dropping
parser.add_option("--optimal_dropping", action="store_true")
# Number of instructions to fast-forward
# During fast-forwarding, full monitoring is performed. Invalidation is enabled
# after fast-forward period.
parser.add_option("--fastforward_insts", type="int", default=0)

# Enable coverage
parser.add_option("--control_coverage", action="store_true")
# Set desired coverage
parser.add_option("--coverage", type="float", default=1.0)
# Set coverage adjustment frequency (after x checks, we will readjust)
parser.add_option("--coverage_adjust", type="int", default=0)
# Enable probabilistic drop
parser.add_option("--probabilistic_drop", action="store_true")
# Output static coverage
parser.add_option("--static_coverage", action="store_true")

# Drop only at source set tag operations
parser.add_option("--source_dropping", action="store_true")

# Configuration for WCET bound mode
parser.add_option("--wcet", action="store_true")

available_monitors = {
  "none" : 0,
  "umc"  : 1,
  "dift" : 2,
  "bc"   : 3,
  "sec"  : 4,
  "hb"   : 5,
  "multidift" : 6,
  "lrc"  : 7,
  "diftrf" : 8,
  "ls"   : 9
}

important_policy = {
  "always"  : 0,
  "slack"   : 1,
  "percent" : 2,
  "unified" : 3
}

ipt_impl = {
  "table" : 0,
  "bloom" : 1
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
MainCPUClass.clock = options.clock
MainCPUClass.numThreads = numThreads;
MainCPUClass.fifo_enabled = True
# The CPU is a main core
MainCPUClass.main_core = True
# Monitoring will be enabled by software after startup
MainCPUClass.monitoring_enabled = False
MainCPUClass.monitor_type = available_monitors[options.monitor]
# Enable slack timer so it can write to it
MainCPUClass.timer_enabled = True
# Don't need flag cache for main core
MainCPUClass.flagcache_enabled = False
# Hard real-time
MainCPUClass.hard_wcet = options.wcet
if (options.simulatestalls and options.cpu_type == 'atomic'):
    # Simulate cache stalls in atomic
    MainCPUClass.simulate_inst_stalls = True
    MainCPUClass.simulate_data_stalls = True

# Create new CPU type for monitoring core
# if options.cpu_type == 'atomic':
#   (MonCPUClass, test_mem_mode, FutureClass) = (AtomicSimpleMonitor, test_mem_mode, None)
# elif options.cpu_type == 'timing':
#   (MonCPUClass, test_mem_mode, FutureClass) = (TimingSimpleMonitor, test_mem_mode, None)
#   #(MonCPUClass, test_mem_mode, FutureClass) = (AtomicSimpleMonitor, test_mem_mode, None)
# else:
#   raise Exception("Unknown what drop core to use for cpu_type %s" % options.cpu_type)
# MonCPUClass.clock = options.monfreq
# Core-based monitor
(MonCPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)
MonCPUClass.clock = options.monfreq
MonCPUClass.numThreads = numThreads;
# Has port to access fifo, but does not enqueue monitoring events
MonCPUClass.fifo_enabled = True
MonCPUClass.monitoring_enabled = False
MonCPUClass.monitor_type = available_monitors[options.monitor]
# Enable slack timer so it can read from it
MonCPUClass.timer_enabled = True
# Monitoring core can access flagcache for revalidation
MonCPUClass.flagcache_enabled = True
#MonCPUClass.monitor_type = available_monitors[options.monitor]
if (options.simulatestalls and options.cpu_type == 'atomic'):
    # Simulate d cache stalls
    MonCPUClass.simulate_data_stalls = True

# Create drop core
prev_cpu_type = options.cpu_type
if options.cpu_type == 'atomic':
  options.cpu_type = 'drop'
elif options.cpu_type == 'timing':
  options.cpu_type = 'drop_timing'
else:
  raise Exception("Unknown what drop core to use for cpu_type %s" % options.cpu_type)
(DropCPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)
options.cpu_type = prev_cpu_type
DropCPUClass.numThreads = numThreads;
# Has port to access fifo, but does not enqueue monitoring events
DropCPUClass.fifo_enabled = True
DropCPUClass.monitoring_enabled = False
DropCPUClass.monitor_type = available_monitors[options.monitor]
if options.invalidation:
    # Enable slack timer so it can read from it
    DropCPUClass.timer_enabled = True
    # Need flag cache for monitoring core
    DropCPUClass.flagcache_enabled = True
if (options.simulatestalls) and options.cpu_type == 'atomic':
    # Simulate d cache stalls
    DropCPUClass.simulate_data_stalls = True

DropCPUClass.clock = MainCPUClass.clock
DropCPUClass.full_clock = MonCPUClass.clock
# Enable output to second fifo
DropCPUClass.forward_fifo_enabled = True
# Emulate filtering set
DropCPUClass.emulate_filtering = options.emulate_filtering
# Backtrack
DropCPUClass.backtrack = options.backtrack
DropCPUClass.backtrack_read_table = options.backtrack_read_table
DropCPUClass.backtrack_write_table = options.backtrack_write_table
DropCPUClass.backtrack_table_dir = options.backtrack_table_dir
DropCPUClass.read_optimal_dropping = options.read_optimal_dropping
DropCPUClass.optimal_dropping = options.optimal_dropping
DropCPUClass.ipt_impl = ipt_impl[options.ipt_impl]
DropCPUClass.ipt_tagged = not options.ipt_tagless
DropCPUClass.ipt_false_positive_rate = options.ipt_fpr
DropCPUClass.ipt_size = options.ipt_size
DropCPUClass.ipt_entry_size = options.ipt_entry_size
DropCPUClass.mpt_size = options.mpt_size
# Coverage options
DropCPUClass.target_coverage = options.coverage
DropCPUClass.check_frequency = options.coverage_adjust
if options.probabilistic_drop:
  DropCPUClass.print_checkid = True
DropCPUClass.print_static_coverage = options.static_coverage
if options.source_dropping:
  DropCPUClass.source_dropping = True

table_dir = os.environ["GEM5"] + "/tables/"
if options.monitor == "umc":
  monitor_bin = "umc_soft_drop"
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  if options.source_dropping:
    # Mark stores at set tag operations for the purpose of source dropping
    MainCPUClass.settag_store = True
  if options.invalidation:
    # Load the invalidation file
    DropCPUClass.invalidation_file = table_dir + "umc_invalidation.txt"
    DropCPUClass.filter_file_1 = table_dir + "umc_filter.txt"
    DropCPUClass.filter_ptr_file = table_dir + "umc_filter_ptrs.txt"
  # Set coverage check flags
  DropCPUClass.check_load = True
  DropCPUClass.check_store = False
  DropCPUClass.check_indctrl = False
elif options.monitor == "dift":
  monitor_bin = "dift_soft_drop"
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  if options.invalidation:
    # Load the invalidation file
    DropCPUClass.invalidation_file = table_dir + "dift_invalidation.txt"
    DropCPUClass.filter_file_1 = table_dir + "dift_filter1.txt"
    DropCPUClass.filter_file_2 = table_dir + "dift_filter2.txt"
    DropCPUClass.filter_ptr_file = table_dir + "dift_filter_ptrs.txt"
    # Set coverage check flags
    DropCPUClass.check_load = False
    DropCPUClass.check_store = False
    DropCPUClass.check_indctrl = True
elif options.monitor == "multidift":
  monitor_bin = "multidift_soft_drop"
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  if options.invalidation:
    # Load the invalidation file
    DropCPUClass.invalidation_file = table_dir + "dift_invalidation.txt"
    DropCPUClass.filter_file_1 = table_dir + "dift_filter1.txt"
    DropCPUClass.filter_file_2 = table_dir + "dift_filter2.txt"
    DropCPUClass.filter_ptr_file = table_dir + "dift_filter_ptrs.txt"
    # Set coverage check flags
    DropCPUClass.check_load = False
    DropCPUClass.check_store = False
    DropCPUClass.check_indctrl = True
elif options.monitor == "bc":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  #MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_intand = True
  MainCPUClass.monitoring_filter_intmov = True
  MainCPUClass.monitoring_filter_intadd = True
  MainCPUClass.monitoring_filter_intsub = True
  MainCPUClass.monitoring_filter_intmul = True
  if options.invalidation:
    # Load the invalidation file
    DropCPUClass.invalidation_file = table_dir + "dift_invalidation.txt"
    DropCPUClass.filter_file_1 = table_dir + "dift_filter1.txt"
    DropCPUClass.filter_file_2 = table_dir + "dift_filter2.txt"
    DropCPUClass.filter_ptr_file = table_dir + "dift_filter_ptrs.txt"
  # Set coverage check flags
  DropCPUClass.check_load = True
  DropCPUClass.check_store = True
  DropCPUClass.check_indctrl = False
elif options.monitor == "hb":
  monitor_bin = "hb_soft_drop"
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  # MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_intand = True
  MainCPUClass.monitoring_filter_intmov = True
  MainCPUClass.monitoring_filter_intadd = True
  MainCPUClass.monitoring_filter_intsub = True
  MainCPUClass.monitoring_filter_intmul = True
  if options.invalidation:
    # Load the invalidation file
    DropCPUClass.invalidation_file = table_dir + "dift_invalidation.txt"
    DropCPUClass.filter_file_1 = table_dir + "dift_filter1.txt"
    DropCPUClass.filter_file_2 = table_dir + "dift_filter2.txt"
    DropCPUClass.filter_ptr_file = table_dir + "dift_filter_ptrs.txt"
  # Set coverage check flags
  DropCPUClass.check_load = True
  DropCPUClass.check_store = True
  DropCPUClass.check_indctrl = False
elif options.monitor == "lrc":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  if options.invalidation:
    # Load the invalidation file
    DropCPUClass.invalidation_file = table_dir + "lrc_invalidation.txt"
    DropCPUClass.filter_file_1 = table_dir + "lrc_filter.txt"
    DropCPUClass.filter_ptr_file = table_dir + "lrc_filter_ptrs.txt"
  # Set coverage check flags
  #DropCPUClass.check_ret = True
elif options.monitor == "ls":
  monitor_bin = "ls_soft_drop"
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  if options.invalidation:
    # Load the invalidation file
    DropCPUClass.invalidation_file = table_dir + "ls_invalidation.txt"
    DropCPUClass.filter_file_1 = table_dir + "ls_filter1.txt"
    DropCPUClass.filter_file_2 = table_dir + "ls_filter2.txt"
    DropCPUClass.filter_ptr_file = table_dir + "ls_filter_ptrs.txt"
  # Set coverage check flags
  DropCPUClass.check_load = True
  DropCPUClass.check_store = True
  DropCPUClass.check_indctrl = False
elif options.monitor == "none":
  # FIXME: need an empty monitor
  monitor_bin = "umc_soft_drop"
else:
  raise Exception("Monitor not recognized: %s" % monitor)

# Create system, CPUs, bus, and memory
system = System(cpu = [MainCPUClass(cpu_id=0), MainCPUClass(cpu_id=1),
                       MonCPUClass(cpu_id=2),  MonCPUClass(cpu_id=3),
                       DropCPUClass(cpu_id=4), DropCPUClass(cpu_id=5)],
                physmem = SimpleMemory(range=AddrRange("1GB"), latency='30ns'),
                membus = CoherentBus(), mem_mode = test_mem_mode)

# Save a list of the CPU classes. These will be used in fast-forwarding
# to create a replica CPU set that runs after fast-forward.
cpu_list = [MainCPUClass, MonCPUClass, DropCPUClass]

# Connect port between drop and monitoring cpu
system.cpu[2].monitor_port = system.cpu[4].monitor_port
system.cpu[3].monitor_port = system.cpu[5].monitor_port

# Number of CPUs
options.num_cpus = 6

# Addresses for peripherals
PERIPHERAL_ADDR_BASE = 0x50000000
FIFO_MAIN_TO_DC_OFFSET = 0x0
FIFO_DC_TO_MON_OFFSET = 0x30000
TIMER_OFFSET = 0x10000
FLAGCACHE_OFFSET = 0x20000
# monitoring structures for thread 0
# Create a "fifo" memory
fifo_main_to_dc_0 = Fifo(range=AddrRange(start=PERIPHERAL_ADDR_BASE + FIFO_MAIN_TO_DC_OFFSET, size="64kB"))
fifo_main_to_dc_0.fifo_size = options.fifo_size
system.fifo_main_to_dc_0 = fifo_main_to_dc_0
# Create a second fifo
fifo_dc_to_mon_0 = Fifo(range=AddrRange(start=PERIPHERAL_ADDR_BASE + FIFO_DC_TO_MON_OFFSET, size="64kB"))
fifo_dc_to_mon_0.fifo_size = 2
system.fifo_dc_to_mon_0 = fifo_dc_to_mon_0
# Create timer
timer_0 = PerformanceTimer(range=AddrRange(start=PERIPHERAL_ADDR_BASE + TIMER_OFFSET, size="64kB"))
timer_0.percent_overhead = options.overhead
# For spec benchmarks, we set init slack here
timer_0.start_cycles = options.headstart_slack
timer_0.start_cycles_clock = MainCPUClass.clock
timer_0.use_start_ticks = True
# We can also set a probabilistic range
if options.probabilistic_drop:
  timer_0.seed = random.randint(-2**30,2**30)
  timer_0.slack_lo = -100
  timer_0.slack_hi = 100
system.timer_0 = timer_0
# Create flag cache
flagcache_0 = FlagCache(range=AddrRange(start=PERIPHERAL_ADDR_BASE + FLAGCACHE_OFFSET, size="64kB"))
system.flagcache_0 = flagcache_0

# Connect CPU to fifo
if system.cpu[0].fifo_enabled:
  system.cpu[0].fifo_port = system.fifo_main_to_dc_0.port
if system.cpu[2].fifo_enabled:
  system.cpu[2].fifo_port = system.fifo_dc_to_mon_0.port
if system.cpu[4].fifo_enabled:
  system.cpu[4].fifo_port = system.fifo_main_to_dc_0.port
if system.cpu[4].forward_fifo_enabled:
  system.cpu[4].forward_fifo_port = system.fifo_dc_to_mon_0.port

for i in (0, 2, 4):
  # Connect CPU to timer
  if system.cpu[i].timer_enabled:
    system.cpu[i].timer_port = system.timer_0.port
  # Connect CPU to flag cache
  if system.cpu[i].flagcache_enabled:
    system.cpu[i].flagcache_port = system.flagcache_0.port

# monitoring structures for thread 1
# Create a "fifo" memory
fifo_main_to_dc_1 = Fifo(range=AddrRange(start=PERIPHERAL_ADDR_BASE + FIFO_MAIN_TO_DC_OFFSET, size="64kB"))
fifo_main_to_dc_1.fifo_size = options.fifo_size
system.fifo_main_to_dc_1 = fifo_main_to_dc_1
# Create a second fifo
fifo_dc_to_mon_1 = Fifo(range=AddrRange(start=PERIPHERAL_ADDR_BASE + FIFO_DC_TO_MON_OFFSET, size="64kB"))
fifo_dc_to_mon_1.fifo_size = 2
system.fifo_dc_to_mon_1 = fifo_dc_to_mon_1
# Create timer
timer_1 = PerformanceTimer(range=AddrRange(start=PERIPHERAL_ADDR_BASE + TIMER_OFFSET, size="64kB"))
timer_1.percent_overhead = options.overhead
# For spec benchmarks, we set init slack here
timer_1.start_cycles = options.headstart_slack
timer_1.start_cycles_clock = MainCPUClass.clock
timer_1.use_start_ticks = True
# We can also set a probabilistic range
if options.probabilistic_drop:
  timer_1.seed = random.randint(-2**30,2**30)
  timer_1.slack_lo = -100
  timer_1.slack_hi = 100
system.timer_1 = timer_1
# Create flag cache
flagcache_1 = FlagCache(range=AddrRange(start=PERIPHERAL_ADDR_BASE + FLAGCACHE_OFFSET, size="64kB"))
system.flagcache_1 = flagcache_1

# Connect CPU to fifo
if system.cpu[1].fifo_enabled:
  system.cpu[1].fifo_port = system.fifo_main_to_dc_1.port
if system.cpu[3].fifo_enabled:
  system.cpu[3].fifo_port = system.fifo_dc_to_mon_1.port
if system.cpu[5].fifo_enabled:
  system.cpu[5].fifo_port = system.fifo_main_to_dc_1.port
if system.cpu[5].forward_fifo_enabled:
  system.cpu[5].forward_fifo_port = system.fifo_dc_to_mon_1.port

for i in (1, 3, 5):
  # Connect CPU to timer
  if system.cpu[i].timer_enabled:
    system.cpu[i].timer_port = system.timer_1.port
  # Connect CPU to flag cache
  if system.cpu[i].flagcache_enabled:
    system.cpu[i].flagcache_port = system.flagcache_1.port

# Assign programs
process0 = LiveProcess()
# If passed a program from script options
if options.cmd:
  process0.executable = options.cmd
  process0.cmd = [options.cmd] + options.options.split()
else:
  print "error: missing program command line"
  sys.exit(1)

if options.input != "":
    process0.input = options.input

system.cpu[0].workload = process0
# assign the same process to cpu[1]
system.cpu[1].workload = process0
# enable monitoring by default on cpu[1]
system.cpu[1].monitoring_enabled = True

process2 = LiveProcess()
process2.executable = os.environ["GEM5"] + ("/tests/monitoring/%s%s.arm" % (monitor_bin, "" if options.invalidation else "_full"))
process2.cmd = ""
system.cpu[2].workload = process2

# same executable for cpu[3]
process3 = LiveProcess()
process3.executable = os.environ["GEM5"] + ("/tests/monitoring/%s%s.arm" % (monitor_bin, "" if options.invalidation else "_full"))
process3.cmd = ""
system.cpu[3].workload = process3

process4 = LiveProcess()
process4.executable = os.environ["GEM5"] + "/tests/test-progs/dummy/dummy.arm"
process4.cmd = ""
system.cpu[4].workload = process4

process5 = LiveProcess()
process5.executable = os.environ["GEM5"] + "/tests/test-progs/dummy/dummy.arm"
process5.cmd = ""
system.cpu[5].workload = process5

options.l1i_latency = '1ps'
options.l1d_latency = '1ps'
# Remove drop cores for connecting up caches
options.num_cpus = 4
# Connect system to the bus
system.system_port = system.membus.slave
# Connect memory to bus
system.physmem.port = system.membus.master
# Set up caches for main and monitoring cores if enabled, connect to memory
# bus, and set up interrupts
CacheConfig.config_cache(options, system)

if options.caches:
  for i in (4, 5):
    icache = L1Cache(size = options.l1i_size,
                     assoc = options.l1i_assoc,
                     block_size=options.cacheline_size,
                     latency = options.l1i_latency)
    dcache = L1Cache(size = options.invalidation_cache_size,
                     assoc = options.l1d_assoc,
                     block_size=16,
                     latency = options.l1d_latency)
    system.cpu[i].addPrivateSplitL1Caches(icache, dcache)
system.cpu[4].createInterruptController()
system.cpu[5].createInterruptController()
if options.l2cache:
    system.cpu[4].connectAllPorts(system.tol2bus, system.membus)
    system.cpu[5].connectAllPorts(system.tol2bus, system.membus)
else:
    system.cpu[4].connectAllPorts(system.membus)
    system.cpu[5].connectAllPorts(system.membus)
# Reinclude drop core for switching CPU count in Simulation.run
options.num_cpus = 6

# Run simulation
root = Root(full_system = False, system = system)
if options.fastforward_insts == 0:
    Simulation.run(options, root, system, FutureClass)
else:
    Simulation.run_ff(options, root, system, cpu_list)
