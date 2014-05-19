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
# Interval for adjusting slack multiplier (in cycles)
parser.add_option("--slack_multiplier_interval", type="int", default=0)
# Granularity of dropping decisions (in number of instructions)
parser.add_option("--dropping_granularity", type="int", default=1)

# compute check sets
parser.add_option("--compute_check_sets", action="store_true")
# read check sets from file
parser.add_option("--read_check_sets", action="store_true")
# compute optimal dropping points
parser.add_option("--compute_optimal_dropping", action="store_true")
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

parser.add_option("--static_coverage", action="store_true")

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
  "lrc"  : 7
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
# Monitoring will be enabled by software after startup
MainCPUClass.monitoring_enabled = False
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
if options.cpu_type == 'atomic':
  (MonCPUClass, test_mem_mode, FutureClass) = (AtomicSimpleMonitor, test_mem_mode, None)
elif options.cpu_type == 'timing':
  (MonCPUClass, test_mem_mode, FutureClass) = (TimingSimpleMonitor, test_mem_mode, None)
  #(MonCPUClass, test_mem_mode, FutureClass) = (AtomicSimpleMonitor, test_mem_mode, None)
else:
  raise Exception("Unknown what drop core to use for cpu_type %s" % options.cpu_type)
MonCPUClass.clock = options.monfreq
MonCPUClass.numThreads = numThreads;
# Has port to access fifo, but does not enqueue monitoring events
MonCPUClass.fifo_enabled = True
MonCPUClass.monitoring_enabled = False
# Enable slack timer so it can read from it
MonCPUClass.timer_enabled = True
# Need flag cache for monitoring core
MonCPUClass.flagcache_enabled = False
MonCPUClass.monitor_type = available_monitors[options.monitor]
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
DropCPUClass.compute_check_sets = options.compute_check_sets
DropCPUClass.read_check_sets = options.read_check_sets
DropCPUClass.compute_optimal_dropping = options.compute_optimal_dropping
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

if options.monitor == "umc":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  if options.invalidation:
    # Load the invalidation file
    DropCPUClass.invalidation_file = os.environ["GEM5"] + "/tables/umc_invalidation.txt"
    DropCPUClass.filter_file_1 = os.environ["GEM5"] + "/tables/umc_filter.txt"
    DropCPUClass.filter_ptr_file = os.environ["GEM5"] + "/tables/umc_filter_ptrs.txt"
  # Set coverage check flags
  DropCPUClass.check_load = True
  DropCPUClass.check_store = False
  DropCPUClass.check_indctrl = False
elif options.monitor == "dift" or options.monitor == "multidift":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  if options.invalidation:
    # Load the invalidation file
    DropCPUClass.invalidation_file = os.environ["GEM5"] + "/tables/dift_invalidation.txt"
    DropCPUClass.filter_file_1 = os.environ["GEM5"] + "/tables/dift_filter1.txt"
    DropCPUClass.filter_file_2 = os.environ["GEM5"] + "/tables/dift_filter2.txt"
    DropCPUClass.filter_ptr_file = os.environ["GEM5"] + "/tables/dift_filter_ptrs.txt"
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
    DropCPUClass.invalidation_file = os.environ["GEM5"] + "/tables/dift_invalidation.txt"
    DropCPUClass.filter_file_1 = os.environ["GEM5"] + "/tables/dift_filter1.txt"
    DropCPUClass.filter_file_2 = os.environ["GEM5"] + "/tables/dift_filter2.txt"
    DropCPUClass.filter_ptr_file = os.environ["GEM5"] + "/tables/dift_filter_ptrs.txt"
  # Set coverage check flags
  DropCPUClass.check_load = True
  DropCPUClass.check_store = True
  DropCPUClass.check_indctrl = False
elif options.monitor == "hb":
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
    DropCPUClass.invalidation_file = os.environ["GEM5"] + "/tables/dift_invalidation.txt"
    DropCPUClass.filter_file_1 = os.environ["GEM5"] + "/tables/dift_filter1.txt"
    DropCPUClass.filter_file_2 = os.environ["GEM5"] + "/tables/dift_filter2.txt"
    DropCPUClass.filter_ptr_file = os.environ["GEM5"] + "/tables/dift_filter_ptrs.txt"
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
    DropCPUClass.invalidation_file = os.environ["GEM5"] + "/tables/lrc_invalidation.txt"
    DropCPUClass.filter_file_1 = os.environ["GEM5"] + "/tables/lrc_filter.txt"
    DropCPUClass.filter_ptr_file = os.environ["GEM5"] + "/tables/lrc_filter_ptrs.txt"
  # Set coverage check flags
  #DropCPUClass.check_ret = True
elif options.monitor == "none":
  # Set up monitoring filter
  pass
else:
  raise Exception("Monitor not recognized: %s" % monitor)

# Create system, CPUs, bus, and memory
system = System(cpu = [MainCPUClass(cpu_id=0), MonCPUClass(cpu_id=1), DropCPUClass(cpu_id=2)],
                physmem = SimpleMemory(range=AddrRange("1GB"), latency='15ns'),
                membus = CoherentBus(), mem_mode = test_mem_mode)

# Save a list of the CPU classes. These will be used in fast-forwarding
# to create a replica CPU set that runs after fast-forward.
cpu_list = [MainCPUClass, MonCPUClass, DropCPUClass]

# Connect port between drop and monitoring cpu
system.cpu[1].monitor_port = system.cpu[2].monitor_port

# Number of CPUs
options.num_cpus = 3

# Addresses for peripherals
PERIPHERAL_ADDR_BASE = 0x50000000
FIFO_MAIN_TO_DC_OFFSET = 0x0
FIFO_DC_TO_MON_OFFSET = 0x30000
TIMER_OFFSET = 0x10000
FLAGCACHE_OFFSET = 0x20000
# Create a "fifo" memory
fifo_main_to_dc = Fifo(range=AddrRange(start=PERIPHERAL_ADDR_BASE + FIFO_MAIN_TO_DC_OFFSET, size="64kB"))
fifo_main_to_dc.fifo_size = options.fifo_size
system.fifo_main_to_dc = fifo_main_to_dc
# Create a second fifo
fifo_dc_to_mon = Fifo(range=AddrRange(start=PERIPHERAL_ADDR_BASE + FIFO_DC_TO_MON_OFFSET, size="64kB"))
fifo_dc_to_mon.fifo_size = 3
system.fifo_dc_to_mon = fifo_dc_to_mon
# Create timer
timer = PerformanceTimer(range=AddrRange(start=PERIPHERAL_ADDR_BASE + TIMER_OFFSET, size="64kB"))
if options.important_policy == "percent":
  timer.percent_overhead = options.overhead - options.important_percent
  if (timer.percent_overhead < 0):
    raise Exception("overhead for important instructions should not exceed total overhead!")
else:
  timer.percent_overhead = options.overhead
# For spec benchmarks, we set init slack here
timer.start_cycles = options.headstart_slack
timer.start_cycles_clock = MainCPUClass.clock
timer.use_start_ticks = True
# policy of forwarding important instructions
timer.important_policy = important_policy[options.important_policy]
timer.important_slack = options.important_slack
timer.important_percent = options.important_percent
timer.increment_important_only = options.increment_important_only
timer.slack_multiplier_interval = options.slack_multiplier_interval
timer.dropping_granularity = options.dropping_granularity
timer.read_slack_multiplier = options.read_slack_multiplier
timer.persistence_dir = options.backtrack_table_dir
# We can also set a probabilistic range
if options.probabilistic_drop:
  timer.seed = random.randint(-2**30,2**30)
  timer.slack_lo = -100
  timer.slack_hi = 100
system.timer = timer
# Create flag cache
flagcache = FlagCache(range=AddrRange(start=PERIPHERAL_ADDR_BASE + FLAGCACHE_OFFSET, size="64kB"))
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

options.l1i_latency = '1ps'
options.l1d_latency = '1ps'
# Remove drop core for connecting up caches
options.num_cpus = 2
if options.ruby:
  options.num_cpus = 3
  options.use_map = True
  Ruby.create_system(options, system)
  assert(options.num_cpus == len(system.ruby._cpu_ruby_ports))

  for i in xrange(options.num_cpus):
    ruby_port = system.ruby._cpu_ruby_ports[i]

    # Create the interrupt controller and connect its ports to Ruby
    system.cpu[i].createInterruptController()
    # system.cpu[i].interrupts.pio = ruby_port.master
    # system.cpu[i].interrupts.int_master = ruby_port.slave
    # system.cpu[i].interrupts.int_slave = ruby_port.master

    # Connect the cpu's cache ports to Ruby
    system.cpu[i].icache_port = ruby_port.slave
    system.cpu[i].dcache_port = ruby_port.slave
else:
  # Connect system to the bus
  system.system_port = system.membus.slave
  # Connect memory to bus
  system.physmem.port = system.membus.master
  # Set up caches for main and monitoring cores if enabled, connect to memory
  # bus, and set up interrupts
  CacheConfig.config_cache(options, system)

  if options.caches:
      icache = L1Cache(size = options.l1i_size,
                       assoc = options.l1i_assoc,
                       block_size=options.cacheline_size,
                       latency = options.l1i_latency)
      dcache = L1Cache(size = options.invalidation_cache_size,
                       assoc = options.l1d_assoc,
                       block_size=16,
                       latency = options.l1d_latency)
      system.cpu[2].addPrivateSplitL1Caches(icache, dcache)
  system.cpu[2].createInterruptController()
  if options.l2cache:
      system.cpu[2].connectAllPorts(system.tol2bus, system.membus)
  else:
      system.cpu[2].connectAllPorts(system.membus)
# Reinclude drop core for switching CPU count in Simulation.run
options.num_cpus = 3

# Run simulation
root = Root(full_system = False, system = system)
if options.fastforward_insts == 0:
    Simulation.run(options, root, system, FutureClass)
else:
    Simulation.run_ff(options, root, system, cpu_list)
