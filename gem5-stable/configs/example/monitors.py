monitor = os.environ["MONITOR"]
model = os.environ["MODEL"]

# Kindle config
MainCPUClass.clock = '500MHz'
MonCPUClass.clock = '500MHz'
# Flexcore config
# MainCPUClass.clock = '1250MHz'
# MonCPUClass.clock = '7500MHz'

# Create system, CPUs, bus, and memory
# Kindle-like configuration
system = System(cpu = [MainCPUClass(cpu_id=0), MonCPUClass(cpu_id=1)],
                physmem = SimpleMemory(range=AddrRange("512MB"), latency='15ns'),
                membus = CoherentBus(), mem_mode = test_mem_mode)
# system = System(cpu = [MainCPUClass(cpu_id=0), MonCPUClass(cpu_id=1)],
#                 physmem = SimpleMemory(range=AddrRange("512MB"), latency='12ns'),
#                 membus = CoherentBus(), mem_mode = test_mem_mode)

# Configure cache size
# Kindle-like configuration
options.l1i_size = '16kB'
options.l1d_size = '16kB'
options.l1i_latency = '1ps'
options.l1d_latency = '1ps'
# options.l1i_size = '32kB'
# options.l1d_size = '32kB'

#######################################
# UMC
#######################################
if monitor == "UMC_FULL":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MonCPUClass.clock = '5500MHz'
  if model == 'ATOMIC':
    delay = 22
  if model == 'TIMING':
    delay = 121
  # Define monitoring executable
  monitor_bin = "umc_full"
elif monitor == "UMC_SWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MonCPUClass.clock = '7500MHz'
  if model == 'ATOMIC':
    delay = 30
  if model == 'TIMING':
    delay = 173
  # Define monitoring executable
  monitor_bin = "umc_swdrop"
elif monitor == "UMC_HWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MonCPUClass.clock = '7250MHz'
  # Load the invalidation file
  MonCPUClass.invalidation_file = "tables/umc_invalidation.txt"
  # Define monitoring executable
  monitor_bin = "umc_hwdrop"
elif monitor == "UMC_HWFILTER":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MonCPUClass.clock = '6500MHz'
  # Load the invalidation file
  MonCPUClass.invalidation_file = "tables/umc_invalidation.txt"
  MonCPUClass.filter_file_1 = "tables/umc_filter.txt"
  MonCPUClass.filter_ptr_file = "tables/umc_filter_ptrs.txt"
  # Define monitoring executable
  monitor_bin = "umc_hwfilter"
  

#######################################
# LRC
#######################################
elif monitor == "LRC_FULL":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  MonCPUClass.clock = '3750MHz'
  if model == 'ATOMIC':
    delay = 15
  if model == 'TIMING':
    delay = 76
  # Define monitoring executable
  monitor_bin = "lrc_full"
elif monitor == "LRC_SWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  MonCPUClass.clock = '4750MHz'
  if model == 'ATOMIC':
    delay = 19
  if model == 'TIMING':
    delay = 21
  # Define monitoring executable
  monitor_bin = "lrc_swdrop"
elif monitor == "LRC_HWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  MonCPUClass.clock = '5500MHz'
  # Load the invalidation file
  MonCPUClass.invalidation_file = "tables/lrc_invalidation.txt"
  # Define monitoring executable
  monitor_bin = "lrc_hwdrop"
elif monitor == "LRC_HWFILTER":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  MonCPUClass.clock = '5000MHz'
  # Load the invalidation file
  MonCPUClass.invalidation_file = "tables/lrc_invalidation.txt"
  MonCPUClass.filter_file_1 = "tables/lrc_filter.txt"
  MonCPUClass.filter_ptr_file = "tables/lrc_filter_ptrs.txt"
  # Define monitoring executable
  monitor_bin = "lrc_hwfilter"

#######################################
# DIFT
#######################################
# Full monitoring always
elif monitor == "DIFT_FULL":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  MonCPUClass.clock = '7500MHz'
  if model == 'ATOMIC':
    delay = 30
  if model == 'TIMING':
    delay = 167
  # Define monitoring executable
  monitor_bin = "dift_full"
# Drop handled in software
elif monitor == "DIFT_SWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  MonCPUClass.clock = '8500MHz'
  if model == 'ATOMIC':
    delay = 34
  if model == 'TIMING':
    delay = 345
  # Define monitoring executable
  monitor_bin = "dift_swdrop"
# Drop handled in hardware without filtering
elif monitor == "DIFT_HWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  MonCPUClass.clock = '11500MHz'
  # Load the invalidation file
  MonCPUClass.invalidation_file = "tables/dift_invalidation.txt"
  # Define monitoring executable
  monitor_bin = "dift_hwdrop"
# Drop handled in hardware with filtering
elif monitor == "DIFT_HWFILTER":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  MonCPUClass.clock = '11500MHz'
  # Load the invalidation file
  MonCPUClass.invalidation_file = "tables/dift_invalidation.txt"
  # Load the filter tables
  MonCPUClass.filter_file_1 = "tables/dift_filter1.txt"
  MonCPUClass.filter_file_2 = "tables/dift_filter2.txt"
  MonCPUClass.filter_ptr_file = "tables/dift_filter_ptrs.txt"
  # Define monitoring executable
  monitor_bin = "dift_hwfilter"

else:
  raise Exception("Monitor not recognized: %s" % monitor)

