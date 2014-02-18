monitor = os.environ["MONITOR"]
model = os.environ["MODEL"]

# Kindle config
MainCPUClass.clock = '500MHz'
MonCPUClass.clock = '500MHz'
# Flexcore config
# MainCPUClass.clock = '1250MHz'
# MonCPUClass.clock = '7500MHz'

mem_latency = '15ns'

# Configure cache size
# Kindle-like configuration
options.l1i_size = '16kB'
options.l1d_size = '16kB'

options.l1i_latency = '1ps'
options.l1d_latency = '1ps'

#######################################
# UMC
#######################################
if monitor == "UMC_FULL":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  if model == 'ATOMIC':
    delay = 19
  if model == 'TIMING':
    delay = 30
  if model == 'FLEX':
    MonCPUClass.clock = '4750MHz'
    delay = 91
  # Define monitoring executable
  monitor_bin = "umc_full"
elif monitor == "UMC_SWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  if model == 'ATOMIC':
    delay = 25
  if model == 'TIMING':
    delay = 44
  if model == 'FLEX':
    MonCPUClass.clock = '6250MHz'
    delay = 118
  # Define monitoring executable
  monitor_bin = "umc_swdrop"
elif monitor == "UMC_HWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  if model == 'FLEX':
    MonCPUClass.clock = '6750MHz'
  # Load the invalidation file
  invalidation_cpu.invalidation_file = "tables/umc_invalidation.txt"
  # Define monitoring executable
  monitor_bin = "umc_hwdrop"
elif monitor == "UMC_HWFILTER":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  if model == 'FLEX':
    MonCPUClass.clock = '6000MHz'
  # Load the invalidation file
  invalidation_cpu.invalidation_file = "tables/umc_invalidation.txt"
  invalidation_cpu.filter_file_1 = "tables/umc_filter.txt"
  invalidation_cpu.filter_ptr_file = "tables/umc_filter_ptrs.txt"
  # Define monitoring executable
  monitor_bin = "umc_hwfilter"
elif monitor == "UMC_FLEX":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  # Load the invalidation file
  invalidation_cpu.invalidation_file = "tables/umc_invalidation.txt"
  invalidation_cpu.filter_file_1 = "tables/umc_filter.txt"
  invalidation_cpu.filter_ptr_file = "tables/umc_filter_ptrs.txt"

#######################################
# LRC
#######################################
elif monitor == "LRC_FULL":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  if model == 'ATOMIC':
    delay = 17
  if model == 'TIMING':
    delay = 18
  if model == 'FLEX':
    MonCPUClass.clock = '1500MHz'
    delay = 29
  # Define monitoring executable
  monitor_bin = "lrc_full"
elif monitor == "LRC_SWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  if model == 'ATOMIC':
    delay = 22
  if model == 'TIMING':
    delay = 33
  if model == 'FLEX':
    MonCPUClass.clock = '2250MHz'
    delay = 9
  # Define monitoring executable
  monitor_bin = "lrc_swdrop"
elif monitor == "LRC_HWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  if model == 'FLEX':
    MonCPUClass.clock = '3250MHz'
  # Load the invalidation file
  invalidation_cpu.invalidation_file = "tables/lrc_invalidation.txt"
  # Define monitoring executable
  monitor_bin = "lrc_hwdrop"
elif monitor == "LRC_HWFILTER":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  if model == 'FLEX':
    MonCPUClass.clock = '3250MHz'
  # Load the invalidation file
  invalidation_cpu.invalidation_file = "tables/lrc_invalidation.txt"
  invalidation_cpu.filter_file_1 = "tables/lrc_filter.txt"
  invalidation_cpu.filter_ptr_file = "tables/lrc_filter_ptrs.txt"
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
  if model == 'ATOMIC':
    delay = 26
  if model == 'TIMING':
    delay = 61
  if model == 'FLEX':
    MonCPUClass.clock = '6500MHz'
    delay = 124
  # Define monitoring executable
  monitor_bin = "dift_full"
# Drop handled in software
elif monitor == "DIFT_SWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  if model == 'ATOMIC':
    delay = 28
  if model == 'TIMING':
    delay = 63
  if model == 'FLEX':
    MonCPUClass.clock = '7000MHz'
    delay = 133
  # Define monitoring executable
  monitor_bin = "dift_swdrop"
# Drop handled in hardware without filtering
elif monitor == "DIFT_HWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  if model == 'FLEX':
    MonCPUClass.clock = '10500MHz'
  # Load the invalidation file
  invalidation_cpu.invalidation_file = "tables/dift_invalidation.txt"
  # Define monitoring executable
  monitor_bin = "dift_hwdrop"
# Drop handled in hardware with filtering
elif monitor == "DIFT_HWFILTER":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  if model == 'FLEX':
    MonCPUClass.clock = '7500MHz'
  # Load the invalidation file
  invalidation_cpu.invalidation_file = "tables/dift_invalidation.txt"
  # Load the filter tables
  invalidation_cpu.filter_file_1 = "tables/dift_filter1.txt"
  invalidation_cpu.filter_file_2 = "tables/dift_filter2.txt"
  invalidation_cpu.filter_ptr_file = "tables/dift_filter_ptrs.txt"
  # Define monitoring executable
  monitor_bin = "dift_hwfilter"
elif monitor == "DIFT_RF_HWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  if model == 'FLEX':
    MonCPUClass.clock = '10500MHz'
  # Load the invalidation file
  invalidation_cpu.invalidation_file = "tables/dift_rf_invalidation.txt"
  # Define monitoring executable
  monitor_bin = "dift_rf_hwdrop"
# Drop handled in hardware with filtering
elif monitor == "DIFT_RF_HWFILTER":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  if model == 'FLEX':
    MonCPUClass.clock = '7500MHz'
  # Load the invalidation file
  invalidation_cpu.invalidation_file = "tables/dift_rf_invalidation.txt"
  # Load the filter tables
  invalidation_cpu.filter_file_1 = "tables/dift_rf_filter1.txt"
  invalidation_cpu.filter_file_2 = "tables/dift_rf_filter2.txt"
  invalidation_cpu.filter_ptr_file = "tables/dift_rf_filter_ptrs.txt"
  # Define monitoring executable
  monitor_bin = "dift_rf_hwfilter"
elif monitor == "DIFT_FLEX":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
elif monitor == "BC_FLEX":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
elif monitor == "HB_FLEX":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
elif monitor == "NONE_FLEX":
  # Set up monitoring filter
  pass

else:
  raise Exception("Monitor not recognized: %s" % monitor)

