monitor = os.environ["MONITOR"]
model = os.environ["MODEL"]

#######################################
# UMC
#######################################
if monitor == "UMC_FULL":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  if model == 'ATOMIC':
    delay = 22
  if model == 'TIMING':
    delay = 301
  # Define monitoring executable
  monitor_bin = "umc_full"
elif monitor == "UMC_SWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  if model == 'ATOMIC':
    delay = 30
  if model == 'TIMING':
    delay = 365
  # Define monitoring executable
  monitor_bin = "umc_swdrop"
elif monitor == "UMC_HWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  # Load the invalidation file
  MonCPUClass.invalidation_file = "tables/umc_invalidation.txt"
  # Define monitoring executable
  monitor_bin = "umc_hwdrop"
elif monitor == "UMC_HWFILTER":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
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
  if model == 'ATOMIC':
    delay = 15
  if model == 'TIMING':
    delay = 170
  # Define monitoring executable
  monitor_bin = "lrc_full"
elif monitor == "LRC_SWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  if model == 'ATOMIC':
    delay = 19
  if model == 'TIMING':
    delay = 168
  # Define monitoring executable
  monitor_bin = "lrc_swdrop"
elif monitor == "LRC_HWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  # Load the invalidation file
  MonCPUClass.invalidation_file = "tables/lrc_invalidation.txt"
  # Define monitoring executable
  monitor_bin = "lrc_hwdrop"
elif monitor == "LRC_HWFILTER":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
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
  # MonCPUClass.delay = 40
  # Define monitoring executable
  monitor_bin = "dift_full"
# Drop handled in software
elif monitor == "DIFT_SWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  # MonCPUClass.delay = 40
  # Define monitoring executable
  monitor_bin = "dift_swdrop"
# Drop handled in hardware without filtering
elif monitor == "DIFT_HWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
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

