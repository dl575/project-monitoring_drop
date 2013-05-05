monitor = os.environ["MONITOR"]

#######################################
# UMC
#######################################
if monitor == "UMC":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  # Load the invalidation file
  MonCPUClass.invalidation_file = "tables/umc_invalidation.txt"
  MonCPUClass.filter_file_1 = "tables/umc_filter.txt"
  MonCPUClass.filter_ptr_file = "tables/umc_filter_ptrs.txt"
  # Define monitoring executable
  monitor_bin = "umc_drop"
  #monitor_bin = "umc"
elif monitor == "UMC_FULL":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  # Define monitoring executable
  monitor_bin = "umc"

#######################################
# LRC
#######################################
elif monitor == "LRC":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  # Define monitoring executable
  monitor_bin = "lrc_drop"
  #monitor_bin = "lrc"

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
  # Define monitoring executable
  monitor_bin = "dift"
# Drop handled in software
elif monitor == "DIFT_SWDROP":
  # Set up monitoring filter
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  MainCPUClass.monitoring_filter_intalu = True
  MainCPUClass.monitoring_filter_indctrl = True
  # Define monitoring executable
  monitor_bin = "dift_swdrop"
else:
  raise Exception("Monitor not recognized: %s" % monitor)

