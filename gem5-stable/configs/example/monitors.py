monitor = os.environ["MONITOR"]
# Set up monitoring filter
if monitor == "UMC":
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  # Load the invalidation file
  # MonCPUClass.invalidation_file = "tables/umc_invalidation.txt"
  # MonCPUClass.filter_file_1 = "tables/umc_filter.txt"
  # MonCPUClass.filter_ptr_file = "tables/umc_filter_ptrs.txt"
  # MonCPUClass.delay = 128
  # Define monitoring executable
  monitor_bin = "umc_drop"
  #monitor_bin = "umc"
elif monitor == "LRC":
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  # Load the invalidation file
  MonCPUClass.invalidation_file = "tables/lrc_invalidation.txt"
  MonCPUClass.filter_file_1 = "tables/lrc_filter.txt"
  MonCPUClass.filter_ptr_file = "tables/lrc_filter_ptrs.txt"
  # MonCPUClass.delay = 40
  # Define monitoring executable
  monitor_bin = "lrc_drop"
  #monitor_bin = "lrc"
else:
  raise Exception("Monitor not recognized: %s" % monitor)

