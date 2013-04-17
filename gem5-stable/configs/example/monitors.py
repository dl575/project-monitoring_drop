# Set up monitoring filter
if monitor == "umc" or monitor == "umc_drop":
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  # Load the invalidation file
  MonCPUClass.invalidation_file = "tables/umc_invalidation.txt"
  MonCPUClass.filter_file_1 = "tables/umc_filter.txt"
  MonCPUClass.filter_ptr_file = "tables/umc_filter_ptrs.txt"
elif monitor == "lrc" or monitor == "lrc_drop":
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
else:
  raise Exception("No monitoring filter defined for %s" % monitor)

