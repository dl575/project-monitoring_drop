# Set up monitoring filter
if monitor == "umc" or monitor == "umc_drop":
  MainCPUClass.monitoring_filter_load = True
  MainCPUClass.monitoring_filter_store = True
  delay = 27
elif monitor == "lrc" or monitor == "lrc_drop":
  MainCPUClass.monitoring_filter_call = True
  MainCPUClass.monitoring_filter_ret = True
  delay = 24
else:
  raise Exception("No monitoring filter defined for %s" % monitor)

