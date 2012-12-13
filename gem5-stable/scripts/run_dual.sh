#gem5.debug configs/example/dual_core.py --cpu-type=atomic
#gem5.debug --debug-flags=Fifo --trace-file=out configs/example/dual_core.py  --cpu-type=atomic
#gem5.debug --debug-flags=Exec --trace-file=out configs/example/dual_core.py  --cpu-type=atomic
gem5.debug --debug-flags=SlackTimer,FifoStall $GEM5/configs/example/dual_core.py  --cpu-type=atomic
