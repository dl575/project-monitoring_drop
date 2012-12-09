#gem5.debug --debug-flags=Exec,Fifo --trace-file=out configs/example/test_timer.py  --cpu-type=atomic
#gem5.debug --debug-flags=SlackTimer configs/example/test_timer.py  --cpu-type=atomic
gem5.debug $GEM5/configs/example/test_timer.py  --cpu-type=atomic
