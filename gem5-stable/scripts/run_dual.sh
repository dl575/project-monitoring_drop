if [ $1 ]; then
    if [ $2 ]; then
        gem5.debug --debug-flags=$2 $GEM5/configs/example/dual_core.py -c $1 --cpu-type=atomic
    else
        gem5.debug $GEM5/configs/example/dual_core.py -c $1 --cpu-type=atomic
    fi
    # gem5.debug $GEM5/configs/example/dual_core.py  --cpu-type=atomic
    # gem5.debug --debug-flags=Fifo $GEM5/configs/example/dual_core.py  --cpu-type=atomic
    # gem5.debug --debug-flags=Fifo,SlackTimer,Task $GEM5/configs/example/dual_core.py  --cpu-type=atomic
    # gem5.debug --debug-flags=SlackTimer,Task $GEM5/configs/example/dual_core.py  --cpu-type=atomic
    # gem5.debug --debug-flags=Task $GEM5/configs/example/dual_core.py  --cpu-type=atomic
    # gem5.debug --debug-flags=Fifo,Exec $GEM5/configs/example/dual_core.py  --cpu-type=atomic
else
  echo "usage: run_dual.sh executable.arm [debug_flags (comma seperated)]"
fi
