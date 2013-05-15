if [ $1 ]; then
    if [ $2 ]; then
        gem5.debug --debug-flags=$2 $GEM5/configs/example/dual_core.py -c $1 --cpu-type=timing --caches
    else
        gem5.debug $GEM5/configs/example/dual_core.py -c $1 --cpu-type=timing --caches
    fi
else
  echo "usage: run_dual.sh executable.arm [debug_flags (comma seperated)]"
fi
