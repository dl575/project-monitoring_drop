if [ $1 ]; then
    if [ $2 ]; then
        gem5.debug --debug-flags=$2 $GEM5/configs/example/flex_core.py -c $1 --cpu-type=atomic --caches --simulatestalls
    else
        gem5.debug $GEM5/configs/example/flex_core.py -c $1 --cpu-type=atomic --caches --simulatestalls
    fi
else
  echo "usage: run_dual.sh executable.arm [debug_flags (comma seperated)]"
fi
