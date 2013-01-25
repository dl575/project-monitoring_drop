if [ $1 ]; then
    # gem5.debug --debug-flags=Task configs/example/slacktimer.py -c $1
    # gem5.debug --debug-flags=SlackTimer,Task configs/example/slacktimer.py -c $1
    echo "
---------------------------------------------------------
Make sure you set the WCET for the subtasks and
tasks high enough so you don't get negative slack.
Also make sure you set the WCET for the monitor 
high enough so it drops everything.
---------------------------------------------------------
"

    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    gem5.debug --debug-flags=Task $GEM5/configs/example/dual_core.py -c $1 --cpu-type=atomic | $DIR/calculate_wcet.pl
else
    echo 'usage: find_wcet.sh executable.arm'
fi

