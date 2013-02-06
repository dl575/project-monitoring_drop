if [ $1 ] && [ $2 ]; then
    # gem5.debug --debug-flags=Task configs/example/slacktimer.py -c $1
    # gem5.debug --debug-flags=SlackTimer,Task configs/example/slacktimer.py -c $1
    echo "
---------------------------------------------------------
Make sure you set the WCET for the subtasks and
tasks high enough so you don't get negative slack.
---------------------------------------------------------
"

    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    gem5.debug --debug-flags=Task $GEM5/configs/example/wcet.py -c $2 --delay=$1 --cpu-type=atomic | tee $3 | $DIR/calculate_wcet.pl
elif [ $1 ]; then
    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    $DIR/calculate_wcet.pl $1
else
    echo "usage: 
  find_wcet.sh drop_delay executable.arm [output.log]
OR
  find_wcet.sh output.log (if you already have the output)"
fi

