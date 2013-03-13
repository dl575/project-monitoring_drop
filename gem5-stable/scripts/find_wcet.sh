if [ $1 ] && [ $2 ] && [ $3 ]; then
    # gem5.debug --debug-flags=Task configs/example/slacktimer.py -c $1
    # gem5.debug --debug-flags=SlackTimer,Task configs/example/slacktimer.py -c $1
    echo "
---------------------------------------------------------
Make sure you set the WCET for the subtasks and
tasks high enough so you don't get negative slack.
---------------------------------------------------------
"

    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    # gem5.debug --debug-flags=SlackTimer,Fifo,Task $GEM5/configs/example/wcet.py -c $3 --monitor=$1 --cpu-type=atomic | tee $4 | $DIR/calculate_wcet.pl -d $2
    gem5.debug --debug-flags=Task $GEM5/configs/example/wcet.py -c $3 --monitor=$1 --cpu-type=atomic | tee $4 | $DIR/calculate_wcet.pl -d $2
elif [ $1 ] && [ $2 ]; then
    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    $DIR/calculate_wcet.pl -d $1 $2
else
    echo "usage: 
  find_wcet.sh monitor drop_delay executable.arm [output.log]
OR
  find_wcet.sh drop_delay output.log (if you already have an output with debug Task flag)"
fi

