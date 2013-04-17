if [ $1 ] && [ $2 ]; then
    # gem5.debug --debug-flags=Task configs/example/slacktimer.py -c $1
    # gem5.debug --debug-flags=SlackTimer,Task configs/example/slacktimer.py -c $1

    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    
    if [[ "$1" =~ ^[0-9]+$ ]]; then
        $DIR/calculate_wcet.pl -d $1 $2
    else
    
    echo "
---------------------------------------------------------
Make sure you set the WCET for the subtasks and
tasks high enough so you don't get negative slack.
---------------------------------------------------------
"

    # gem5.debug --debug-flags=SlackTimer,Fifo,Task $GEM5/configs/example/wcet.py -c $2 --monitor=$1 --cpu-type=atomic | tee $3 | $DIR/calculate_wcet.pl
    gem5.debug --debug-flags=Task $GEM5/configs/example/wcet.py -c $2 --monitor=$1 --cpu-type=timing --caches | tee $3 | $DIR/calculate_wcet.pl
    
    fi
elif [ $1 ]; then
    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
    $DIR/calculate_wcet.pl $1
else
    echo "usage: 
  find_wcet.sh monitor executable.arm [output.log]
OR
  find_wcet.sh [delay] output.log (if you already have an output with debug Task flag)"
fi

