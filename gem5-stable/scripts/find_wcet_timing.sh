if [ $1 ] ; then
   
    echo "
---------------------------------------------------------
Make sure you set the WCET for the subtasks and
tasks high enough so you don't get negative slack.
---------------------------------------------------------
"

    DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

    # gem5.debug --debug-flags=SlackTimer,Fifo,Task $GEM5/configs/example/wcet.py -c $1 --cpu-type=timing --caches | tee $2 | $DIR/calculate_wcet.pl
    gem5.debug --debug-flags=Task $GEM5/configs/example/wcet.py -c $1 --cpu-type=timing --caches | tee $2 | $DIR/calculate_wcet.pl

else
    echo "usage: 
  find_wcet.sh executable.arm [output.log]"
fi


