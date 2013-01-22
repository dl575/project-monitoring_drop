if [ $1 ]; then
    gem5.debug --debug-flags=Task configs/example/slacktimer.py -c $1
    # gem5.debug --debug-flags=SlackTimer,Task configs/example/slacktimer.py -c $1
else
    echo 'usage: find_wcet.sh executable.arm'
fi

