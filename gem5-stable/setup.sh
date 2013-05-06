if [[ ! ":$PATH:" =~ (^|:)"`pwd`/build/ARM/"(:|$) ]]; then
    export PATH=$PATH:`pwd`/build/ARM/
fi
export GEM5=`pwd`/
if [ $1 ]; then
    export MONITOR=$1
else
    export MONITOR='UMC_HWFILTER'
fi
if [ $2 ]; then
    export MODEL=$2
else
    export MODEL='ATOMIC'
fi
