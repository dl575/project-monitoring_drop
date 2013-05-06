if [[ ! ":$PATH:" =~ (^|:)"`pwd`/build/ARM/"(:|$) ]]; then
    export PATH=$PATH:`pwd`/build/ARM/
fi
export GEM5=`pwd`/
if [ $1 ]; then
    export MONITOR=$1
else
    export MONITOR='UMC'
fi
