export PATH=$PATH:`pwd`/build/ARM/
export GEM5=`pwd`/
if [ $1 ]; then
    export MONITOR=$1
else
    export MONITOR='UMC'
fi