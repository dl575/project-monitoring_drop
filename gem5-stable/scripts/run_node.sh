#!/bin/bash
cd /ufs/cluster/mismail/monitoring_drop/gem5-stable
export LD_LIBRARY_PATH=/ufs/cluster/tchen/local/lib:/ufs/cluster/tchen/local/lib64
export PATH=/ufs/cluster/tchen/local/bin:$PATH
export PATH=/ufs/cluster/tsg/tools/binutils-2.23.1/bin:$PATH
export PATH=`pwd`/build/ARM/:$PATH
export GEM5=`pwd`/
export MONITOR=UMC_HWDROP
export MODEL=TIMING
if [ $1 ]; then
    if [ $MODEL = ATOMIC ]; then
        gem5.debug $GEM5/configs/example/dual_core.py -c $1 --cpu-type=atomic > $GEM5/m5out/malarden/$MONITOR/$MODEL/$(basename ${1%arm})log
    elif [ $MODEL = TIMING ]; then
        gem5.debug $GEM5/configs/example/dual_core.py -c $1 --cpu-type=timing --caches > $GEM5/m5out/malarden/$MONITOR/$MODEL/$(basename ${1%arm})log
    fi
fi
