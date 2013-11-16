#!/bin/bash
cd /ufs/cluster/mismail/monitoring_drop/gem5-stable
export PATH=$PATH:`pwd`/build/ARM/
export GEM5=`pwd`/
# export MONITOR='UMC_HWDROP'
# export MONITOR='UMC_HWFILTER'
# export MONITOR='LRC_HWDROP'
# export MONITOR='LRC_HWFILTER'
# export MONITOR='DIFT_HWDROP'
# export MONITOR='DIFT_HWFILTER'
# export MODEL='TIMING'
export MODEL='FLEX'

for MONITOR in 'UMC_HWDROP' 'UMC_HWFILTER' 'LRC_HWDROP' 'LRC_HWFILTER' 'DIFT_HWDROP' 'DIFT_HWFILTER'
do
    perl -np -e "s/<MONITOR>/$MONITOR/; s/<MODEL>/$MODEL/;" $GEM5/scripts/run_node.sh.tmpl > $GEM5/scripts/run_node.sh

    # unzip -o tests/monitoring/monitoring.$MODEL.zip -d tests/monitoring
    tar -xvjf tests/monitoring/monitoring.$MODEL.bz2 -C tests/monitoring
    rm -rf tests/malarden_monitor/generated/malarden*.arm
    # unzip -o tests/malarden_monitor/generated/generated.$MONITOR.$MODEL.zip -d tests/malarden_monitor/generated
    tar -xvjf tests/malarden_monitor/generated/generated.$MONITOR.$MODEL.bz2 -C tests/malarden_monitor/generated
    rm -rf m5out/malarden/$MONITOR/$MODEL
    mkdir -p m5out/malarden/$MONITOR/$MODEL

    for file in tests/malarden_monitor/generated/malarden*.arm
    do
        echo qsub -b y "$GEM5/scripts/run_node.sh $GEM5/$file"
        qsub -b y "$GEM5/scripts/run_node.sh $GEM5/$file"
        sleep 0.2
    done

    scripts/cluster_wait.pl

done
