#!/bin/bash
cd /ufs/cluster/mismail/monitoring_drop/gem5-stable
export PATH=$PATH:`pwd`/build/ARM/
export GEM5=`pwd`/
export MONITOR='UMC_HWFILTER'
export MODEL='ATOMIC'
NUM_CPU=8

perl -np -e "s/<MONITOR>/$MONITOR/; s/<MODEL>/$MODEL/;" $GEM5/scripts/run_node.sh.tmpl > $GEM5/scripts/run_node.sh

# unzip -o tests/monitoring/monitoring.$MODEL.zip -d tests/monitoring
tar -xvjf tests/monitoring/monitoring.$MODEL.bz2 -C tests/monitoring
rm -rf tests/malarden_monitor/generated/malarden*.arm
# unzip -o tests/malarden_monitor/generated/generated.$MONITOR.$MODEL.zip -d tests/malarden_monitor/generated
tar -xvjf tests/malarden_monitor/generated/generated.$MONITOR.$MODEL.bz2 -C tests/malarden_monitor/generated
rm -rf m5out/malarden/$MONITOR/$MODEL
mkdir -p m5out/malarden/$MONITOR/$MODEL

COUNTER=1
for file in tests/malarden_monitor/generated/malarden*.arm
do
    echo $GEM5/scripts/run_node.sh $GEM5/$file
    if [ $COUNTER -eq $NUM_CPU ]; then
        let COUNTER=1
        $GEM5/scripts/run_node.sh $GEM5/$file
    else
        let COUNTER=COUNTER+1
        $GEM5/scripts/run_node.sh $GEM5/$file &
    fi
    
done