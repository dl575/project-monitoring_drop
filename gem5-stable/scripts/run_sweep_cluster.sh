#!/bin/bash
cd /ufs/cluster/mismail/monitoring_drop/gem5-stable
export PATH=$PATH:`pwd`/build/ARM/
export GEM5=`pwd`/
export MONITOR='UMC_HWFILTER'
export MODEL='ATOMIC'
perl -np -e "s/<MONITOR>/$MONITOR/; s/<MODEL>/$MODEL/;" $GEM5/scripts/run_node.sh.tmpl > $GEM5/scripts/run_node.sh

unzip -o tests/monitoring/monitoring.$MODEL.zip -d tests/monitoring
rm -rf tests/malarden_monitor/generated/malarden*.arm
unzip -o tests/malarden_monitor/generated/generated.$MONITOR.$MODEL.zip -d tests/malarden_monitor/generated
rm -rf m5out/malarden/$MONITOR/$MODEL
mkdir -p m5out/malarden/$MONITOR/$MODEL
for file in tests/malarden_monitor/generated/malarden*.arm
do
    echo qsub -b y "$GEM5/scripts/run_node.sh $GEM5/$file"
    qsub -b y "$GEM5/scripts/run_node.sh $GEM5/$file"
    sleep 0.5
done