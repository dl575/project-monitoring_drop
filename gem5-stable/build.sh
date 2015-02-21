source setup_cluster.csh
set path = (/ufs/cluster/tsg/tools/scons-2.2.0/bin/ $path)
echo $path
scons build/ARM/gem5.fast
