setenv  LD_LIBRARY_PATH /ufs/cluster/tchen/local/lib:/ufs/cluster/tchen/local/lib64

set path = (/ufs/cluster/tchen/local/bin /usr/local/bin /usr/X11R6/bin /sbin /usr/bin /usr/sbin /bin /usr/local/sbin /usr/local/cad/bin /opt/cadtools/cadence/ius/tools.lnx86/bin $path)
# TSG tools on Cluster
# binutils
set path = (/ufs/cluster/tsg/tools/binutils-2.23.1/bin $path)

# Sun Grid Engine on cluster
if (`hostname -s` =~ "cluster") then
  source /usr/local/sge/default/common/settings.csh
endif
