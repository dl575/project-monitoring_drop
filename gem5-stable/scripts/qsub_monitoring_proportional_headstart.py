#!/usr/bin/env python

"""
Run monitoring for different FIFO sizes and monitor frequency.
Submit jobs on cluster.
"""

from __future__ import print_function
import subprocess
import os
import sys
import time
import json
import multiprocessing
import itertools
import shutil
import optparse

qsub_script_header = '''#!/usr/local/bin/tcsh

# setup environment
setenv LD_LIBRARY_PATH /ufs/cluster/tchen/local/lib:/ufs/cluster/tchen/local/lib64
setenv PATH /ufs/cluster/tchen/local/bin:$PATH
setenv GEM5 /ufs/cluster/dlo/monitoring_drop2/gem5-stable/

# hack for gobmk
if ( ! -d games ) then
  ln -s /ufs/cluster/tchen/spec2006/benchspec/CPU2006/445.gobmk/data/all/input/games
endif
if ( ! -e teapot.env ) then
  ln -s /ufs/cluster/tchen/parsec/parsec-3.0/ext/splash2/apps/raytrace/run/teapot.env
endif
if ( ! -e teapot.geo ) then
  ln -s /ufs/cluster/tchen/parsec/parsec-3.0/ext/splash2/apps/raytrace/run/teapot.geo
endif
if ( ! -e random.in ) then
  ln -s /ufs/cluster/tchen/parsec/parsec-3.0/ext/splash2/apps/water_nsquared/run/random.in
endif

# start gem5
'''

current_time = time.strftime('%Y%m%d-%H%M%S', time.localtime())
# base_dir = './'
base_dir = '/ufs/cluster/dlo/run/drop_m5out'

class ClusterDispatcher:
  '''A simulation task dispatcher using Sun Grid Engine'''
  def __init__(self, n_jobs=64):
    self.n_jobs = n_jobs
    self.tasks = []

  def add_task(self, task):
    self.tasks.append(task)

  def run(self, run_dir):
    '''Run tasks in task queue'''
    # build task queues
    self.queues = []
    if self.n_jobs == 0:
      self.n_jobs = len(self.tasks)
    for j in range(self.n_jobs):
      self.queues.append([self.tasks[i] for i in range(j, len(self.tasks), self.n_jobs)])
    # build scripts
    j = 1
    for q in self.queues:
      script = '' + qsub_script_header % ()
      # Record start time
      script += "\necho `date +\"%%s\"`' '`date` > queue%03d.start \n" % (j)
      for task in q:
        script += '\n%s\n' % task
      # Touch done flag
      #script += "\ntouch queue%03d.done\n" % (j)
      # Record finish time
      script += "\necho `date +\"%%s\"`' '`date` > queue%03d.done \n" % (j)
      # write script
      with open(os.path.join(run_dir, "queue%03d.csh" % j), 'w') as f:
        f.write(script)
      j += 1

    # submit jobs to SGE


def run(config, n_jobs):
  "Run benchmarks according to configuration"
  # determine Gem5 directory
  if os.environ.has_key('GEM5'):
    gem5_dir = os.environ['GEM5']
  else:
    # assume we are in scripts directory
    gem5_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
  gem5_exe = os.path.join(gem5_dir, 'build', 'ARM', config['gem5'])
  # task list
  tasks = []
  for prod in itertools.product(config['cpu_type'], config['core_frequency'], config['monitor_frequency'], 
                                config['monitor'], config['benchmarks'], config['overhead'], 
                                config['invalidation_cache_size']):
    prod_list = list(prod)

    out_dir = os.path.join(base_dir, current_time, prod[0], 'core_%s' % (prod[1]),
                           'mon_%s' % (prod[2]), prod[3], prod[6], prod[4], 'overhead_%.4f' % (prod[5]))
    if not config['dryrun']:
      os.makedirs(out_dir)
    gem5_args = ' --remote-gdb-port=0 --outdir=%s ' % (out_dir)
    sim_config = config['config']
    config_args = ' --cpu-type=%s --clock=%s --monfreq=%s --monitor=%s' % (prod[0], prod[1], prod[2], prod[3])


    # Scale headstart based on slack and cycles of simulation
    config_args += ' --headstart_slack=%d' % (prod[5]*config['max_insts']/10)
    if config['static_coverage']:
      config_args += ' --static_coverage'
    if config['max_insts']:
      config_args += ' --maxinsts_cpu0=%d' % (config['max_insts'])
    if config['ff_insts']:
      config_args += ' --fastforward_insts=%d' % (config['ff_insts'])
    if config['emulate_filtering']:
      config_args += ' --emulate_filtering'
    if config['cache_enabled']:
      config_args += ' --caches --simulatestalls'
      if config['l2_cache_enabled']:
        config_args += ' --l2cache --l2_size=%s --l2_assoc=%d' % (config['l2_size'], config['l2_assoc'])
      if 'cache_sizes' in config and prod[1] in config['cache_sizes']:
        config_args += ' --l1d_size=%s --l1i_size=%s --l1d_assoc=%d --l1i_assoc=%d' % (config['cache_sizes'][prod[1]], config['cache_sizes'][prod[1]], config['l1_assoc'], config['l1_assoc'])
    if config['invalidation']:
      config_args += ' --invalidation --invalidation_cache_size=%s --overhead=%.4f' % (prod[6], prod[5])
    if config['source_dropping']:
      config_args += ' --source_dropping'
    if prod[3] == 'none' or prod[3] == 'ls':
      suffix = ''
    elif prod[3] == 'multidift' or prod[3] == 'insttype':
      suffix = '-dift'
    else:
      suffix = '-' + prod[3]
    config_args += ' --cmd=%s%s' % (config['benchmarks'][prod[4]]['executable'], suffix) 
    config_args += ' --options=\'%s\'' % (config['benchmarks'][prod[4]]['options'])
    if config['benchmarks'][prod[4]].get('input'):
        config_args += ' --input=\'%s\'' % (config['benchmarks'][prod[4]]['input'])
    if config.get('optimal_dropping'):
      # optimal dropping
      config_args += ' --read_optimal_dropping --optimal_dropping --ipt_tagless --important_policy=always'
      # optimal dropping points directory
      config_args += ' --backtrack_table_dir=/ufs/cluster/tchen/run/drop_m5out/odp/atomic/core_0.5GHz/mon_0.5GHz/%s/1kB/%s/overhead_0.1000' % (prod[3], prod[4])
    run_cmd = gem5_exe + gem5_args + sim_config + config_args
    tasks.append(run_cmd)
  if not config['dryrun']:
    dispatcher = ClusterDispatcher(n_jobs)
    for t in tasks:
      dispatcher.add_task(t)
    dispatcher.run(os.path.join(base_dir, current_time))
  else:
    for t in tasks:
      print(t)

def usage():
  print('Usage:', sys.argv[0], 'config [n_jobs]')

def load_config(config_file):
  with open(config_file) as f:
    return json.load(f)

def main():
  if len(sys.argv) < 2:
    usage()
    exit()
  # load sweep configuration
  config = load_config(sys.argv[1])
  if len(sys.argv) == 3:
    n_jobs = int(sys.argv[2])
  else:
    n_jobs = config['jobs']
  # print(config)
  run(config, n_jobs)
  # Copy config file for future reference
  shutil.copy(sys.argv[1], os.path.join(base_dir, current_time))


if __name__ == '__main__':
  main()
