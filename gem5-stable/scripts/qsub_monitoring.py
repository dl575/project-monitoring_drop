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

qsub_script_header = '''#!/usr/local/bin/tcsh

# setup environment
setenv LD_LIBRARY_PATH /ufs/cluster/tchen/local/lib:/ufs/cluster/tchen/local/lib64
setenv PATH /ufs/cluster/tchen/local/bin:$PATH
setenv GEM5 /ufs/cluster/dlo/monitoring_drop/gem5-stable/

# hack for gobmk
if ( ! -d games ) then
  ln -s /ufs/cluster/tchen/spec2006/benchspec/CPU2006/445.gobmk/data/all/input/games
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
      for task in q:
        script += '\n%s\n' % task
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
    if config['max_insts']:
      config_args += ' --maxinsts=%d' % (config['max_insts'])
    if config.get('ff_insts'):
      config_args += ' --fastforward_insts=%d' % (config['ff_insts'])
    if config['cache_enabled']:
      config_args += ' --caches --simulatestalls'
      if config['l2_cache_enabled']:
        config_args += ' --l2cache --l2_size=%s' % (config['l2_size'])
      if 'cache_sizes' in config and prod[1] in config['cache_sizes']:
        config_args += ' --l1d_size=%s --l1i_size=%s' % (config['cache_sizes'][prod[1]], config['cache_sizes'][prod[1]])
    if config['invalidation']:
      config_args += ' --invalidation --invalidation_cache_size=%s --overhead=%.4f' % (prod[6], prod[5])
    config_args += ' --cmd=%s%s' % (config['benchmarks'][prod[4]]['executable'], '' if prod[3] == 'none' else '-'+prod[3])
    config_args += ' --options=\'%s\'' % (config['benchmarks'][prod[4]]['options'])
    if config.get('headstart_slack'):
      config_args += ' --headstart_slack=%d' % (config['headstart_slack'])
    if config.get('backtrack'):
      config_args += ' --backtrack'
      if config.get('ipt_size'):
        config_args += ' --ipt_size=%d' % config['ipt_size']
      if config.get('mpt_size'):
        config_args += ' --mpt_size=%d' % config['mpt_size']
      if config.get('important_policy'):
        config_args += ' --important_policy=%s' % config['important_policy']
      if config.get('important_slack'):
        config_args += ' --important_slack=%d' % config['important_slack']
      if config.get('important_percent'):
        config_args += ' --important_percent=%f' % config['important_percent']
      run_cmd = [gem5_exe + gem5_args + sim_config + config_args + ' --backtrack_write_table' + ' --backtrack_table_dir=%s' % out_dir]
      # compress table and stats
      run_cmd.append("pushd %s\ntar czvf tables.0.tar.gz *.table\ntar czvf stats.0.tar.gz stats.txt\npopd" % out_dir)
      for i in xrange(config.get('backtrack_iterations', 2)):
        run_cmd.append(gem5_exe + gem5_args + sim_config + config_args + ' --backtrack_read_table --backtrack_write_table' + ' --backtrack_table_dir=%s' % out_dir)
        # compress tables and stats
        run_cmd.append("pushd %s\ntar czvf tables.%d.tar.gz *.table\ntar czvf stats.%d.tar.gz stats.txt\npopd" % (out_dir, i+1, i+1))
      # remove tables to free up space
      run_cmd.append("rm -f %s/*.table" % out_dir)
      tasks.append('\n\n'.join(run_cmd))
    else:
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

if __name__ == '__main__':
  main()
