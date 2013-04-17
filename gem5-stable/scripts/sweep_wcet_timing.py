#!/usr/bin/python

"""
Compile for a range of different WCET. Run simulation and gather results on
execution time and number of dropped checks.
"""

import subprocess
import os

program = "timer_monitor"
monitor = "UMC"
# monitor = "LRC"

# WCET per task to try (in percent)
wcets = [i for i in range(50, 100, 10) + range(100, 401, 20)]
# wcets = [i for i in range(20, 100, 20) + range(100, 200, 50)]
# directory where simulation results are stored
log_dir = os.environ["GEM5"] + "/m5out/%s/" % program
compile_dir = os.environ["GEM5"] + "/tests/monitoring"

if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# Clear out old logs
p = subprocess.Popen("rm -v %s/wcet*.log" % log_dir, shell=True)
p.wait()

# For each WCET to try
for wcet in wcets:
  # Compile the main program
  compile_cmd = "arm-linux-gnueabi-gcc -O2 -D%s -DUNIX -DWCET_SCALE=%f \
      %s.c -o %s.arm --static" % (monitor, (float(wcet)/100), program, program)
  print compile_cmd
  p = subprocess.Popen(compile_cmd, cwd=compile_dir, shell=True)
  p.wait()

  # Run simulation
  run_cmd = "gem5.debug $GEM5/configs/example/dual_core.py \
             -c %s/%s.arm --cpu-type=timing --caches" % \
             (compile_dir, program)
  # Output file to store simulation results in
  log_file = log_dir + ("wcet%.4d.log" % wcet)
  output = open(log_file, 'w')
  p = subprocess.Popen(run_cmd, cwd=os.environ["GEM5"], shell=True, stdout=output)
  result = p.wait()
  output.close()
  if (result != 0):
    p = subprocess.Popen("rm -v %s" % log_file, shell=True)
    p.wait()

