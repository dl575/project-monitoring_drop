#!/usr/bin/python

"""
Compile for a range of different WCET. Run simulation and gather results on
execution time and number of dropped checks.
"""

import subprocess
import os

# WCET per task to try (in cycles)
wcets = [i for i in range(100, 601, 20)]
# directory where simulation results are stored
log_dir = os.environ["GEM5"] + "/m5out/"
compile_dir = os.environ["GEM5"] + "/tests/monitoring"

# Clear out old logs
p = subprocess.Popen("rm -v %s/wcet*.log" % log_dir, shell=True)
p.wait()

# For each WCET to try
for wcet in wcets:
  # Compile the main program
  compile_cmd = "arm-linux-gnueabi-gcc -O2 -DUMC -DUNIX -DWCET_SCALE=%f \
      timer_monitor.c -o timer_monitor.arm --static" % (float(wcet)/100)
  print compile_cmd
  p = subprocess.Popen(compile_cmd, cwd=compile_dir, shell=True)
  p.wait()

  # Run simulation
  run_cmd = "gem5.debug $GEM5/configs/example/dual_core.py \
             -c %s/timer_monitor.arm --cpu-type=atomic" % \
             (compile_dir)
  # Output file to store simulation results in
  output = open(log_dir + ("wcet%.4d.log" % wcet), 'w')
  p = subprocess.Popen(run_cmd, cwd=os.environ["GEM5"], shell=True, stdout=output)
  p.wait()
  output.close()

