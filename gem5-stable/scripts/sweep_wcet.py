#!/usr/bin/python

"""
Compile for a range of different WCET. Run simulation and gather results on
execution time and number of dropped checks.
"""

import subprocess
import os

# WCET per task to try (in cycles)
wcets = [i for i in range(0, 300, 20)]
# directory where simulation results are stored
log_dir = os.environ["GEM5"] + "/m5out/"
compile_dir = os.environ["GEM5"] + "/tests/monitoring"

# For each WCET to try
for wcet in wcets:
  # Compile the main program
  compile_cmd = "arm-linux-gnueabi-gcc -DUNIX -DWCET_CYCLES=%d \
      timer_monitor.c -o timer_monitor.arm --static" % wcet
  p = subprocess.Popen(compile_cmd, cwd=compile_dir, shell=True)
  p.wait()

  # Run simulation
  run_cmd = "./scripts/run_dual.sh"
  # Output file to store simulation results in
  output = open(log_dir + ("wcet%.4d.log" % wcet), 'w')
  p = subprocess.Popen(run_cmd, cwd=os.environ["GEM5"], shell=True, stdout=output)
  p.wait()
  output.close()

