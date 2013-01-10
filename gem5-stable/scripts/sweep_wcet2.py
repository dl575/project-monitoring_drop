#!/usr/bin/python

"""
Compile for a range of different WCET. Run simulation and gather results on
execution time and number of dropped checks.
"""

import subprocess
import os

# WCET per task to try (in cycles)
nsteps = 10
wmax = 4000
wcets = [i for i in range(0, wmax, int(wmax/nsteps))]
# directory where simulation results are stored
log_dir = os.environ["GEM5"] + "/m5out/"
compile_dir = os.environ["GEM5"] + "/tests/malarden_monitor"

# For each WCET to try
for wcet in wcets:
  # Compile the main program
  compile_cmd = "arm-linux-gnueabi-gcc -DUNIX -DWCET_CYCLES1=%d -DWCET_CYCLES2=%d\
      multi_fac.c -o multi_fac.arm --static" % (wcet, wcet)
  p = subprocess.Popen(compile_cmd, cwd=compile_dir, shell=True)
  p.wait()

  # Run simulation
  run_cmd = "./scripts/run_dual.sh"
  # Output file to store simulation results in
  output = open(log_dir + ("wcet%.5d.log" % wcet), 'w')
  p = subprocess.Popen(run_cmd, cwd=os.environ["GEM5"], shell=True, stdout=output)
  p.wait()
  output.close()

