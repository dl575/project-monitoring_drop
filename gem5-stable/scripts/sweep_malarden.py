#!/usr/bin/python

"""
For each of the generated Malarden task sets, compile for a range of different
WCET. Run simulation and save stdout of these simulations.
"""

import glob
import os
import re
import subprocess

monitor = os.environ["MONITOR"]
model = os.environ["MODEL"]

# WCET per task to try (in cycles)
wcets = [i for i in range(97,100,1) + range(100,501,50)]
# directory where simulation results are stored
log_dir = os.environ["GEM5"] + "/m5out/malarden/"
# Directory where generated sources are
compile_dir = os.environ["GEM5"] + "/tests/malarden_monitor/generated/"

if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# Clear out old logs
p = subprocess.Popen("rm -v %s/wcet*.log" % log_dir, shell=True)
p.wait()

# For each generated Malarden task set
for filename in glob.glob(os.path.join(compile_dir, "malarden*.c")):
  # Parse the filename to find what benchmark combination this is
  find_benchmarks = re.search("malarden_([\w_]*)\.c", filename)
  benchmarks = find_benchmarks.group(1)
  # For each WCET to try
  for wcet in wcets:
    # Compile the main program 
    # Move it to ../multi_malarden.arm to work with config script
    compile_cmd = "arm-linux-gnueabi-gcc -D%s -D%s -DUNIX -O2 -DWCET_SCALE=%f \
        %s malarden_%s.c -o malarden_%s.arm --static;" % ( monitor, model, \
        (float(wcet)/100), ' '.join([('../include/%s.c' % elem) for elem in benchmarks.split('_')]), \
         benchmarks, benchmarks)
    print compile_cmd
    p = subprocess.Popen(compile_cmd, cwd=compile_dir, shell=True)
    p.wait()

    # Run simulation
    run_cmd = "gem5.debug $GEM5/configs/example/dual_core.py \
               -c %s/malarden_%s.arm --cpu-type=atomic" % \
               (compile_dir, benchmarks)
    # Output file to store simulation results in
    output = open(log_dir + "wcet_%s_%d.log" % (benchmarks, wcet), 'w')
    p = subprocess.Popen(run_cmd, cwd=os.environ["GEM5"], shell=True, stdout=output)
    result = p.wait()
    output.close()
    if (result != 0):
        p = subprocess.Popen("rm -v %s/wcet_%s_%d.log" % (log_dir, benchmarks, wcet), shell=True)
        p.wait()

