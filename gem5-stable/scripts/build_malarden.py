#!/usr/bin/python

"""
For each of the generated Malarden task sets, compile for a range of different
WCET. Run simulation and save stdout of these simulations.
"""

import glob
import os
import re
import subprocess

models = ['FLEX', 'TIMING'] #, 'ATOMIC']
monitors = ['UMC_HWDROP', 'UMC_HWFILTER', 'LRC_HWDROP', 'LRC_HWFILTER', 'DIFT_HWDROP', 'DIFT_HWFILTER']

# WCET per task to try (in cycles)
wcets = [range(100,301,25), range(100,301,25), range(100,301,25), range(100,301,25), range(100,301,25), range(100,301,25)]
# Directory where generated sources are
compile_dir = os.environ["GEM5"] + "/tests/malarden_monitor/generated/"

for i, monitor in enumerate(monitors):
  for model in models:
    # Clear out old executables
    p = subprocess.Popen("rm -v %s/malarden*.arm" % compile_dir, shell=True)
    p.wait()

    # For each generated Malarden task set
    for filename in glob.glob(os.path.join(compile_dir, "malarden*.c")):
      # Parse the filename to find what benchmark combination this is
      find_benchmarks = re.search("malarden_([\w_]*)\.c", filename)
      benchmarks = find_benchmarks.group(1)
      # For each WCET to try
      for wcet in wcets[i]:
        # Compile the main program 
        compile_cmd = "arm-linux-gnueabi-gcc -D%s -D%s -DUNIX -O2 -DWCET_SCALE=%f \
            %s malarden_%s.c -o malarden_%s_%d.arm --static;" % ( monitor, model, \
            (float(wcet)/100), ' '.join([('../include/%s.c' % elem) for elem in benchmarks.split('_')]), \
             benchmarks, benchmarks, wcet)
        print compile_cmd
        p = subprocess.Popen(compile_cmd, cwd=compile_dir, shell=True)
        p.wait()
    
    # Zip executables
    #p = subprocess.Popen("zip generated.%s.%s.zip malarden*.arm" % (monitor, model), cwd=compile_dir, shell=True)
    p = subprocess.Popen("tar -cjvf generated.%s.%s.bz2 malarden*.arm" % (monitor, model), cwd=compile_dir, shell=True)
    p.wait()

