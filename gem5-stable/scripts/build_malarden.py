#!/usr/bin/python

"""
For each of the generated Malarden task sets, compile for a range of different
WCET. Run simulation and save stdout of these simulations.
"""

import glob
import os
import re
import subprocess

models = ['TIMING', 'FLEXHW'] #, 'ATOMIC']
monitors = {'ATOMIC': ['UMC_HWDROP', 'UMC_HWFILTER', 'LRC_HWDROP', 'LRC_HWFILTER', 'DIFT_HWDROP', 'DIFT_HWFILTER', 'DIFT_RF_HWDROP', 'DIFT_RF_HWFILTER'], \
            'TIMING': ['UMC_HWDROP', 'UMC_HWFILTER', 'LRC_HWDROP', 'LRC_HWFILTER', 'DIFT_HWDROP', 'DIFT_HWFILTER', 'DIFT_RF_HWDROP', 'DIFT_RF_HWFILTER'], \
            'FLEXHW': ['UMC_HWFILTER', 'LRC_HWFILTER', 'DIFT_HWFILTER', 'DIFT_RF_HWFILTER'], }

# WCET per task to try (in cycles)
wcets = {'ATOMIC': {'UMC_HWDROP': [100], 'UMC_HWFILTER': range(100,701,50), 'LRC_HWDROP': [100], 'LRC_HWFILTER': range(100,701,50), 'DIFT_HWDROP': [100], 'DIFT_HWFILTER': range(100,701,50), 'DIFT_RF_HWDROP': [100], 'DIFT_RF_HWFILTER': range(100,701,50)}, \
         'TIMING': {'UMC_HWDROP': [100], 'UMC_HWFILTER': range(100,701,50), 'LRC_HWDROP': [100], 'LRC_HWFILTER': range(100,701,50), 'DIFT_HWDROP': [100], 'DIFT_HWFILTER': range(100,701,50), 'DIFT_RF_HWDROP': [100], 'DIFT_RF_HWFILTER': range(100,701,50)}, \
         'FLEXHW': {'UMC_HWDROP': [100], 'UMC_HWFILTER': range(100,111,1), 'LRC_HWDROP': [100], 'LRC_HWFILTER': range(100,111,1), 'DIFT_HWDROP': [100], 'DIFT_HWFILTER': range(100,111,1), 'DIFT_RF_HWDROP': [100], 'DIFT_RF_HWFILTER': range(100,111,1)}, \
        }
# Directory where generated sources are
compile_dir = os.environ["GEM5"] + "/tests/malarden_monitor/generated/"

for model in models:
  for monitor in monitors[model]:
    # Clear out old executables
    p = subprocess.Popen("rm -v %s/malarden*.arm" % compile_dir, shell=True)
    p.wait()

    # For each generated Malarden task set
    for filename in glob.glob(os.path.join(compile_dir, "malarden*.c")):
      # Parse the filename to find what benchmark combination this is
      find_benchmarks = re.search("malarden_([\w_]*)\.c", filename)
      benchmarks = find_benchmarks.group(1)
      # For each WCET to try
      for wcet in wcets[model][monitor]:
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

