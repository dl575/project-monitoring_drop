#!/usr/bin/python

"""
Generate C file that runs a combination of Malarden benchmarks.
"""

import os
import subprocess

monitor = os.environ["MONITOR"]
model = os.environ["MODEL"]

# WCET per task to try (in cycles)
wcets = [i for i in range(97,100,1) + range(100,501,50)]
# C source for generating benchmarks
src_dir = os.environ["GEM5"] + "/tests/malarden_monitor/"
# Directory to place generated benchmarks
gen_dir = src_dir + "generated/"
# Create generated directory if it doesn't exist
if not os.path.isdir(gen_dir):
  p = subprocess.Popen("mkdir %s" % gen_dir, shell=True)
  p.wait()
# Template filename
input_filename = src_dir + "malarden_template.c"

# Malarden functions
funcs = ['insertsort', 'crc', 'edn', 'compress', 'fir', 'jfdc', 'nsichneu', 'statemate']
# Number of different benchmark functions
nfuncs = len(funcs)

# Clean out any old generated benchmarks
p = subprocess.Popen("rm -v %s/*" % gen_dir, shell=True)
p.wait()

# For each pair of different functions
for i in range(nfuncs):
  # Copy template to create new file
  source_filename = gen_dir + "malarden_%s.c" % (funcs[i])
  # p = subprocess.Popen("cp -v %s %s" % (input_filename, output_filename), shell=True)
  # p.wait()
  # Replace <INSERT_FUNCTIONS> with the selected functions
  p = subprocess.Popen("perl -p -w -e 's/<INSERT FUNCTIONS>/%s();/g;' %s > %s" % \
      (funcs[i], input_filename, source_filename), shell=True)
  p.wait()
  # For each WCET to try
  for wcet in wcets:
    # Compile the main program 
    # Move it to ../multi_malarden.arm to work with config script
    compile_cmd = "arm-linux-gnueabi-gcc -D%s -D%s -DUNIX -O2 -DWCET_SCALE=%f \
        ../include/%s.c malarden_%s.c -o malarden_%s_%d.arm --static;" % ( monitor, model, \
        (float(wcet)/100), funcs[i], funcs[i], funcs[i], wcet)
    print compile_cmd
    p = subprocess.Popen(compile_cmd, cwd=gen_dir, shell=True)
    p.wait()

