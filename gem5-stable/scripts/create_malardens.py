#!/usr/bin/python

"""
Generate C file that runs a combination of Malarden benchmarks.
"""

import os
import subprocess

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
funcs = ["insertsort", "crc"]
# Number of different benchmark functions
nfuncs = len(funcs)

# Clean out any old generated benchmarks
p = subprocess.Popen("rm -v %s/*" % gen_dir, shell=True)
p.wait()

# For each pair of different functions
for i in range(nfuncs):
  for j in range(i+1, nfuncs):
    # Copy template to create new file
    output_filename = gen_dir + "malarden_%s_%s.c" % (funcs[i], funcs[j])
    # p = subprocess.Popen("cp -v %s %s" % (input_filename, output_filename), shell=True)
    # p.wait()
    # Replace <INSERT_FUNCTIONS> with the selected functions
    p = subprocess.Popen("perl -p -w -e 's/<INSERT FUNCTIONS>/%s();\n    %s();/g;' %s > %s" % \
        (funcs[i], funcs[j], input_filename, output_filename), shell=True)
    p.wait()

