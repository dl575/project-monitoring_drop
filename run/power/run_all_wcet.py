#!/usr/bin/python

import glob
import sys
import os
from parse_lib import *

base_dir = sys.argv[1]

# Clear output file
out_filename = "run_all_results.txt"
out_file = open(out_filename, 'w')
out_file.close()

# Find all directories with stats.txt files and using 0.1 overhead
"""
roots = []
for root,dirs,files in os.walk(base_dir):
  for f in files:
    if f == "stats.txt":
      roots.append(root)
"""
include = ["100", "HWFILTER"]
stf = get_txt_files(base_dir, include)

# For each file
for f in stf:
  # Write out filename
  print f
  out_file = open(out_filename, 'a')
  out_file.write("%s\n" % f)
  out_file.close()

  # Copy stats file to this directory
  #os.system("cp -v %s ." % f)

  # Run McPAT until successful
  ret = 1
  while ret != 0:
    # Run McPAT
    if "UMC" in f or "LRC" in f:
      os.system("rm -f results")
      os.system("./m5-mcpat.pl %s config.ini mim_umc_wcet.xml > output.xml" % f)
    else:
      os.system("rm -f results")
      os.system("./m5-mcpat.pl %s config.ini mim_wcet.xml > output.xml" % f)
    ret = os.system("mcpat -infile output.xml -print_level 5 > results")

    # When successful,
    if ret == 0:
      # Save detailed results
      #os.system("cp results %s/mcpat.txt" % root)
      # write results to single outfile
      os.system("./parse_results.py results %s" % out_filename)

sys.exit()
