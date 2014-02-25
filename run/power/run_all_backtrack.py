#!/usr/bin/python

import glob
import sys
import os

base_dir = sys.argv[1]

# Clear output files
metadata_out_filename = "run_all_results_metadata.txt"
out_file = open(metadata_out_filename, 'w')
out_file.close()
backtrack_out_filename = "run_all_results_backtrack.txt"
out_file = open(backtrack_out_filename, 'w')
out_file.close()

# Find all directories with stats.txt files and using 0.1 overhead
roots = []
for root,dirs,files in os.walk(base_dir):
  for f in files:
    if f == "stats.txt" and "overhead_0.1000" in root:
      roots.append(root)

# For each directory
for root in roots:
  print root
  out_file = open(metadata_out_filename, 'a')
  out_file.write("%s\n" % root)
  out_file.close()
  out_file = open(backtrack_out_filename, 'a')
  out_file.write("%s\n" % root)
  out_file.close()
  # Copy in files
  os.system("cp %s/* ." % root)
  # Run McPat until successful
  ret = 1
  while ret != 0:
    if "umc" in root:
      os.system("./m5-mcpat.pl stats.txt config.ini mim_backtrack_umc.xml > output.xml")
    else:
      os.system("./m5-mcpat.pl stats.txt config.ini mim_backtrack.xml > output.xml")
    ret = os.system("mcpat -infile output.xml -print_level 5 > results")
    if ret == 0:
      os.system("cp results %s/mcpat.txt" % root)
      os.system("./parse_results.py results %s %s" % (metadata_out_filename, backtrack_out_filename))
