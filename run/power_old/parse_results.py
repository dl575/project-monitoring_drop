#!/usr/bin/python

import sys
import re

in_filename = sys.argv[1]
# Store metadata area/power in one file
metadata_filename = sys.argv[2]
if len(sys.argv) > 3:
  # Store backtrack area/power in another file
  backtrack_filename = sys.argv[3]
else:
  backtrack_filename = False
in_file = open(in_filename, 'r')

core = False
metadata = False
backtrack = False
for line in in_file:
  # New section
  if ":" in line:
    core = False
    metadata = False
    backtrack = False
  # Core section
  if line.strip() == "Core:":
    core = True
  # Backtracking hardware
  if line.strip() == "Metadata Invalidation Module:":
    metadata = True
  # Backtracking hardware
  if line.strip() == "Backtrack Hardware:":
    backtrack = True

  if core:
    res = re.search("Total Peak = ([0-9\.]+)", line)
    if res:
      core_total_peak = float(res.group(1))
    res = re.search("Total Runtime = ([0-9\.]+)", line)
    if res:
      core_total_runtime = float(res.group(1))
    res = re.search("Area = ([0-9\.]+)", line)
    if res:
      core_area = float(res.group(1))

  if metadata:
    res = re.search("Total Peak = ([0-9\.]+)", line)
    if res:
      metadata_total_peak = float(res.group(1))
    res = re.search("Total Runtime = ([0-9\.]+)", line)
    if res:
      metadata_total_runtime = float(res.group(1))
    res = re.search("Area = ([0-9\.]+)", line)
    if res:
      metadata_area = float(res.group(1))

  if backtrack:
    res = re.search("Total Peak = ([0-9\.]+)", line)
    if res:
      backtrack_total_peak = float(res.group(1))
    res = re.search("Total Runtime = ([0-9\.]+)", line)
    if res:
      backtrack_total_runtime = float(res.group(1))
    res = re.search("Area = ([0-9\.]+)", line)
    if res:
      backtrack_area = float(res.group(1))

in_file.close()

# Remove metadata+backtrack area/power from core total
core_area -= metadata_area
core_total_peak -= metadata_total_peak
core_total_runtime -= metadata_total_runtime
# Remove backtrack area/power from metadat total
metadata_area -= backtrack_area
metadata_total_peak -= backtrack_total_peak
metadata_total_runtime -= backtrack_total_runtime

# Write out metadata area/power
out_file = open(metadata_filename, 'a')
out_file.write("Area: %f/%f = %f\n" % (metadata_area, core_area, metadata_area/core_area))
out_file.write("Peak: %f/%f = %f\n" % (metadata_total_peak, core_total_peak, metadata_total_peak/core_total_peak))
out_file.write("Runtime: %f/%f = %f\n" % (metadata_total_runtime, core_total_runtime, metadata_total_runtime/core_total_runtime))
out_file.close()

# Write out backtrack area/power
if backtrack_filename:
  out_file = open(backtrack_filename, 'a')
  out_file.write("Area: %f/%f = %f\n" % (backtrack_area, core_area, backtrack_area/core_area))
  out_file.write("Peak: %f/%f = %f\n" % (backtrack_total_peak, core_total_peak, backtrack_total_peak/core_total_peak))
  out_file.write("Runtime: %f/%f = %f\n" % (backtrack_total_runtime, core_total_runtime, backtrack_total_runtime/core_total_runtime))
  out_file.close
