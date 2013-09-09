#!/usr/bin/python

import sys
import re

in_filename = sys.argv[1]
out_filename = sys.argv[2]
in_file = open(in_filename, 'r')

core = False
metadata = False
for line in in_file:
  # New section
  if ":" in line:
    core = False
    metadata = False
  # Core section
  if line.strip() == "Core:":
    core = True
    metadata = False
  # Metadata hardware section
  if line.strip() == "Metadata Invalidation Module:":
    core = False
    metadata = True

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
in_file.close()

out_file = open(out_filename, 'a')
out_file.write("Area: %f/%f = %f\n" % (metadata_area, core_area, metadata_area/core_area))
out_file.write("Peak: %f/%f = %f\n" % (metadata_total_peak, core_total_peak, metadata_total_peak/core_total_peak))
out_file.write("Runtime: %f/%f = %f\n" % (metadata_total_runtime, core_total_runtime, metadata_total_runtime/core_total_runtime))
out_file.close()
