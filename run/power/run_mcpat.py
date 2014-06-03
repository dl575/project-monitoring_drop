#!/usr/bin/python

"""
Runs McPat on all simulation outputs in the passed directory. The optional --norun flag
skips running McPat and can be used to parse existing data.

usage: run_mcpat.py directory config.xml [--norun]
"""

from parse_lib import *
from config import *
import sys
import os
import numpy

"""
Check for outliers in the passed stats. We define an outlier as being 3
standard deviations above or below the mean.
"""
def check_outliers(stats, n=3):
  for i in range(3):
    s = stats[:, i]
    mean = s.mean()
    std = s.std()
    if max(stats[:, i]) > mean + n*std or min(stats[:, i]) < mean - n*std:
      print "  Warning: Outlier found for %s" % (["area", "peak", "runtime"][i])
      print "    mean +- %d*std = [%f, %f]" % (n, mean - n*std, mean + n*std)
      print "    min, max = [%f, %f]" % (min(s), max(s))


# Handle command line arguments
if len(sys.argv) < 3:
  print __doc__
  sys.exit()
base_dir = sys.argv[1]
xml_file = sys.argv[2]
norun = False
if len(sys.argv) == 4:
  if sys.argv[3] == "--norun":
    norun = True

######################################################################
# Run McPat for all simulation runs
######################################################################
if not norun:
  # Get stats.txt and config.ini files
  stats_files = get_stats_files(base_dir, include=include)
  config_files = get_files(base_dir, "config.ini", include=include)

  # For each simulation run
  for (stat_file, config_file) in zip(stats_files, config_files):
    print "Processing %s" % (stat_file)
    # Create config xml using simulation data
    if "umc" in stat_file:
      os.system("./m5-mcpat.pl %s %s %s.umc > output.xml" % (stat_file, config_file, xml_file))
    else:
      os.system("./m5-mcpat.pl %s %s %s > output.xml" % (stat_file, config_file, xml_file))
    # Run McPat until successful
    ret = 1
    while ret != 0:
      ret = os.system("mcpat -infile output.xml -print_level 5 > results.txt")
    # Save full McPat results back in simulation directory
    os.system("cp results.txt %s/mcpat.txt" % os.path.dirname(stat_file))
    # Cleanup
    os.system("rm results.txt output.xml")
  print

######################################################################
# Parse core results
######################################################################
mcpat_files = get_files(base_dir, "mcpat.txt")
core_stats = get_mcpat_stats(mcpat_files, "Core:")
# Get MIM results and subtract them out
mim_stats = get_mcpat_stats(mcpat_files, "Metadata Invalidation Module:")
assert(len(core_stats) == len(mim_stats))
core_stats = numpy.array(core_stats)
mim_stats = numpy.array(mim_stats)
core_stats -= mim_stats

# Print out averages and standard deviations
print "Core"
check_outliers(core_stats)
corem = [core_stats[:, i].mean() for i in range(3)]
print "  Area: ", corem[0]
print "  Peak: ", corem[1]
print "  Runtime: ", corem[2]

######################################################################
# Print out results for each monitor
######################################################################
for mon in monitors:
  print mon
  # Get MIM and Backtrack Hardware McPat results
  mcpat_files = get_files(base_dir, "mcpat.txt", include=[mon])
  mim_stats = get_mcpat_stats(mcpat_files, "Metadata Invalidation Module:")
  backtrack_stats = get_mcpat_stats(mcpat_files, "Backtrack Hardware:")
  assert(len(mim_stats) == len(backtrack_stats))
  # Convert to numpy arrays
  mim_stats = numpy.array(mim_stats)
  backtrack_stats = numpy.array(backtrack_stats)
  # Subtract out backtrack hardware from MIM results
  mim_stats -= backtrack_stats

  print "  Metadata Invalidation Module"
  check_outliers(mim_stats)
  mimm = [mim_stats[:, i].mean() for i in range(3)]
  nmimm = normalize(mimm, corem)
  print "    Area: %f (%f%%)" % (mimm[0], 100*nmimm[0])
  print "    Peak: %f (%f%%)" % (mimm[1], 100*nmimm[1])
  print "    Runtime: %f (%f%%)" % (mimm[2], 100*nmimm[2])
  # print "  Backtrack Hardware"
  # check_outliers(backtrack_stats)
  # btm = [backtrack_stats[:, i].mean() for i in range(3)]
  # nbtm = normalize(btm, corem)
  # print "    Area: %f (%f%%)" % (btm[0], 100*nbtm[0])
  # print "    Peak: %f (%f%%)" % (btm[1], 100*nbtm[1])
  # print "    Runtime: %f (%f%%)" % (btm[2], 100*nbtm[2])
