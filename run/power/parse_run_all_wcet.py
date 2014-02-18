#!/usr/bin/python

from parse_lib import *
import sys
import re
import numpy

if len(sys.argv) != 2:
  print "usage: parse_run_all_wcet.py run_all_results.txt"
  sys.exit()
in_filename = sys.argv[1]

in_file = open(in_filename, 'r')

# Parse data
data = {}
baseline_area = []
baseline_peak = []
baseline_runtime = []
for line in in_file:
  if "Area" in line:
    res = re.search(" ([0-9\.]+)/([0-9\.]+) = ([0-9\.]+)", line)
    # Absolute area
    data[model][mon][bench][3] = float(res.group(1))
    # Baseline area
    baseline_area.append(float(res.group(2)))
    # % area
    data[model][mon][bench][0] = float(res.group(3))
  elif "Peak" in line:
    res = re.search(" ([0-9\.]+)/([0-9\.]+) = ([0-9\.]+)", line)
    # Absolute peak power
    data[model][mon][bench][4] = float(res.group(1))
    # Baseline peak power
    baseline_peak.append(float(res.group(2)))
    # % peak power
    data[model][mon][bench][1] = float(res.group(3))
  elif "Runtime" in line:
    res = re.search(" ([0-9\.]+)/([0-9\.]+) = ([0-9\.]+)", line)
    # Absolute runtime power
    data[model][mon][bench][5] = float(res.group(1))
    # Baseline runtime power
    baseline_runtime.append(float(res.group(2)))
    # % runtime power
    data[model][mon][bench][2] = float(res.group(3))
  # Parse out current configuration
  else:
    sline = line.split('/')
    mon = ''.join(sline[-3].split('_')[0:-1])
    model = sline[-2]
    bench = sline[-1].split('_')[1]

    # Create entries in data dict if they do not exist
    if model not in data.keys():
      data[model] = {}
    if mon not in data[model].keys():
      data[model][mon] = {}
    # (% area, % peak, % runtime, area, peak, runtime)
    data[model][mon][bench] = [0]*6


in_file.close()

# Print out baseline data
baseline_area = numpy.array(baseline_area)
baseline_peak = numpy.array(baseline_peak)
baseline_runtime = numpy.array(baseline_runtime)
print "Baseline:"
print "  Area %.2f mm^2 std(%.1f)" % (baseline_area.mean(), baseline_area.std())
print "  Peak Power %.1f mW std(%.1f)" % (baseline_peak.mean()*1000, baseline_peak.std()*1000)
print "  Runtime power %.1f mW std(%.1f)" % (baseline_runtime.mean()*1000, baseline_runtime.std()*1000)

# Return average of statistic across benchmarks
def avg_benchmarks(d, index):
  l = numpy.array([d[b][index] for b in d.keys()])
  return (l.mean(), l.std())

for b in data["TIMING"]["UMC"].keys():
  data["TIMING"]["UMC"][b][0] = 0
for model in ["TIMING", "FLEXHW"]:

  print model
  print "  Area (% mean, std) (absolute mean,s td)"
  abs_area = []
  percent_area = []
  for mon in data[model].keys():
    for bench in data[model][mon].keys():
      abs_area.append(data[model][mon][bench][0])
      percent_area.append(data[model][mon][bench][3])
  abs_area = numpy.array(abs_area)
  percent_area = numpy.array(percent_area)
  print "  ", (abs_area.mean(), abs_area.std()), (percent_area.mean(), percent_area.std())

  # print "  ", avg_benchmarks(d, 0), avg_benchmarks(d, 3)
  for mon in ["UMC", "LRC", "DIFT", "DIFTRF"]:
    d = data[model][mon]
    print "  ", mon
    print "     Peak Power (% mean, std) (absolute mean, std)"
    print "    ", avg_benchmarks(d, 1), avg_benchmarks(d, 4)
    print "     Runtime Power (% mean, std) (absolute mean, std)"
    print "    ", avg_benchmarks(d, 2), avg_benchmarks(d, 5)
