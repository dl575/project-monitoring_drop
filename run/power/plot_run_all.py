#!/usr/bin/python

import sys
import matplotlib.pyplot as plot
import itertools
import numpy
import re

def plot_one(axis, extension, size, data_index, color):
  axis.plot([data[extension][size][x][data_index] for x in data[extension][size].iterkeys()], color + '.', markersize=10)

def plot_extension(extension):
  ax1 = plot.subplot(221)
  plot_one(ax1, extension, "512B", 0, 'b')
  plot_one(ax1, extension, "1kB",  0, 'r')
  plot_one(ax1, extension, "2kB",  0, 'g')
  plot_one(ax1, extension, "4kB",  0, 'k')
  ax1.set_title("Area")
  ax1.legend(["512B", "1kB", "2kB", "4kB"])
  ax1.set_xlim([-1, 7])

  ax2 = plot.subplot(222)
  plot_one(ax2, extension, "512B", 1, 'b')
  plot_one(ax2, extension, "1kB",  1, 'r')
  plot_one(ax2, extension, "2kB",  1, 'g')
  plot_one(ax2, extension, "4kB",  1, 'k')
  ax2.set_title("Peak Power")
  ax2.set_xlim([-1, 7])

  ax3 = plot.subplot(223)
  plot_one(ax3, extension, "512B", 2, 'b')
  plot_one(ax3, extension, "1kB",  2, 'r')
  plot_one(ax3, extension, "2kB",  2, 'g')
  plot_one(ax3, extension, "4kB",  2, 'k')
  ax3.set_title("Runtime Power")
  ax3.set_xlim([-1, 7])

"""
Returns average and standard deviation of area across all extensions
and benchmarks for the passed cache_size.
"""
def average_area(data, cache_size):
  nums = numpy.array([data[e][cache_size][b][0] for (e, b) in itertools.product(data.keys(), data["dift"][cache_size].keys())])
  return (nums.mean(), nums.std())
def average_absolute_area(data, cache_size):
  nums = numpy.array([data[e][cache_size][b][3] for (e, b) in itertools.product(data.keys(), data["dift"][cache_size].keys())])
  return (nums.mean(), nums.std())

def average_peak_power(data, cache_size):
  nums = numpy.array([data[e][cache_size][b][1] for (e, b) in itertools.product(data.keys(), data["dift"][cache_size].keys())])
  return (nums.mean(), nums.std())

def average_runtime_power(data, cache_size):
  nums = numpy.array([data[e][cache_size][b][2] for (e, b) in itertools.product(data.keys(), data["dift"][cache_size].keys())])
  return (nums.mean(), nums.std())

def average_extension(data, extension, cache_size, data_type):
  nums = numpy.array([data[extension][cache_size][b][data_type] for b in data[extension][cache_size].keys()])
  return (nums.mean(), nums.std())
def average_extension_peak_power(data, extension, cache_size):
  return average_extension(data, extension, cache_size, 1)
def average_extension_runtime_power(data, extension, cache_size):
  return average_extension(data, extension, cache_size, 2)
def average_extension_absolute_peak_power(data, extension, cache_size):
  return average_extension(data, extension, cache_size, 4)
def average_extension_absolute_runtime_power(data, extension, cache_size):
  return average_extension(data, extension, cache_size, 5)

in_filename = sys.argv[1]
in_file = open(in_filename, 'r')

data = {}
current_size = None
current_bench = None
current_extension = None
baseline_area = [] 
baseline_peak = []
baseline_runtime = []
for line in in_file:
  if "Area" in line:
    data[current_extension][current_size][current_bench][0] = float(line.strip().split()[-1])
    res = re.search(" ([0-9\.]+)/([0-9\.]+)", line)
    data[current_extension][current_size][current_bench][3] = float(res.group(1))
    baseline_area.append(float(res.group(2)))
  elif "Peak" in line:
    data[current_extension][current_size][current_bench][1] = float(line.strip().split()[-1])
    res = re.search(" ([0-9\.]+)/([0-9\.]+)", line)
    data[current_extension][current_size][current_bench][4] = float(res.group(1))
    baseline_peak.append(float(res.group(2)))
  elif "Runtime" in line:
    data[current_extension][current_size][current_bench][2] = float(line.strip().split()[-1])
    res = re.search(" ([0-9\.]+)/([0-9\.]+)", line)
    data[current_extension][current_size][current_bench][5] = float(res.group(1))
    baseline_runtime.append(float(res.group(2)))
  else:
    sline = line.split('/')
    current_extension = sline[-4]
    current_size = sline[-3]
    current_bench = sline[-2]
    if current_extension not in data.keys():
      data[current_extension] = {}
    if current_size not in data[current_extension].keys():
      data[current_extension][current_size] = {}
    # Data is [% area, % peak power, % runtime power, abs area, abs peak power, abs runtime power]
    data[current_extension][current_size][current_bench] = [0]*6

in_file.close()

baseline_area = numpy.array(baseline_area)
baseline_peak = numpy.array(baseline_peak)
baseline_runtime = numpy.array(baseline_runtime)
print "Baseline:"
print "  Area %.2f mm^2 std(%.1f)" % (baseline_area.mean(), baseline_area.std())
print "  Peak Power %.1f mW std(%.1f)" % (baseline_peak.mean()*1000, baseline_peak.std()*1000)
print "  Runtime power %.1f mW std(%.1f)" % (baseline_runtime.mean()*1000, baseline_runtime.std()*1000)

print data["dift"]["1kB"].keys()

#for cache_size in ["512B", "1kB", "2kB", "4kB"]:
for cache_size in ["1kB"]:
  print cache_size
  print "   Area (% mean, std) (absolute mean, std)"
  print "  ", average_area(data, cache_size), average_absolute_area(data, cache_size)
  for extension in ["dift", "umc", "hb"]:
    print "  ", extension
    print "     Peak Power (% mean, std) (absolute mean, std)"
    print "    ", average_extension_peak_power(data, extension, cache_size), average_extension_absolute_peak_power(data, extension, cache_size)
    print "     Runtime Power (% mean, std) (absolute mean, std)"
    print "    ", average_extension_runtime_power(data, extension, cache_size), average_extension_absolute_runtime_power(data, extension, cache_size)

# Plot data
"""
fig = plot.figure()
fig.suptitle("DIFT")
plot_extension("dift")

fig = plot.figure()
fig.suptitle("UMC")
plot_extension("umc")

fig = plot.figure()
fig.suptitle("BC")
plot_extension("hb")

plot.show()
"""
