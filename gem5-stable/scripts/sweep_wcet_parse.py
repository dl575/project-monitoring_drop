#!/usr/bin/python

"""
Parse output generated by sweep_wcet.py. Log files are assumed to be at
$GEM5/wcet*.log. For each log file, the number of dropped events, processed
events, and totla execution file are extracted. The number of dropped
and non-dropped events are plotted versus WCET. The execution time is also
plotted versus WCET.
"""

import glob
import re
import subprocess
import os

import matplotlib.pyplot as plot

program = "timer_monitor"

# directory where simulation results are stored
log_dir = os.environ["GEM5"] + "/m5out/%s/" % program

# Lists for storing data
wcets = []
drops = []
not_drops = []
filtered = []
ticks = []

# For each log file
for filename in glob.glob(os.path.join(log_dir, "wcet*.log")):
  # Save WCET for this simulation run
  wcets.append(int(re.search("wcet([\d]*).log", filename).group(1)))
  log_file = open(filename, 'r')
  for line in log_file:
    # Find drops line
    drop_match = re.search("Drops = ([\d]*), Non-drops = ([\d]*), Filtered = ([\d]*)", line)
    # Find total ticks
    tick_match = re.search("Exiting @ tick ([\d]*)", line)
    if drop_match:
      drops.append(int(drop_match.group(1)))
      not_drops.append(int(drop_match.group(2)))
      filtered.append(int(drop_match.group(3)))
    if tick_match:
      ticks.append(int(tick_match.group(1))) 
  log_file.close()

# Print out drop results to check that data is parsed correctly
print wcets
print drops

# Plot results
fig = plot.figure()

# Plot drops/non-drops versus WCET
ax1 = plot.subplot(211)
ax1.scatter(wcets, drops, c='r')
ax1.scatter(wcets, not_drops, c='g')
ax1.scatter(wcets, filtered, c='b')
plot.xlabel("WCET")
plot.ylabel("Drops")
plot.grid(True)
plot.legend(("Dropped", "Not dropped", "Filtered"), loc="best")

# Plot execution ticks versus WCET
plot.subplot(212)
plot.scatter(wcets, ticks)
plot.xlabel("WCET")
plot.ylabel("Execution time")
plot.grid(True)

plot.show()
