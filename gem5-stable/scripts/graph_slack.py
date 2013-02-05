#!/usr/bin/python

"""
Plot slack from stdin
"""

import re
import sys
import matplotlib.pyplot as plot

time = []
slack = []
readtime = []
readslack = []

while 1:
    line = sys.stdin.readline()
    if not line:
        break
    start = re.search("Written to timer: task start", line)
    if start:
        time = []
        slack = []
        readtime = []
        readslack = []
    m = re.search("^(\d+).*slack\s*=.*?(-?\d+)$", line)
    if m:
        time.append(int(m.group(1)))
        slack.append(int(m.group(2)))
    r = re.search("^(\d+).*read from timer:\s*(-?\d+)\s*ticks", line)
    if r:
        readtime.append(int(r.group(1)))
        readslack.append(int(r.group(2)))
    end = re.search("Written to timer: task end", line)
    if end:
        # Plot results
        fig = plot.figure()
        # Plot slack versus time
        ax1 = plot.subplot(111)
        ax1.plot(time, slack, '-', readtime, readslack, 'o')
        plot.grid(True)

plot.show()
