
import os
import re

"""
This file is meant to provide a library of useful functions for parsing
data for the monitoring_bound project.

Functions:
  average(l)
  geomean(l)
  add_lists(l1, l2)
  hamming_weight(i)

  normalize(data, baseline)
  list_to_csv(l)
  list_to_tex(l)

  get_stats_files(base_dir, include, exclude)
  get_txt_files(base_dir, include, exclude)
  get_queueo_files(base_dir, include, exclude)
  queuenos_from_include(base_dir, include, exclude)
  get_sim_ticks(stats_files)
  get_coverage(stats_files, mon, cpu)
  get_coverage_with_target(stats_files, mon, cpu)
  get_packets(stats_files, cpu)
  get_stat(stats_files, stat_name)
  no_mon_sim_ticks(baseline_dir)

  get_configs(stats_files)
  benchmark_from_path(paths)
  fullbenchmark_from_path(paths):
"""

import os

# Data directory
base_dir = "/Users/dlo/research/monitoring_drop/data/runs/"
# Constants
monitors = ["umc", "hb", "dift"]

"""
Return the arithmetic mean of the passed list.
"""
def average(l):
  # Remove "None" entries
  l2 = []
  for ll in l:
    if ll != None:
      l2.append(ll)
  if not l2:
    return '-'
  else:
    return sum(l2)/len(l2)

"""
Return the geometric mean of the passed list.
"""
def geomean(l):
  prod = 1.0
  n = 0
  for ll in l:
    prod *= ll
    n += 1
  return prod**(1.0/n)

"""
Add the two lists element-wise and return the result. If one of the
lists is non-empty, then return the non-empty list.
"""
def add_lists(l1, l2):
  # Empty l1, return l2
  if not l1:
    return l2
  # Empty l2, return l1
  elif not l2:
    return l1
  # Otherwise, add and return
  else:
    return [ll1 + ll2 for (ll1, ll2) in zip(l1, l2)]

"""
Divide the entries from the first passed list by the values in
the second list. If any entry in the second list is 0, the returned entry is None.
"""
def normalize(data, baseline):
  assert len(data) == len(baseline), "Length of lists (%d != %d) do not match" % (len(data), len(baseline))
  return [(None if float(baseline[i]) == 0 else float(data[i])/float(baseline[i])) for i in range(len(data))]

"""
Calculates the Hamming weight (number of set bits) in the passed integer.
"""
def hamming_weight(i):
  i = str(bin(i)[2:])
  return sum([int(x) for x in i])

"""
Create a comma delimited list for outputting to csv.
"""
def list_to_csv(l):
  return ','.join(['' if x == None else str(x) for x in l])

"""
Create a '&' delimited list for outputting to tex. It is assumed that the passed list
is a list of fractions which should be converted to percentages.
"""
def list_to_tex(l):
  l2 = []
  for ll in l:
    if ll == None:
      l2.append("-")
    elif isinstance(ll, str):
      l2.append(ll)
    elif ll == 0:
      l2.append("0.0\%")
    elif ll < .00005:
      l2.append("%.3f\%%" % (100*ll))
    elif ll < 0.01:
      l2.append("%.2f\%%" % (100*ll))
    else:
      l2.append("%.1f\%%" % (100*ll))
  return ' & '.join(l2)

"""
Return list of stats.txt in the passed directory. include is a list of strings
that must be in the path. exclude is a list of strings that must not be in the
path. The returned list is sorted.
"""
def get_stats_files(base_dir, include = [], exclude = []):
  assert os.path.exists(base_dir), "%s does not exist" % base_dir
  stats_dirs = []
  for root, dirnames, filenames in os.walk(base_dir):
    for filename in filenames:
      if filename == "stats.txt":
        full_path = os.path.join(root, filename)
        add = True
        for e in exclude:
          if e in full_path:
            add = False
            break
        for i in include:
          if i not in full_path:
            add = False
            break
        if add:
          stats_dirs.append(os.path.join(root, filename))
  return sorted(stats_dirs)

"""
Same as get_stats_files but returns an *.txt files.
"""
def get_txt_files(base_dir, include = [], exclude = []):
  assert os.path.exists(base_dir), "%s does not exist" % base_dir
  stats_dirs = []
  for root, dirnames, filenames in os.walk(base_dir):
    for filename in filenames:
      if ".txt" in filename: 
        full_path = os.path.join(root, filename)
        add = True
        for e in exclude:
          if e in full_path:
            add = False
            break
        for i in include:
          if i not in full_path:
            add = False
            break
        if add:
          stats_dirs.append(os.path.join(root, filename))
  return sorted(stats_dirs)

"""
Return a list of queue*.o* files in the passed directory.
"""
def get_queueo_files(base_dir, include = [], exclude = []):
  import re
  assert os.path.exists(base_dir), "%s does not exist" % base_dir
  stats_dirs = []
  for root, dirnames, filenames in os.walk(base_dir):
    for filename in filenames:
      res = re.search("queue[0-9]+\.csh\.o[0-9]+", filename)
      if res:
        full_path = os.path.join(root, filename)
        add = True
        for e in exclude:
          if e in full_path:
            add = False
            break
        for i in include:
          if i not in full_path:
            add = False
            break
        if add:
          stats_dirs.append(os.path.join(root, filename))
  return sorted(stats_dirs)

"""
Return a list of queue numbers that match the include/exclude information. For
example, this can be used to identify which queue number corresponds to a
certain benchmark, monitor configuration.
"""
def queuenos_from_include(base_dir, include = [], exclude = []):
  import re
  assert os.path.exists(base_dir), "%s does not exist" % base_dir
  queue_numbers = []
  for root, dirnames, filenames in os.walk(base_dir):
    for filename in filenames:
      # Find queue*.csh files
      res = re.search("queue([0-9]+)\.csh$", filename)
      if res:
        full_path = os.path.join(root, filename)

        # Found one, open it
        f = open(full_path, 'r')
        for line in f:
          # Find gem5 simulation line
          if "gem5.fast" in line:
            add = True
            # Check that excludes are not in command
            for e in exclude:
              if e in line:
                add = False
                break
            # Check that includes are in command
            for i in include:
              if i not in line:
                add = False
                break
            # If everything checks, then add file to list
            if add:
              queue_numbers.append(res.group(1))
            # Continue on to finding more files
            break
        f.close()

  return queue_numbers

"""
Return sim_ticks for each of the stats.txt files in the passed list.
"""
def get_sim_ticks(stats_files):
  sim_ticks = []
  for stats_file in stats_files:
    f = open(stats_file, 'r')
    found = False
    for line in f:
      if "sim_ticks" in line:
        sim_ticks.append(line.split()[1]) 
        found = True
        break
    f.close()
    if not found:
      sim_ticks.append(0)
  return [float(x) for x in sim_ticks]

"""
Return coverage for each of the stats.txt files in the passed list.
Drop cpu has to be specified. This should be cpu2 when not 
fast-forwarding and switch_cpus2 when fast-forwarding.
The monitor must also be specified in order to know what
instructions are considered checks.
"""
def get_coverage(stats_files, mon, cpu="cpu2"):
  mon = mon.lower()
  # Instruction types that are considered checks
  checks = {"umc":["LOAD"],
      "hb":["LOAD", "STORE"],
      "dift":["INDCTRL"],
      "multidift":["INDCTRL"],
      "lrc":["RET"]}
  
  full = []
  drop = []
  filtered = []
  # For each check instruction type
  for check_inst in checks[mon]:
    # Get counts
    full = add_lists(full, get_stat(stats_files, cpu + ".non_drops::" + check_inst))
    drop = add_lists(drop, get_stat(stats_files, cpu + ".drops::" + check_inst))
    filtered = add_lists(filtered, get_stat(stats_files, cpu + ".filtered::" + check_inst))
  # Total checks
  total = add_lists(add_lists(full, drop), filtered)

  # Normalize
  fulln = normalize(full, total)

  return fulln

"""
Return total number of check packets.
"""
def get_checks(stats_files, mon, cpu):
  # Instruction types that are considered checks
  checks = {"umc":["LOAD"],
      "hb":["LOAD", "STORE"],
      "dift":["INDCTRL"],
      "multidift":["INDCTRL"]}
  
  full = []
  drop = []
  filtered = []
  # For each check instruction type
  for check_inst in checks[mon]:
    # Get counts
    full = add_lists(full, get_stat(stats_files, cpu + ".non_drops::" + check_inst))
    drop = add_lists(drop, get_stat(stats_files, cpu + ".drops::" + check_inst))
    filtered = add_lists(filtered, get_stat(stats_files, cpu + ".filtered::" + check_inst))
  # Total checks
  total = add_lists(add_lists(full, drop), filtered)

  return total

"""
Return coverage for each of the stats.txt files in the passed
list. This version is for runs using a coverage target.
"""
def get_coverage_with_target(stats_files, mon, cpu):
  # Instruction types that are considered checks
  checks = {"umc":["LOAD"],
      "hb":["LOAD", "STORE"],
      "dift":["INDCTRL"]}

  # Check that nothing was dropped by the timer
  drop = get_stat(stats_files, cpu + ".drops::total")
  for d in drop:
    assert(d == 0)
  
  full = []
  coverage_drop = []
  filtered = []
  # For each check instruction type
  for check_inst in checks[mon]:
    # Get counts
    full = add_lists(full, get_stat(stats_files, cpu + ".non_drops::" + check_inst))
    coverage_drop = add_lists(coverage_drop, get_stat(stats_files, cpu + ".coverage_drops::" + check_inst))
    filtered = add_lists(filtered, get_stat(stats_files, cpu + ".filtered::" + check_inst))
  # Total checks
  total = add_lists(add_lists(full, coverage_drop), filtered)

  # Normalize
  fulln = normalize(full, total)

  return fulln



"""
Returns a list containing the percentage breakdown of packets in terms of
(non_drops, drop, filtered). Drop cpu has to be specified. This should be cpu2 when
not fast-forwarding and switch_cpus2 when fast-forwarding.
"""
def get_packets(stats_files, cpu):
  non_drop = get_stat(stats_files, cpu + ".non_drops::total")
  drop = get_stat(stats_files, cpu + ".drops::total")
  filtered = get_stat(stats_files, cpu + ".filtered::total")
  # Total packets
  total = add_lists(add_lists(non_drop, drop), filtered)
  # Normalize
  non_drop = normalize(non_drop, total)
  drop = normalize(drop, total)
  filtered = normalize(filtered, total)

  return (non_drop, drop, filtered)

"""
Return specified statistic for each stats.txt file in the passed list.
"""
def get_stat(stats_files, stat_name):
  stats = []
  for stats_file in stats_files:
    f = open(stats_file, 'r')
    found = False
    for line in f:
      if stat_name in line:
        stats.append(line.split()[1])
        found = True
        break
    f.close()
    if not found:
      stats.append(0)
  return [float(x) for x in stats]

"""
Get sim_ticks for monitor=none in the passed directory.
"""
def no_mon_sim_ticks(baseline_dir):
  stf = get_stats_files(baseline_dir, include = ["none"])
  sim_ticks = get_sim_ticks(stf)
  return sim_ticks

"""
Return a list of all the different configurations included in the
passed list of stats.txt file paths.
"""
def get_configs(stats_files):
  configs = []
  for stats_file in stats_files:
    config = stats_file.split("/")
    if not configs:
      configs = [list() for i in range(len(config))]
    for (i, c) in enumerate(config):
      if c not in configs[i]:
        configs[i].append(c)
  # Remove repeats
  return configs

"""
Strip SPEC benchmark out from passed list of path strings.
"""
def benchmark_from_path(paths):
  benchmarks = []
  for path in paths:
    split_path = path.split("/")
    for sp in split_path:
      res = re.search("^[0-9]+\.([a-z0-9]+)$", sp)
      if res:
        benchmarks.append(res.group(1))
        break
  # Remove repeated benchmarks
  benchmarks = benchmarks
  return benchmarks

"""
Strip Malardalen benchmark out from passed list of path strings.
"""
def malardalen_benchmark_from_path(paths):
  benchmarks = []
  for path in paths:
    split_path = path.split("/")
    res = re.search("^malarden_(.*)_.*\.txt$", split_path[-1])
    if res:
      benchmarks.append(res.group(1))
  # Remove repeated benchmarks
  benchmarks = benchmarks
  return benchmarks




"""
Strip benchmark out from passed list of path strings. Include # designation as
well as benchmark name.
"""
def fullbenchmark_from_path(paths):
  benchmarks = []
  for path in paths:
    split_path = path.split("/")
    for sp in split_path:
      res = re.search("^([0-9]+\.[a-z0-9]+)$", sp)
      if res:
        benchmarks.append(res.group(1))
        break
  # Remove repeated benchmarks
  benchmarks = benchmarks
  return benchmarks


