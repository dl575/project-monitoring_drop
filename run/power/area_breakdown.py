#!/usr/bin/python

import sys

in_filename = sys.argv[1]

in_file = open(in_filename, 'r')

mim_total = -1
mirf = -1
mim_alu = -1
mic = -1
mim_config = -1
mfm_alu = -1
mfm_config = -1
mfm_table = -1

def get_next_data(in_file):
  return float(in_file.readline().strip().split()[-2])

while True:
  line = in_file.readline()
  if len(line) == 0:
    break

  if "Metadata Invalidation Module" in line:
    mim_total = get_next_data(in_file)
  elif "Metadata Register Files" in line:
    mirf = get_next_data(in_file)
  elif "MIM Integer ALUs" in line:
    mim_alu = get_next_data(in_file)
  elif "Metadata Load Store Unit" in line:
    mic = get_next_data(in_file)
  elif "MIM Config Table" in line:
    mim_config = get_next_data(in_file)
  elif "MFM Integer ALUs" in line:
    mfm_alu = get_next_data(in_file)
  elif "MFM Config Table" in line:
    mfm_config = get_next_data(in_file)
  elif "MFM Filter Lookup Table" in line:
    mfm_table = get_next_data(in_file)

in_file.close()

alus = mim_alu + mfm_alu
configs = mim_config + mfm_config + mfm_table
mirf = mirf
mic = mic

print "ALU \t %f" % (alus/mim_total)
print "Config Tables \t %f" % (configs/mim_total)
print "MIRF \t %f" % (mirf/mim_total)
print "MIC \t %f" % (mic/mim_total)
