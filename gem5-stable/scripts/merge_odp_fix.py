#!/usr/bin/env python

import os
import sys

SPEC = ["400.perlbench", "401.bzip2", "403.gcc", "429.mcf", "445.gobmk", "456.hmmer",
		"458.sjeng", "462.libquantum", "464.h264ref", "471.omnetpp", "473.astar", "483.xalan"]

MONITORS = ['umc', 'hb', 'dift']

NODROP_THRESHOLD = {
	'umc' : {
		"400.perlbench"		: 20,
		"401.bzip2"			: 10,
		"403.gcc"			: 10,
		"429.mcf"			: 5,
		"445.gobmk"			: 10,
		"456.hmmer"			: 5,
		"458.sjeng"			: 10,
		"462.libquantum"	: 3,
		"464.h264ref"		: 5,
		"471.omnetpp"		: 3,
		"473.astar"			: 2,
		"483.xalan"			: 5
	},
	'hb' : {
		"400.perlbench"		: 800,
		"401.bzip2"			: 100,
		"403.gcc"			: 200,
		"429.mcf"			: 100,
		"445.gobmk"			: 600,
		"456.hmmer"			: 800,
		"458.sjeng"			: 800,
		"462.libquantum"	: 150,
		"464.h264ref"		: 600,
		"471.omnetpp"		: 600,
		"473.astar"			: 40,
		"483.xalan"			: 600
	},
	'dift' : {
		"400.perlbench"		: 1000,
		"401.bzip2"			: 100,
		"403.gcc"			: 600,
		"429.mcf"			: 100,
		"445.gobmk"			: 380,
		"456.hmmer"			: 200,
		"458.sjeng"			: 80,
		"462.libquantum"	: 25,
		"464.h264ref"		: 100,
		"471.omnetpp"		: 100,
		"473.astar"			: 15,
		"483.xalan"			: 800
	}
}

def parse_odp(odp_file):
	'''Return a list of optimal dropping points'''
	odp = []
	with open(odp_file) as f:
		for line in f:
			line = line.strip()
			if line:
				odp.append(int(line, 16))
	return odp

def parse_checksets(checkset_file, monitor, benchmark):
	'''Return a list of nodes with empty checksets and
		a list of nodes with a large number of checksets'''
	empty_checksets = []
	large_checksets = []
	threshold = 999
	#threshold = NODROP_THRESHOLD[monitor][benchmark]
	with open(checkset_file) as f:
		for line in f:
			fields = line.strip().split(',')
			if len(fields) == 1:
				# empty check set
				empty_checksets.append(int(fields[0], 16))
			elif len(fields) > threshold:
				large_checksets.append(int(fields[0], 16))
	return empty_checksets, large_checksets

def show_usage():
	print 'Usage:', sys.argv[0], 'sim_dir'

def main():
	if len(sys.argv) != 2:
		show_usage()
		exit(1)
	for monitor in MONITORS:
		for benchmark in SPEC:
			path = os.path.join(sys.argv[1], 'atomic', 'core_0.5GHz', 'mon_0.25GHz',
								monitor, '1kB', benchmark, 'overhead_0.1000')
			checkset_file = os.path.join(path, 'checksets.txt')
			odp_file = os.path.join(path, 'optimal_dropping.txt')
			odp = parse_odp(odp_file)
			empty_checksets, large_checksets = parse_checksets(checkset_file, monitor, benchmark)
			# merge odp and empty checksets
			#merged_odp = list(set(odp).union(set(empty_checksets)).difference(set(large_checksets)))
			merged_odp = list(set(odp).union(set(empty_checksets)))
			merged_odp.sort()
			# write back to file
			with open(os.path.join(path, 'odp.txt'), 'w') as f:
				for odp in merged_odp:
					f.write(hex(odp)[2:] + '\n')

if __name__ == '__main__':
	main()
