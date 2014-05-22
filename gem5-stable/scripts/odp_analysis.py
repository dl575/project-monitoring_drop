#!/usr/bin/env python

import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt

def find_config(sim_dir):
    json_files = filter(lambda x : x.endswith('.json'), os.listdir(sim_dir))
    if len(json_files) != 1:
        raise IOError, "Cannot find config file in %s" % (sim_dir)
    else:
        return os.path.join(sim_dir, json_files[0])

def load_config(config_file):
    with open(config_file) as f:
        return json.load(f)

def parse_checksets(fn):
    '''Return a list containing the size of each check set'''
    checkset_sizes = []
    with open(fn) as f:
        for line in f:
            fields = line.strip().split(',')
            if len(fields) < 2:
                continue
            checkset_sizes.append(len(fields)-1)
    # sort it
    checkset_sizes.sort()
    return checkset_sizes

def parse_odps(fn):
    '''Return a list of optimal dropping points'''
    odps = []
    with open(fn) as f:
        for line in f:
            line = line.strip()
            if line:
                odps.append(int(line, 16))
    return odps

def parse(sim_dir):
    # find config file inside sim_dir
    config_file = find_config(sim_dir)
    config = load_config(config_file)
    data = {}
    for monitor in config['monitor']:
        if not data.has_key(monitor):
            data[monitor] = {}
        for benchmark in config['benchmarks']:
            if not data[monitor].has_key(benchmark):
                data[monitor][benchmark] = {}
            out_dir = os.path.join(sim_dir, 'atomic', 'core_0.5GHz', 'mon_0.25GHz', monitor, '1kB', benchmark, 'overhead_0.1000')
            checksets = parse_checksets(os.path.join(out_dir, 'checksets.txt'))
            data[monitor][benchmark]['checksets'] = checksets
            odps = parse_odps(os.path.join(out_dir, 'optimal_dropping.txt'))
            data[monitor][benchmark]['odps'] = odps
    return data

def plot(data):
    # plot check sets
    for monitor in data.keys():
        for benchmark in data[monitor].keys():
            plt.clf()
            # compute CDF of check set size
            checksets = data[monitor][benchmark]['checksets']
            X = range(1, len(checksets)+1)
            Y = np.array(checksets)
            plt.plot(X, Y)
            plt.xlabel('Instruction #')
            plt.ylabel('Check set size')
            plt.xlim(0, len(checksets))
            plt.title('%s_%s' % (monitor, benchmark))
            plt.savefig('%s_%s_cdf.pdf' % (monitor, benchmark), format='pdf')
    # plot optimal dropping points
    for benchmark in data.values()[0].keys():
        plt.clf()
        x_pos = np.arange(len(data.keys()))
        n_odps = []
        xticks = []
        for monitor in data.keys():
            n_odps.append(len(data[monitor][benchmark]['odps']))
            xticks.append(monitor)
        plt.bar(x_pos, n_odps, align='center', color='grey')
        plt.xticks(x_pos, xticks)
        plt.ylabel('# of ODPs')
        plt.ylim(ymin=0)
        plt.title('%s' % (benchmark))
        plt.savefig('%s_odp.pdf' % (benchmark), format='pdf')

def show_usage():
    print 'Usage:', sys.argv[0], 'sim_dir'

def main():
    if len(sys.argv) != 2:
        show_usage()
        exit(1)
    else:
        data = parse(sys.argv[1])
        plot(data)

if __name__ == '__main__':
    main()

