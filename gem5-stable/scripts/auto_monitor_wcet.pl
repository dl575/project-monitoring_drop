#!/usr/bin/perl

use strict;
use warnings;

my $gem5 = $ENV{'GEM5'};
die "GEM5 path not defined" unless defined $gem5;

my @monitors = ('UMC_FULL', 'UMC_SWDROP', 'UMC_HWDROP', 'UMC_HWFILTER', 'LRC_FULL', 'LRC_SWDROP', 'LRC_HWDROP', 'LRC_HWFILTER');
my @models = ('ATOMIC', 'TIMING');
my %ticks_to_cycles = ('ATOMIC'=>800, 'TIMING'=>800);
my $output_str = "";
my $default_full = 100;
my $default_drop = 10;

foreach my $model (@models){
    system("cd $gem5/tests/monitoring; unzip -o monitoring.$model.zip") && die "Could not unzip monitoring.$model.zip\n";
    my $t2c = (defined $ticks_to_cycles{$model})? $ticks_to_cycles{$model} : 1;
    $ENV{'MODEL'} = $model;
    $output_str .= "#ifdef $model\n";
    foreach my $monitor(@monitors){
        $ENV{'MONITOR'} = $monitor;
        my $wcet_cmd = "";
        if ($model eq 'ATOMIC'){
            $wcet_cmd = "perl $gem5/scripts/monitor_wcet_atomic.pl";
        } elsif ($model eq 'TIMING'){
            $wcet_cmd = "perl $gem5/scripts/monitor_wcet_timing.pl";
        }
        my $full_delay = $default_full;
        my $drop_delay = $default_drop;
        open my $ch, "-|", $wcet_cmd or die "Could not run monitor WCET.";
        while (<$ch>){
            if (/Max full delay is:\s*(\d+)/){
                $full_delay = $1/$t2c;
            }
            if (/Max drop delay is:\s*(\d+)/){
                $drop_delay = $1/$t2c;
            }
        }
        close $ch;
        $output_str .= "#ifdef $monitor\n";
        $output_str .= "\t#define MON_WCET $full_delay\n";
        $output_str .= "\t#define MON_DROP_WCET $drop_delay\n";
        $output_str .= "#endif\n";
    }
    $output_str .= "#endif\n";
}

open my $fh, ">", "$gem5/tests/monitoring/monitor_timing.h" or die "Could not create file $gem5/tests/monitoring/monitor_timing.h\n";
print $fh "/*
 * Include file for information about monitor timing
 * This is AUTO GENERATED.
 * Author: Mohamed Ismail
 */
 
#ifndef __MONITOR_TIME_H__
#define __MONITOR_TIME_H__

";

print $fh $output_str;

print $fh "\n#endif // __MONITOR_TIME_H__\n";

close $fh;

system ("perl $gem5/scripts/build_monitors.pl");
