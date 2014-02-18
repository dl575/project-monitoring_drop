#!/usr/bin/perl

use strict;
use warnings;
use POSIX;

my $gem5 = $ENV{'GEM5'};
die "GEM5 path not defined" unless defined $gem5;

my %monitors = ('ATOMIC' => ['UMC_FULL', 'UMC_SWDROP', 'UMC_HWDROP', 'UMC_HWFILTER', 'LRC_FULL', 'LRC_SWDROP', 'LRC_HWDROP', 'LRC_HWFILTER', 'DIFT_FULL', 'DIFT_SWDROP', 'DIFT_HWDROP', 'DIFT_HWFILTER', 'DIFT_RF_HWDROP', 'DIFT_RF_HWFILTER'],
                'TIMING' => ['UMC_FULL', 'UMC_SWDROP', 'UMC_HWDROP', 'UMC_HWFILTER', 'LRC_FULL', 'LRC_SWDROP', 'LRC_HWDROP', 'LRC_HWFILTER', 'DIFT_FULL', 'DIFT_SWDROP', 'DIFT_HWDROP', 'DIFT_HWFILTER', 'DIFT_RF_HWDROP', 'DIFT_RF_HWFILTER'],
                'FLEX' => ['UMC_FULL', 'UMC_SWDROP', 'UMC_HWDROP', 'UMC_HWFILTER', 'LRC_FULL', 'LRC_SWDROP', 'LRC_HWDROP', 'LRC_HWFILTER', 'DIFT_FULL', 'DIFT_SWDROP', 'DIFT_HWDROP', 'DIFT_HWFILTER'],
                'FLEXHW' => ['UMC_HWDROP', 'UMC_HWFILTER', 'LRC_HWDROP', 'LRC_HWFILTER', 'DIFT_HWDROP', 'DIFT_HWFILTER', 'DIFT_RF_HWDROP', 'DIFT_RF_HWFILTER']
                );
my @models = ('ATOMIC', 'TIMING', 'FLEXHW'); #, 'TIMING', 'FLEX');

# FlexCore: 2 cycles for atomic
# Kindle: 500 MHz -> 2 ns -> 2000 ticks
my %ticks_to_cycles = ('FLEX' => {'UMC_FULL'=>210, 'UMC_SWDROP'=>160, 'UMC_HWDROP'=>148, 'UMC_HWFILTER'=>166, 
                                  'LRC_FULL'=>666, 'LRC_SWDROP'=>444, 'LRC_HWDROP'=>307, 'LRC_HWFILTER'=>307,
                                  "DIFT_FULL"=>153, "DIFT_SWDROP"=>142, "DIFT_HWDROP"=>95, "DIFT_HWFILTER"=>133 },
                       'ATOMIC' => {'UMC_FULL'=>2000, 'UMC_SWDROP'=>2000, 'UMC_HWDROP'=>2000, 'UMC_HWFILTER'=>2000, 
                                    'LRC_FULL'=>2000, 'LRC_SWDROP'=>2000, 'LRC_HWDROP'=>2000, 'LRC_HWFILTER'=>2000,
                                    "DIFT_FULL"=>2000, "DIFT_SWDROP"=>2000, "DIFT_HWDROP"=>2000, "DIFT_HWFILTER"=>2000, "DIFT_RF_HWDROP"=>2000, "DIFT_RF_HWFILTER"=>2000 },
                       'TIMING' => {'UMC_FULL'=>2000, 'UMC_SWDROP'=>2000, 'UMC_HWDROP'=>2000, 'UMC_HWFILTER'=>2000, 
                                    'LRC_FULL'=>2000, 'LRC_SWDROP'=>2000, 'LRC_HWDROP'=>2000, 'LRC_HWFILTER'=>2000,
                                    "DIFT_FULL"=>2000, "DIFT_SWDROP"=>2000, "DIFT_HWDROP"=>2000, "DIFT_HWFILTER"=>2000, "DIFT_RF_HWDROP"=>2000, "DIFT_RF_HWFILTER"=>2000 },
                       'FLEXHW' => {'UMC_HWDROP'=>2000, 'LRC_HWDROP'=>2000, 'DIFT_HWDROP'=>2000, 'DIFT_RF_HWDROP'=>2000,
                                    'UMC_HWFILTER'=>2000, 'LRC_HWFILTER'=>2000, 'DIFT_HWFILTER'=>2000, 'DIFT_RF_HWFILTER'=>2000 }
                      );

my $default_full = 100;
my $default_drop = 10;

open my $fh, ">", "$gem5/tests/monitoring/monitor_timing.h" or die "Could not create file $gem5/tests/monitoring/monitor_timing.h\n";
print $fh "/*
 * Include file for information about monitor timing
 * This is AUTO GENERATED.
 * Author: Mohamed Ismail
 */
 
#ifndef __MONITOR_TIME_H__
#define __MONITOR_TIME_H__

";

my $pyfile = "#
# Include file for information about monitor timing
# This is AUTO GENERATED.
# Author: Mohamed Ismail
#
 
";

foreach my $model (@models){
    # system("cd $gem5/tests/monitoring; unzip -o monitoring.$model.zip") && die "Could not unzip monitoring.$model.zip\n";
    if ($model ne 'FLEXHW'){
        system("cd $gem5/tests/monitoring; tar -xvjf monitoring.$model.bz2") && die "Could not unzip monitoring.$model.bz2\n";
    }
    $ENV{'MODEL'} = $model;
    print $fh "#ifdef $model\n";
    $pyfile .= "if model == '$model':\n";
    foreach my $monitor(@{$monitors{$model}}){
        my $t2c = (defined $ticks_to_cycles{$model} && defined $ticks_to_cycles{$model}->{$monitor})? $ticks_to_cycles{$model}->{$monitor} : 1;
        $ENV{'MONITOR'} = $monitor;
        my $wcet_cmd = "";
        if ($model eq 'ATOMIC'){
            $wcet_cmd = "perl $gem5/scripts/monitor_wcet_atomic.pl";
        } elsif ($model eq 'TIMING'){
            $wcet_cmd = "perl $gem5/scripts/monitor_wcet_timing.pl";
        } elsif ($model eq 'FLEX'){
            $wcet_cmd = "perl $gem5/scripts/monitor_wcet_flex.pl";
        } elsif ($model eq 'FLEXHW'){
            $wcet_cmd = "perl $gem5/scripts/monitor_wcet_flexhw.pl";
        }
        my $full_delay = $default_full;
        my $drop_delay = $default_drop;
        open my $ch, "-|", $wcet_cmd or die "Could not run monitor WCET.";
        while (<$ch>){
            if (/Max full delay is:\s*(\d+)/){
                $full_delay = ceil($1/$t2c);
            }
            if (/Max drop delay is:\s*(\d+)/){
                $drop_delay = ceil($1/$t2c);
            }
        }
        close $ch;
        print $fh "#ifdef $monitor\n";
        print $fh "\t#define MON_WCET $full_delay\n";
        print $fh "\t#define MON_DROP_WCET $drop_delay\n";
        print $fh "#endif\n";
        $pyfile .= "\tif monitor == '$monitor':\n";
        $pyfile .= "\t\tfull_wcet = $full_delay\n";
        $pyfile .= "\t\tdrop_wcet = $drop_delay\n";
    }
    print $fh "#endif\n";
    $pyfile .= "\n";
}

print $fh "\n#endif // __MONITOR_TIME_H__\n";

close $fh;

open $fh, ">", "$gem5/configs/example/monitor_timing.py" or die "Could not create file $gem5/configs/example/monitor_timing.py\n";
print $fh $pyfile;
close $fh; 

system ("perl $gem5/scripts/build_monitors.pl");
