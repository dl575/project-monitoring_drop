#!/usr/bin/perl

#########################################################
#                                                       #
#  Displays the number of ticks the WCET monitor takes  #
#  for different number of delays                       #
#                                                       #
#  Author: Mohamed Ismail                               #            
#                                                       #
#########################################################

use warnings;
use strict;

my $gem5 = $ENV{'GEM5'};
die "Could not locate GEM5 environment. Did you source 'setup.sh'?\n" unless defined $gem5 && -d $gem5;

print <<END;
---------------------------------------------------------
Make sure you set your monitor in the dual_core.py
configuration script. This will run a program to test
the WCET for the monitor with and without dropping.
---------------------------------------------------------

END

open my $ex, "-|", "gem5.debug --debug-flags=SlackTimer,Fifo $gem5/configs/example/dual_core.py -c $gem5/tests/monitoring/wcet_program.arm --cpu-type=atomic" or die "Failed to run gem5\n";
# open my $ex, "-|", "gem5.debug --debug-flags=SlackTimer,Fifo $gem5/configs/example/dual_core.py -c $gem5/tests/monitoring/wcet_program.arm --cpu-type=timing --caches" or die "Failed to run gem5\n";

my $intask = 0;
my $stall_start;
my $stall_time = 0;

my $last_pop;
my $max_delay_full;
my $max_delay_drop;

while (<$ex>){
    if (/^(\d+).*?Check if Fifo empty: 0/){
        $stall_time = (defined $stall_start)? ($1 - $stall_start) : 0;
        undef $stall_start;
    }
    if (/^(\d+).*?Check if Fifo empty: 1/){
        $stall_start = $1;
    }
    if (/Creating custom packet/){
        undef $last_pop;
    }
    if (/Written to timer: task start/){
        $intask = 1;
        undef $last_pop;
    }
    if (/Written to timer: task end/){
        $intask = 0;
        undef $last_pop;
    }
    
    if (/^(\d+).*?Pop fifo/){
        if (!defined $last_pop){
            $last_pop = $1;
        }else{
            my $delay = $1 - $last_pop - $stall_time;
            $last_pop = $1;
            if ($intask){
                unless (defined $max_delay_drop && $max_delay_drop > $delay){
                    $max_delay_drop = $delay;
                }
            }else{
                unless (defined $max_delay_full && $max_delay_full > $delay){
                    $max_delay_full = $delay;
                }
            }
        }
        $stall_time = 0;
    }
}
close $ex;

$max_delay_full = 'n/a' unless defined $max_delay_full;
$max_delay_drop = 'n/a' unless defined $max_delay_drop;

print "\nResults\n--------\n";
print "Max full delay is: $max_delay_full\n";
print "Max drop delay is: $max_delay_drop\n";
print "\n";

