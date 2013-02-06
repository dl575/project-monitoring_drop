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

my @delays;
my @range = 31..32;

foreach my $i(@range){
    open my $ex, "-|", "gem5.debug --debug-flags=Task,Fifo configs/example/wcet.py -c tests/monitoring/wcet_program.arm --cpu-type=atomic --delay=$i" or die "Could not run\n";
    my $first_reschedule = 0;
    my $second_reschedule = 0;
    my $first_pop;
    my $second_pop;
    while (<$ex>){
        if (/Rescheduling/){
            $second_reschedule = 1 if $first_reschedule;
            $first_reschedule = 1;
        }
        if (/^(\d+).*?Pop fifo/){
            if ($second_reschedule){
                $second_pop = $1;
                last;
            }elsif ($first_reschedule){
                $first_pop = $1;
            }
        }
    }
    close $ex;
    $delays[$i] = $second_pop - $first_pop;
}

for my $i(@range){
    print "$i -> $delays[$i]\n";
}
