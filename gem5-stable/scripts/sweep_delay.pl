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
my @range = 0..3;

my $gem5 = $ENV{'GEM5'};
die "Could not locate GEM5 environment. Did you source 'setup.sh'?\n" unless defined $gem5 && -d $gem5;

foreach my $i(@range){
    open my $ex, "-|", "gem5.debug --debug-flags=Fifo $gem5/configs/example/wcet.py -c $gem5/tests/monitoring/wcet_program.arm --cpu-type=atomic --delay=$i" or die "Failed to run gem5\n";
    my $isvalid = 0;
    my $last_pop;
    my $max_delay;
    
    while (<$ex>){
        $isvalid = 1 if (/Check if Fifo empty: 0/);
        if (/Check if Fifo empty: 1/){
            $isvalid = 0;
            undef $last_pop;
        }
        
        if ($isvalid && /^(\d+).*?Pop fifo/){
            if (!defined $last_pop){
                $last_pop = $1;
            }else{
                my $delay = $1 - $last_pop;
                $last_pop = $1;
                unless (defined $max_delay && $max_delay > $delay){
                    $max_delay = $delay;
                }
            }
        }
    }
    close $ex;
    
    if (!defined $max_delay){
        print "Could not determine the delay for $i.\n";
        $delays[$i] = "n/a";
    }else{
        $delays[$i] = $max_delay;
    }
}

for my $i(@range){
    print "$i -> $delays[$i]\n";
}
