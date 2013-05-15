#!/usr/bin/perl

use strict;
use warnings;

my $gem5 = $ENV{'GEM5'};
die "GEM5 path not defined" unless defined $gem5;

my @models = ('ATOMIC', 'TIMING', 'FLEX');
my @monitors = ("UMC_FULL", "UMC_SWDROP", "UMC_HWDROP", "UMC_HWFILTER", "LRC_FULL", "LRC_SWDROP", "LRC_HWDROP", "LRC_HWFILTER", "DIFT_FULL", "DIFT_SWDROP", "DIFT_HWDROP", "DIFT_HWFILTER");
my @monitor_exes = map { (lc $_).".arm" } @monitors;

foreach my $model (@models){
    foreach my $monitor (@monitors){
        my $monbin = lc $monitor;
        system("cd $gem5/tests/monitoring; make TARGET=$monbin MONITOR=$monitor MODEL=$model");
    }

    # system("cd $gem5/tests/monitoring; zip monitoring.$model.zip @monitor_exes");
    system("cd $gem5/tests/monitoring; tar -cvjf monitoring.$model.bz2 @monitor_exes");
}