#!/usr/bin/perl

use strict;
use warnings;

my $gem5 = $ENV{'GEM5'};
die "GEM5 path not defined" unless defined $gem5;

my @models = ('ATOMIC', 'TIMING');
my @monitors = ('umc_full', 'umc_swdrop', 'umc_hwdrop', 'umc_hwfilter', 'lrc_full', 'lrc_swdrop', 'lrc_hwdrop', 'lrc_hwfilter');
my @monitor_exes = map { "$_.arm" } @monitors;

foreach my $model (@models){
    foreach my $monitor (@monitors){
        my $moncaps = uc $monitor;
        system("cd $gem5/tests/monitoring; make TARGET=$monitor MONITOR=$moncaps MODEL=$model");
    }

    system("cd $gem5/tests/monitoring; zip monitoring.$model.zip @monitor_exes");
}