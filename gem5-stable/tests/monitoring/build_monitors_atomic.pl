#!/usr/bin/perl

use strict;
use warnings;

my @monitors = ('umc_full', 'umc_swdrop', 'umc_hwdrop', 'umc_hwfilter', 'lrc_full', 'lrc_swdrop', 'lrc_hwdrop', 'lrc_hwfilter');

foreach my $monitor (@monitors){
    my $moncaps = uc $monitor;
    system("make TARGET=$monitor MONITOR=$moncaps MODEL=ATOMIC");
}
