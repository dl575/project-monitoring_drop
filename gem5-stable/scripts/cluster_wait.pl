#!/usr/bin/perl

use warnings;
use strict;

my $num = 1;

while ($num > 0){
    my $cmd = "qstat | wc";
    open my $ps, "-|", $cmd or die "Could not run qstat\n";
    my $result;
    while (<$ps>){
        chomp;
        $result = $_;
    }
    close $ps;

    $num = ($result =~ /^\s*(\d+)/)? ($1-2) : 0;
    $num = 0 if $num < 0;
    
    print "Remaining: $num\n";
    
    sleep 10;
}