#!/usr/bin/perl

use warnings;
use strict;

my %wcet;
my %tasktype;
my @wcetorder;

while (<>){
    if (/^\d+:.*?(\w+)\s*@\s*(\w+)\s*ET\s*=\s*(\d+)$/){
        if (!defined $wcet{$2}){
            push @wcetorder, $2;
            $wcet{$2} = $3;
            $tasktype{$2} = $1;
        }else{
            $wcet{$2} = $3 if $3 > $wcet{$2};
        }
    }
}

foreach my $pc (@wcetorder){
    print "WCET for $tasktype{$pc} @ $pc = $wcet{$pc}\n";
}
