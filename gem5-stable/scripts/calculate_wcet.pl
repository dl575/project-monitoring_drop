#!/usr/bin/perl

use warnings;
use strict;

my %wcet;
my %numwcet;
my %maxwcet;
my %maxpackets;
my %tasktype;
my @wcetorder;

while (<>){
    if (/^\d+:.*?(\w+)\s*@\s*(\w+)\s*ET\s*=\s*(\d+)(?:\s*,\s*Packets\s*=\s*(\d+))?$/){
        if (!defined $wcet{$2}){
            push @wcetorder, $2;
            $wcet{$2} = $3;
            $tasktype{$2} = $1;
            $maxwcet{$2} = $3;
            $numwcet{$2} = 1;
        }else{
            $wcet{$2} += $3;
            $maxwcet{$2} = $3 if $3 > $maxwcet{$2};
            $numwcet{$2}++;
        }
        if (defined $4){
            $maxpackets{$2} = $4 if (!defined $maxpackets{$2} or $4 > $maxpackets{$2})
        }
    }
}

foreach my $pc (@wcetorder){
    print "WCET for $tasktype{$pc} @ $pc = Max: $maxwcet{$pc}, Avg: ".($wcet{$pc}/$numwcet{$pc}).", N: $numwcet{$pc}\n";
    print "Max Packets for $tasktype{$pc} @ $pc = $maxpackets{$pc}\n" if defined $maxpackets{$pc};
}
