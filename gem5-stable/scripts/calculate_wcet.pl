#!/usr/bin/perl

use warnings;
use strict;
use Getopt::Long;

my $mon_time = 0;

GetOptions( "drop_time|d=i" => \$mon_time );

my %wcet;
my %numwcet;
my %maxwcet;
my %maxpackets;
my %tasktype;
my @wcetorder;

while (<>){
    if (/^\d+:.*?(\w+)\s*@\s*(\w+)\s*ET\s*=\s*(\d+)(?:\s*,\s*Packets\s*=\s*(\d+))?$/){
        my $num_packets = $4 or 0;
        my $wcet_time = $3 + $num_packets*$mon_time;
        if (!defined $wcet{$2}){
            push @wcetorder, $2;
            $wcet{$2} = $wcet_time;
            $tasktype{$2} = $1;
            $maxwcet{$2} = $wcet_time;
            $numwcet{$2} = 1;
        }else{
            $wcet{$2} += $wcet_time;
            $maxwcet{$2} = $wcet_time if $wcet_time > $maxwcet{$2};
            $numwcet{$2}++;
        }
        if (defined $4){
            $maxpackets{$2} = $4 if (!defined $maxpackets{$2} or $4 > $maxpackets{$2})
        }
    }
}

foreach my $pc (@wcetorder){
    print "WCET for $tasktype{$pc} @ $pc = Max: $maxwcet{$pc}, Avg: ".($wcet{$pc}/$numwcet{$pc}).", N: $numwcet{$pc}".(($maxpackets{$pc})? ", MP: $maxpackets{$pc}" : "")."\n";
}
