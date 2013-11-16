#!/usr/bin/perl

use warnings;
use strict;

my @stdstats = ('Drops', 'Non-Drops', 'Filtered', 'Aliased', 'Exec-Time');

foreach my $dir (@ARGV) {
    
    print "$dir\n";
    
    opendir my $dh, $dir or die "Could not open $dir.\n";
    my @files = readdir $dh;
    closedir $dh;

    my %data;
    my %stats;
    foreach my $file (@files) {
        next unless $file =~ /^malarden_(\w+)_(\d+).txt$/;
        my $bmark = $1;
        my $scale = $2;
        $data{$bmark} = {} unless defined $data{$bmark};
        $data{$bmark}->{$scale} = {};
        open my $fh, "<", "$dir/$file" or die "Could not open $dir/$file\n";
        while (<$fh>){
            if (/system.flagcache.num_aliased\s+(\d+)/){
                $data{$bmark}->{$scale}->{'Aliased'} = $1;
            }
            if (/system.cpu1.drops::total\s+(\d+)/){
                $data{$bmark}->{$scale}->{'Drops'} = $1;
            } elsif (/system.cpu1.filtered::total\s+(\d+)/){
                $data{$bmark}->{$scale}->{'Filtered'} = $1;
            } elsif (/system.cpu1.non_drops::total\s+(\d+)/){
                $data{$bmark}->{$scale}->{'Non-Drops'} = $1;
            } elsif (/system.cpu1.drops::(\w+)\s+(\d+)/){
                my $stat_name = 'Drop_'.$1;
                $data{$bmark}->{$scale}->{$stat_name} = $2;
                $stats{$stat_name} = 1;
            } elsif (/system.cpu1.filtered::(\w+)\s+(\d+)/){
                my $stat_name = 'Filter_'.$1;
                $data{$bmark}->{$scale}->{$stat_name} = $2;
                $stats{$stat_name} = 1;
            }
            if (/final_tick\s+(\d+)/){
                $data{$bmark}->{$scale}->{'Exec-Time'} = $1;
            }
        }
        close $fh;
    }

    foreach my $bmark (keys %data) {
        open my $fh, ">", "$dir/$bmark.csv" or die "Could not create $dir/$bmark.csv";
        my @scales = (sort {$a <=> $b} keys %{$data{$bmark}});
        print $fh ",".(join ',', @scales)."\n";
        foreach my $stat (@stdstats, sort keys %stats){
            print $fh $stat;
            foreach my $scale (@scales){
                print $fh ",";
                print $fh $data{$bmark}->{$scale}->{$stat} if defined $data{$bmark}->{$scale}->{$stat};
            }
            print $fh "\n";
        }

        close $fh;
    }

}