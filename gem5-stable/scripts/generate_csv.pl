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
        next unless $file =~ /^malarden_(\w+)_(\d+).log$/;
        my $bmark = $1;
        my $scale = $2;
        $data{$bmark} = {} unless defined $data{$bmark};
        $data{$bmark}->{$scale} = {};
        open my $fh, "<", "$dir/$file" or die "Could not open $dir/$file\n";
        while (<$fh>){
            if (/Drops = (\d+), Non-drops = (\d+), Filtered = (\d+), Aliased = (\d+)/){
                $data{$bmark}->{$scale}->{'Drops'} = $1;
                $data{$bmark}->{$scale}->{'Non-Drops'} = $2;
                $data{$bmark}->{$scale}->{'Filtered'} = $3;
                $data{$bmark}->{$scale}->{'Aliased'} = $4;
            }
            if (/^\s*(\w+)\s*:\s*(\d+)\s*$/){
                $data{$bmark}->{$scale}->{$1} = $2;
                $stats{$1} = 1;
            }
            if (/tick (\d+)/){
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