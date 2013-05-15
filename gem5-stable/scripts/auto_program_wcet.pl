#!/usr/bin/perl

use strict;
use warnings;

my $gem5 = $ENV{'GEM5'};
die "GEM5 path not defined" unless defined $gem5;

my @data = ('WCET_IS_1',
            'WCET_IS_2',
            'WCET_IS',
            'WCET_CRC_1',
            'WCET_CRC_3',
            'WCET_CRC_4',
            'WCET_CRC_5',
            'WCET_CRC_2',
            'WCET_CRC',
            'WCET_EDN_1',
            'WCET_EDN_2',
            'WCET_EDN_3',
            'WCET_EDN_4',
            'WCET_EDN_5',
            'WCET_EDN_6',
            'WCET_EDN_7',
            'WCET_EDN_8',
            'WCET_EDN_9',
            'WCET_EDN_10',
            'WCET_EDN_11',
            'WCET_EDN_12',
            'WCET_EDN_13',
            'WCET_EDN_14',
            'WCET_EDN_15',
            'WCET_EDN',
            'WCET_CMP_1',
            'WCET_CMP_2',
            'WCET_CMP_3',
            'WCET_CMP_4',
            'WCET_CMP_5',
            'WCET_CMP',
            'WCET_FIR_1',
            'WCET_FIR_2',
            'WCET_FIR_3',
            'WCET_FIR_4',
            'WCET_FIR',
            'WCET_JFDC_1',
            'WCET_JFDC_2',
            'WCET_JFDC_3',
            'WCET_JFDC_4',
            'WCET_JFDC_5',
            'WCET_JFDC',
            'WCET_NSI_1',
            'WCET_NSI_2',
            'WCET_NSI_3',
            'WCET_NSI_4',
            'WCET_NSI_5',
            'WCET_NSI_6',
            'WCET_NSI_7',
            'WCET_NSI_8',
            'WCET_NSI',
            'WCET_STA_1',
            'WCET_STA_2',
            'WCET_STA_3',
            'WCET_STA_4',
            'WCET_STA_5',
            'WCET_STA'
        );

my $output_str = "";
my $default_delay = 1000;

my @monitors = ('UMC_FULL', 'UMC_SWDROP', 'LRC_FULL', 'LRC_SWDROP', 'LRC_HWDROP', 'DIFT_FULL', 'DIFT_SWDROP');
my %defaliases = ( 'LRC_HWDROP' => '#if defined UMC_HWDROP || defined UMC_HWFILTER || defined LRC_HWDROP || defined LRC_HWFILTER || defined DIFT_HWDROP || defined DIFT_HWFILTER' );
my %drop_delays = ('UMC_FULL'=>{'ATOMIC'=>2, 'TIMING'=>11}, 
                   'UMC_SWDROP'=>{'ATOMIC'=>1, 'TIMING'=>10},
                   'LRC_FULL'=>{'ATOMIC'=>2, 'TIMING'=>11},
                   'LRC_SWDROP'=>{'ATOMIC'=>1, 'TIMING'=>2},
                   'DIFT_FULL'=>{'ATOMIC'=>2, 'TIMING'=>12},
                   'DIFT_SWDROP'=>{'ATOMIC'=>2, 'TIMING'=>10},
                   );
my @models = ('ATOMIC', 'TIMING');

foreach my $model (@models){

    # system("cd $gem5/tests/monitoring; unzip -o monitoring.$model.zip") && die "Could not unzip monitoring.$model.zip\n";
    # system("cd $gem5/tests/monitoring; tar -xvjf monitoring.$model.bz2") && die "Could not unzip monitoring.$model.bz2\n";
    $output_str .= "#ifdef $model\n";

    foreach my $monitor (@monitors){
    
        my $mon_drop_delays = (defined $drop_delays{$monitor})? $drop_delays{$monitor} : {};
        my $drop_delay = (defined $mon_drop_delays->{$model})? $mon_drop_delays->{$model} : 0;
        
        $ENV{'WCET_SCALE'} = ($drop_delay > 0)? 5 : 1;
        $ENV{'MONITOR'} = $monitor;
        $ENV{'MODEL'} = $model;

        system("cd $gem5/tests/malarden_monitor; make -B");
        my $wcet_cmd = "";
        if ($model eq 'ATOMIC'){
            $wcet_cmd = "gem5.debug --debug-flags=Task $gem5/configs/example/wcet.py -c $gem5/tests/malarden_monitor/malarden_wcet.arm --cpu-type=atomic | $gem5/scripts/calculate_wcet.pl -d $drop_delay";
        } elsif ($model eq 'TIMING'){
            $wcet_cmd = "gem5.debug --debug-flags=Task $gem5/configs/example/wcet.py -c $gem5/tests/malarden_monitor/malarden_wcet.arm --cpu-type=timing --caches | $gem5/scripts/calculate_wcet.pl -d $drop_delay";
        }
        print "$wcet_cmd\n";

        $output_str .= (defined $defaliases{$monitor})? "$defaliases{$monitor}\n" : "#ifdef $monitor\n";
        
        my $data_counter = 0;
        open my $ch, "-|", $wcet_cmd;
        while (<$ch>){
            if (/WCET for (\w+).*Max:\s*(\d+)/){
                $output_str .= "\t#define ".($data[$data_counter++])." $2".(($1 eq 'Task')? "*(WCET_SCALE-1)" : '')."\n";
            }
        }
        close $ch;
        while ($data_counter < scalar @data){
            $output_str .= "\t#define ".($data[$data_counter++])." $default_delay*(WCET_SCALE-1)\n";
        }
        $output_str .= "#endif\n";
    }

    $output_str .= "#endif\n";

}

open my $tfh, "<", "$gem5/tests/malarden_monitor/include/malarden.h.tmpl" or die "Could not open $gem5/tests/malarden_monitor/include/malarden.h.tmpl";
open my $ofh, ">", "$gem5/tests/malarden_monitor/include/malarden.h" or die "Could not write to $gem5/tests/malarden_monitor/include/malarden.h";
while (<$tfh>){
    if (/<INSERT_WCET>/){
        print $ofh $output_str;
    } else {
        print $ofh $_;
    }
}
close $ofh;
close $tfh;
