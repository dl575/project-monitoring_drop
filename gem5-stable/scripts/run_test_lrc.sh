#!/bin/bash

#
# run_test_lrc.sh
# 
# Script to run some tests on LRC functionality. Test files are of the form
# test_lrc$1.c and are in $GEM5/tests/monitoring/. The LRC program
# assumed is $GEM5/tests/monitoring/lrc.c and configs/example/dual_core.py
# should set this accordingly.
# 
# To build all tests:
#   run_test_lrc.sh make
# To run all tests:
#   run_test_lrc.sh
# To run a specific test number $1:
#   run_test_lrc.sh $1
#
# CCFLAGS can be set in this script to use different flags when compiling the
# test programs.
# CPUTYPE can be set to define the --cpu-type flag used by gem5.
#

TESTDIR="tests/monitoring/test_lrc"
MONDIR="tests/monitoring"

CCFLAGS="-O2 -I $GEM5/$MONDIR"
CONFIGFLAGS="--cpu-type=atomic"

function makeall {
  cd $GEM5/$TESTDIR
  make TARGET=test_lrc0 CCFLAGS="$CCFLAGS"
  make TARGET=test_lrc1 CCFLAGS="$CCFLAGS"
  make TARGET=test_lrc2 CCFLAGS="$CCFLAGS"
  make TARGET=test_lrc3 CCFLAGS="$CCFLAGS"
  make TARGET=test_lrc4 CCFLAGS="$CCFLAGS"
  make TARGET=test_lrc5 CCFLAGS="$CCFLAGS"
  cd $GEM5/$MONDIR
  make TARGET=lrc
}

# Check that simulation $1 finishes with no errors
function test_finished {
  gem5.debug --redirect-stdout $GEM5/configs/example/dual_core.py \
    -c $GEM5/$TESTDIR/test_lrc$1.arm $CONFIGFLAGS
  if grep --quiet "fifo done flag received" $GEM5/m5out/simout && ! grep --quiet "LRC Error" $GEM5/m5out/simout
    then
      echo "Test $1 passed"
    else
      echo "Test $1 failed"
  fi

}

# Check that simulation $1 errors
function test_error {
  gem5.debug --redirect-stdout $GEM5/configs/example/dual_core.py \
    -c $GEM5/$TESTDIR/test_lrc$1.arm $CONFIGFLAGS
  if ! grep --quiet "fifo done flag received" $GEM5/m5out/simout && grep --quiet "LRC Error" $GEM5/m5out/simout
    then
      echo "Test $1 passed"
    else
      echo "Test $1 failed"
  fi
}

# If no arguments
if [ $# -eq 0 ]
then
  # Run all tests
  test_finished 0
  test_error 1
  test_finished 2
  test_error 3
  test_finished 4
  test_error 5
else
  # Run test based on argument
  case $1 in
    0)
      test_finished 0
      ;;
    1)
      test_error 1
      ;;
    2)
      test_finished 2
      ;;
    3)
      test_error 3
      ;;
    4)
      test_finished 4
      ;;
    5)
      test_error 5
      ;;
    make)
      makeall
      ;;
    *)
      echo "Unknown argument"
      ;;
  esac
fi
