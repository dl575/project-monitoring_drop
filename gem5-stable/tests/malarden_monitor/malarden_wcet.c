
/*
 * Test benchmark to find WCET of Malarden tasks. Run each benchmark
 * multiple times with slack timer enabled.
 */

#include <stdio.h>

#include "../monitoring/timer.h"
#include "include/malarden.h"

// Number of benchmarks
#define NBENCH 3

int *timer;
// Array of benchmarks
int (*benchmarks[NBENCH])() = {factorial, insertsort, fibcall};

int main(int argc, char* argv[]) {

  timer = (int *)TIMER_ADDR;
  
  int i, bench;

  for (bench = 0; bench < NBENCH; bench++) {
    for (i = 0; i < 10; i++) {
      (*benchmarks[bench])(); 
    }
  }

  return 0;
}
