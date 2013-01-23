
/*
 * Test benchmark to find WCET of Malarden tasks. Run each benchmark
 * multiple times with slack timer enabled.
 */

#include <stdio.h>

#include "../monitoring/timer.h"
#include "../monitoring/monitoring.h"
#include "include/malarden.h"

// Number of benchmarks
#define NBENCH 3

volatile int *timer;
// Array of benchmarks
int (*benchmarks[NBENCH])() = {factorial, insertsort, fibcall};

int main(int argc, char* argv[]) {
  
  INIT_MONITOR;
  INIT_CODE;
  ENABLE_MONITOR;
  
  timer = (int *)TIMER_ADDR;
  
  int i, bench;

  for (bench = 0; bench < NBENCH; bench++) {
    for (i = 0; i < 10; i++) {
      (*benchmarks[bench])(); 
    }
  }
  
  DISABLE_MONITOR;
  MAIN_DONE;

  return 0;
}
