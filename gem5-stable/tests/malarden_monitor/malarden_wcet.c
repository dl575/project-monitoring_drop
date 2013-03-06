
/*
 * Test benchmark to find WCET of Malarden tasks. Run each benchmark
 * multiple times with slack timer enabled.
 */

#include <stdio.h>

#include "../monitoring/timer.h"
#include "../monitoring/monitoring.h"
#include "include/malarden.h"

// Number of benchmarks
#define NBENCH 8

// Array of benchmarks
int (*benchmarks[NBENCH])() = {insertsort, crc, edn, compress, fir, jfdc, nsichneu, statemate};

int main(int argc, char* argv[]) {
  
  INIT_MONITOR;
  INIT_CODE;
  INIT_TIMER;
  
  ENABLE_MONITOR;
  
  register int result = 0;
  int i, bench;

  for (bench = 0; bench < NBENCH; bench++) {
    for (i = 0; i < 10; i++) {
      result += (*benchmarks[bench])(); 
    }
  }
  
  DISABLE_MONITOR;
  
  printf("Result is: %d\n", result);
  
  MAIN_DONE;

  return 0;
}
