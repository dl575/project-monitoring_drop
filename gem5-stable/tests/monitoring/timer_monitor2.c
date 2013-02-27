
/*
 * Toy program for testing lrc_drop.
 */

#include <stdio.h>

#include "timer.h"
#include "monitoring.h"

#define WCET_CYCLES 14
#define WCET 155

#ifndef WCET_SCALE
  #define WCET_SCALE 28
#endif

int __attribute__((noinline)) add(int a, int b) {
  return a + b;
}
int __attribute__((noinline)) sub(int a, int b) {
  return a - b;
}
int __attribute__((noinline)) mul(int a, int b) {
  int i = 0;
  int sum = 0;
  for (i = 0; i < b; i++) {
    sum = add(sum, a);
  }
  return sum;
}

int main(int argc, char *argv[]) {

  // initialize monitoring
  INIT_MONITOR;
  // initialize objects for timer
  INIT_TIMER;
  // initialize code and make sure it finishes (included in macro)
  //INIT_CODE;
  
  // Start monitoring
  ENABLE_MONITOR;
  
  // Initialize program variables
  register int i;
  int sum;
  int nsum;
  int prod;
  int array[10];

  START_TASK((WCET_SCALE-1)*WCET);

  START_SUBTASK(66);
  // Initialize array
  for (i = 0; i < 10; i++)
    array[i] = i + 1;
  // Initalize sum
  sum = 0;
  nsum = 0;
  prod = 1;

  for (i = 0; i < 10; i++) {
//    ENDSTART_SUBTASK(300*TICKS_PER_CYCLE);
    ENDSTART_SUBTASK(WCET_CYCLES);
    //sum += array[i];
    sum = add(sum, array[i]);
    nsum = sub(nsum, array[i]);
    prod = mul(prod, array[i]);
  }
  
  END_SUBTASK

  END_TASK(FIFO_SIZE*MON_DROP_WCET)

  // Stop monitoring
  DISABLE_MONITOR;

  // Print result: 55, -55, 3628800
  printf("%d, %d, %d\n", sum, nsum, prod);

  // main core done
  MAIN_DONE;

  return 0;
}
