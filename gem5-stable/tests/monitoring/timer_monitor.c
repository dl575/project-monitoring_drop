
#include <stdio.h>

#include "timer.h"
#include "monitoring.h"

#define TICKS_PER_CYCLE 500
#ifndef WCET_CYCLES
  #define WCET_CYCLES 300
#endif

int main(int argc, char *argv[]) {

  // initialize monitoring
  INIT_MONITOR;
  // Start monitoring
  ENABLE_MONITOR;
  // initialize objects for timer
  INIT_TIMER;
  int read_timer;

  // Initialize program variables
  register int i;
  int sum;
  int array[10];

  START_TASK(200*TICKS_PER_CYCLE);

  START_SUBTASK(300*TICKS_PER_CYCLE);
  // Initialize array
  for (i = 0; i < 10; i++)
    array[i] = i;
  // Initalize sum
  sum = 0;

  for (i = 0; i < 10; i++) {
//    ENDSTART_SUBTASK(300*TICKS_PER_CYCLE);
    ENDSTART_SUBTASK(WCET_CYCLES*TICKS_PER_CYCLE);
    sum += array[i];
  }

  END_TASK;

  // Stop monitoring
  DISABLE_MONITOR;

  // Print result
  printf("%d\n", sum);

  // main core done
  MAIN_DONE;

  while(1);

  return 0;
}
