
#include <stdio.h>

#include "timer.h"
#include "monitoring.h"

#define WCET_CYCLES 400

#ifndef WCET_SCALE
  #define WCET_SCALE 1
#endif

int main(int argc, char *argv[]) {

  // initialize monitoring
  INIT_MONITOR;
  // initialize objects for timer
  INIT_TIMER;
  
  // initialize code and make sure it finishes
  START_TASK(CYCLES(0));
  INIT_CODE;
  END_TASK(CYCLES(400));
  
  // Start monitoring
  ENABLE_MONITOR;
  
  // Initialize program variables
  register int i;
  int sum;
  int array[10];

  START_TASK(CYCLES(0));

  START_SUBTASK(CYCLES(WCET_SCALE*400));
  // Initialize array
  for (i = 0; i < 10; i++)
    array[i] = i;
  // Initalize sum
  sum = 0;

  for (i = 0; i < 10; i++) {
//    ENDSTART_SUBTASK(300*TICKS_PER_CYCLE);
    ENDSTART_SUBTASK(CYCLES(WCET_SCALE*WCET_CYCLES));
    sum += array[i];
  }
  
  END_SUBTASK

  END_TASK(CYCLES(160))

  // Stop monitoring
  DISABLE_MONITOR;

  // Print result
  printf("%d\n", sum);

  // main core done
  MAIN_DONE;

  return 0;
}
