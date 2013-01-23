
#include <stdio.h>

#include "timer.h"
#include "monitoring.h"

#define WCET_CYCLES 12

#ifndef WCET_SCALE
  #define WCET_SCALE 1
#endif

int main(int argc, char *argv[]) {

  // initialize monitoring
  INIT_MONITOR;
  // initialize objects for timer
  INIT_TIMER;
  // initialize code and make sure it finishes (included in macro)
  INIT_CODE;
  
  // Start monitoring
  ENABLE_MONITOR;
  
  // Initialize program variables
  register int i;
  int sum;
  int array[10];

  START_TASK(0);

  START_SUBTASK(WCET_SCALE*66);
  // Initialize array
  for (i = 0; i < 10; i++)
    array[i] = i;
  // Initalize sum
  sum = 0;

  for (i = 0; i < 10; i++) {
//    ENDSTART_SUBTASK(300*TICKS_PER_CYCLE);
    ENDSTART_SUBTASK(WCET_SCALE*WCET_CYCLES);
    sum += array[i];
  }
  
  END_SUBTASK

  END_TASK(FIFO_SIZE*MON_DROP_WCET)

  // Stop monitoring
  DISABLE_MONITOR;

  // Print result
  printf("%d\n", sum);

  // main core done
  MAIN_DONE;

  return 0;
}
