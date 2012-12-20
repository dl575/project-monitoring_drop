
#include <stdio.h>

#include "timer.h"
#include "monitoring.h"

#define TICKS_PER_CYCLE 500

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

  START_SUBTASK(350*TICKS_PER_CYCLE);
  // Initialize array
  for (i = 0; i < 10; i++)
    array[i] = i;
  // Initalize sum
  sum = 0;
  END_SUBTASK;

  for (i = 0; i < 10; i++) {
    START_SUBTASK(300*TICKS_PER_CYCLE);
    sum += array[i];
    END_SUBTASK;
  }

  END_TASK;

  // main core odne
  MAIN_DONE;
  // Stop monitoring
  DISABLE_MONITOR;

  // Print result
  printf("%d\n", sum);

  while(1);

  return 0;
}
