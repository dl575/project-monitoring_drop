
#include <stdio.h>

#include "timer.h"
#include "monitoring.h"

#define TICKS_PER_CYCLE 500

int main(int argc, char *argv[]) {

  // initialize monitoring
  INIT_MONITOR
  // Start monitoring
  ENABLE_MONITOR
  // initialize objects for timer
  INIT_TIMER
  int read_timer;

  // Initialize program variables
  register int i;
  int sum;
  int array[10];

  // Initialize array
  for (i = 0; i < 10; i++)
    array[i] = 0x10;
  // Initalize sum
  sum = 0;

  START_TASK(0)

  for (i = 0; i < 10; i++) {
    START_SUBTASK(96500 + 20*TICKS_PER_CYCLE);
    sum += array[i];
    END_SUBTASK;

    // Print out accumulated slack
//    read_timer = READ_SLACK;
//    printf("slack = %d\n", read_timer);
  }

  END_TASK

  // Stop monitoring
  DISABLE_MONITOR

  // Print result
  printf("%d\n", sum);

  while(1);

  return 0;
}
