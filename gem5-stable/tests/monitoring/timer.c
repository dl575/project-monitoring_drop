
#include <stdio.h>

#include "timer.h"

int main(int argc, char *argv[]) {

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

  START_TASK(200)

  for (i = 0; i < 10; i++) {
    START_SUBTASK(20);
    sum += array[i];
    END_SUBTASK;

    // Print out accumulated slack
    read_timer = READ_SLACK;
    printf("slack = %d\n", read_timer);
  }

  END_TASK

  printf("%d\n", sum);

  return 0;
}
