
#include <stdio.h>

// Timer base address
#define TIMER_ADDR 0x30010000

// Initialization
#define INIT_TIMER_SUBTASK_START int *timer_start_subtask = (int *)TIMER_ADDR;
#define INIT_TIMER_SUBTASK_END   int *timer_end_subtask = (int *)(TIMER_ADDR + 0x1);
#define INIT_TIMER_TASK_START    int *timer_start_task = (int *)(TIMER_ADDR + 0x2);
#define INIT_TIMER_TASK_END      int *timer_end_task = (int *)(TIMER_ADDR + 0x3);
#define INIT_TIMER_READ          int *timer = (int *)TIMER_ADDR;
#define INIT_TIMER INIT_TIMER_SUBTASK_START INIT_TIMER_SUBTASK_END INIT_TIMER_TASK_START INIT_TIMER_TASK_END INIT_TIMER_READ

// start/end subtask
#define START_SUBTASK(WCET) *timer_start_subtask = WCET;
#define END_SUBTASK         *timer_end_subtask = 1;
// start/end task
#define START_TASK *timer_start_task = 1;
#define END_TASK   *timer_end_task = 1;

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

  START_TASK

  for (i = 0; i < 10; i++) {
    START_SUBTASK(20*500);
    sum += array[i];
    END_SUBTASK;

    // Print out accumulated slack
    read_timer = *timer;
    printf("slack = %d\n", read_timer);
  }

  END_TASK

  printf("%d\n", sum);

  return 0;
}
