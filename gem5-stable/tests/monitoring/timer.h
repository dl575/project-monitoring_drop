/*
 * Include file for accessing slack timer.
 *
 * Author: Daniel Lo
 */

#ifndef __SLACKTIMER_H__
#define SLACKTIMER_H

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
// read from slack timer
#define READ_SLACK *timer

#endif // SLACKTIMER_H
