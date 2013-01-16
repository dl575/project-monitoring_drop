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
#define INIT_TIMER volatile int *timer = (int *)TIMER_ADDR;

// start/end task
#define START_TASK(x)          *(timer) = x;
#define END_TASK(x)            *(timer + 1)= x;
// start/end subtask
#define START_SUBTASK(WCET)    *(timer + 2) = WCET;
#define END_SUBTASK            *(timer + 3) = 1;
// end subtask and start a new one
#define ENDSTART_SUBTASK(WCET) *(timer + 4) = WCET;
// read from slack timer
#define READ_SLACK             *timer

#ifndef TICKS_PER_CYCLE
  #define TICKS_PER_CYCLE 500
#endif

#define CYCLES(x) x*TICKS_PER_CYCLE

#endif // SLACKTIMER_H
