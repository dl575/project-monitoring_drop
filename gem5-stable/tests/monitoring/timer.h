/*
 * Include file for accessing slack timer.
 *
 * Author: Daniel Lo
 */

#ifndef __SLACKTIMER_H__
#define __SLACKTIMER_H__

#include "umc_timing.h"
#include "lrc_timing.h"

// Timer base address
#define TIMER_ADDR 0x30010000

// Initialization
volatile int *timer;
#define INIT_TIMER timer = (int *)TIMER_ADDR;

// start/end task
#define START_TASK(x)          *(timer) = x;
#define END_TASK(x)            *(timer + 1)= x;
// start/end subtask
#define START_SUBTASK(WCET)    *(timer + 2) = WCET;
#define END_SUBTASK            *(timer + 3) = 1;
// end subtask and start a new one
#define ENDSTART_SUBTASK(WCET) *(timer + 4) = WCET;
// define drop threshold for monitor
#define SET_THRES(WCET)        *(timer + 5) = WCET;

// read from slack timer
#define READ_SLACK             *(timer)
// read if drop from slack timer
#define READ_SLACK_DROP        *(timer + 1)

#endif // __SLACKTIMER_H__
