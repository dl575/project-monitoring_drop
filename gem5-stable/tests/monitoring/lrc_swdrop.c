#ifdef LRC_SWDROP
/*
 * lrc_drop.c
 *
 * Drop version of link register check. On non-drop, check that the link
 * register on a call matches the value on the return. On a call, push
 * the link register on to a stack. On a return, pop the stack and
 * check that the next instruction address matches the popped value. In
 * addition, invalidate the popped value in the stack.
 *
 * On a drop, increment and decrement the link register stack pointer,
 * but don't actually write the value (on call) or do the check (on ret).
 * The result on a call is that the pointer is incremented but the value
 * remains invalid. On the non-drop check, if the value is invalid, then
 * the check is skipped.
 *
 * Author: Daniel Lo
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "timer.h"
#include "monitoring.h"

// Stack to store link registers
int lr[256];

int main(int argc, char *argv[]) {
  register int temp;
  // Stack pointer (points to next write position, top entry is at lr_ptr-1)
  register int lr_ptr = 0;

  // Set up monitoring
  INIT_MONITOR
  // set up timer interface for reading
  INIT_TIMER
  // set drop threshold in timer
  SET_THRES(MON_WCET - MON_DROP_WCET);

  while(1) {
    // Run full monitoring
    if (READ_SLACK_DROP == 1) {
      // On call, save link register onto stack
      if (temp = READ_FIFO_CALL) {
        lr[lr_ptr] = READ_FIFO_LR;
        // Push
        lr_ptr++;
      // On return, check link register, then pop entry from stack
      } else if (temp = READ_FIFO_RET) {
        // if stored lr == 0, then it is invalid and skip check
        if ((lr[lr_ptr-1] && lr[lr_ptr-1] - 1 != READ_FIFO_NEXTPC)) {
          printf("LRC Error\n");
          return 1;
        }
        // Invalidate entry
        lr[lr_ptr-1] = 0;
        // Pop
        lr_ptr--;
      }
    }
    // Not enough slack, drop
    else {
      // On a call,
      if (temp = READ_FIFO_CALL) {
        // Increment stack pointer (entry remains invalid)
        lr_ptr++;
      // On a return,
      } else if (temp = READ_FIFO_RET) {
        // Invalidate entry and pop, no check is performed
        lr[lr_ptr-1] = 0;
        lr_ptr--;
      }
    }

  }

  return 1;
}

#endif
