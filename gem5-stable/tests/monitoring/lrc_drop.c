
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

#define DEBUG 1

// Stack to store link registers
int lr[256];

int main(int argc, char *argv[]) {
  register int temp;
  register int drops = 0;
  register int not_drops = 0;

  register int lr_ptr = 0;

  // Set up monitoring
  INIT_MONITOR
  // set up timer interface for reading
  INIT_TIMER
  // slack needed for full monitoring
  int full_slack = MON_WCET - MON_DROP_WCET;

  while(1) {
    // Skip if fifo is empty
    if ((temp = READ_FIFO_EMPTY))
        continue;
 
    // Check if packet is valid
    if ((temp = READ_FIFO_VALID) == 0) {
      POP_FIFO;
      continue;
    }

    // If main core has finished, exit
    if (temp = READ_FIFO_DONE) {
      printf("Finished monitoring\n");
      printf("Drops = %d, Non-drops = %d\n", drops, not_drops);
      return 0;
    }

    // Not enough slack, drop
    if (READ_SLACK < full_slack) {
      // Call
      if (temp = READ_FIFO_CALL) {
        // Push pointer, lr will equal 0 (invalid)
        lr_ptr++; 
      // Return
      } else if (temp = READ_FIFO_RET) {
        // Pop pointer, no check
        lr_ptr--;
      }
      // Count number of dropped events
#ifdef DEBUG
      drops++;
#endif
      POP_FIFO;
      // Finish here, next loop iteration
      continue;
    }

    // Count number of non-dropped events
#ifdef DEBUG
    not_drops++;
#endif
    // On call, save link register onto stack
    if (temp = READ_FIFO_CALL) {
      lr[lr_ptr] = READ_FIFO_LR;
      // Push
      lr_ptr++;
    // On return, check link register, then pop entry from stack
    } else if (temp = READ_FIFO_RET) {
      // if stored lr == 0, then it is invalid and skip check
      if (lr[lr_ptr-1] && lr[lr_ptr-1] - 1 != READ_FIFO_NEXTPC) {
      //if (lr[lr_ptr-1] - 1 != READ_FIFO_NEXTPC) {
        printf("LRC Error\n");
        return 1;
      }
      // Invalidate entry
      lr[lr_ptr-1] = 0;
      // Pop
      lr_ptr--;
    }
    POP_FIFO;
  } // while(1)

  return 1;
}
