#ifdef LRC_FULL
/*
 * lrc.c
 *
 * Link register check. Check that the link register on a call
 * matches the value on the return.
 *
 * Author: Daniel Lo
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "monitoring.h"

// Stack to store link registers
int lr[256];

int main(int argc, char *argv[]) {
  register int temp;
  register int lr_ptr = 0;

  // Set up monitoring
  INIT_MONITOR;

  // Main loop, loop until main core signals done
  while(1) {
    
    POP_FIFO;
    
#ifdef DEBUG
    printf("pc = %x, ccr=(%d, %d, %d), lr = %x, nextpc = %x\n", READ_FIFO_PC, 
        READ_FIFO_CONTROL, READ_FIFO_CALL, READ_FIFO_RET, READ_FIFO_LR,
        READ_FIFO_NEXTPC);
#endif

    // On call, save link register onto stack
    if (temp = READ_FIFO_CALL) {
      lr[lr_ptr] = READ_FIFO_LR;
#ifdef DEBUG
      printf("push lr[%d]: %x\n", lr_ptr, lr[lr_ptr]);
#endif
      lr_ptr++;
    // On return, check link register, then pop entry from stack
    } else if (temp = READ_FIFO_RET) {
      if (lr[lr_ptr-1] - 1 != READ_FIFO_NEXTPC) {
        printf("LRC Error\n");
        return 1;
      }
      lr_ptr--;
#ifdef DEBUG
      printf("pop, now lr[%d]: %x\n", lr_ptr, lr[lr_ptr]);
#endif
    }

  } // while(1)

  return 1;
}

#endif
