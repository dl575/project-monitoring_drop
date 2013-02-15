
/*
 * umc.c
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

int main(int argc, char *argv[]) {
  register int temp;
  register int lr;

  // Set up monitoring
  INIT_MONITOR;

  // Main loop, loop until main core signals done
  while(1) {
    //printf("pc = %x, ccr=(%d, %d, %d), lr = %x\n", READ_FIFO_PC, READ_FIFO_CONTROL, 
    //    READ_FIFO_CALL, READ_FIFO_RET, READ_FIFO_LR);

    // On call, save link register
    if (temp = READ_FIFO_CALL) {
      lr = READ_FIFO_LR;
    // On return, check link register
    } else if (temp = READ_FIFO_RET) {
      if (lr != READ_FIFO_LR) {
        printf("LRC Error\n");
        return 1;
      }
    }

    // Exit if done
    if (temp = READ_FIFO_DONE) {
      printf("Finished monitoring\n");
      return 0;
    }
    // Next entry
    POP_FIFO;
  } // while(1)

  return 1;
}
