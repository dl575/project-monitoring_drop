
/*
 * umc.c
 *
 * Unitialized memory check. Sets the metadata for a memory address to a 1
 * when written to. On a read, checks that the memory address is 1. If not
 * indicates an error. This version does not take into account slack.
 *
 * Author: Daniel Lo
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "timer.h"
#include "monitoring.h"

#define METADATA_ADDRESSES 1048576

int main(int argc, char *argv[]) {
  register int temp;
  // flags for whether memory was initialized
  bool metadata[METADATA_ADDRESSES];

  // Set up monitoring
  INIT_MONITOR

  // Main loop, loop until main core signals done
  while(1) {
    // Skip if fifo is empty
    if (!(temp = READ_FIFO_VALID))
      continue;

    // If main core has finished, exit
    if (temp = READ_FIFO_DONE) {
      printf("Finished monitoring\n");
      return 0;
    }
    
    // Store
    if (temp = READ_FIFO_STORE) {
      // Write metadata
      for (temp = (READ_FIFO_MEMADDR >> 2); temp <= (READ_FIFO_MEMEND >> 2); ++temp){
        metadata[temp % METADATA_ADDRESSES] = 1;
      }
    // Load
    } else if (READ_FIFO_NUMSRCREGS && READ_FIFO_SRCREG(0) == PCREG) {
        //Case of loading from PC relative value. We skip the check in this case.
        // printf("PC Relative Load @PC: %x\n", READ_FIFO_PC);
    } else {
      for (temp = (READ_FIFO_MEMADDR >> 2); temp <= (READ_FIFO_MEMEND >> 2); ++temp){
        if (metadata[temp % METADATA_ADDRESSES] == 0) {
            printf("UMC error @ PC: %x\n", READ_FIFO_PC);
            // Exit if UMC error
            return 1;
        }
      }
    }
  }

  return 1;
}
