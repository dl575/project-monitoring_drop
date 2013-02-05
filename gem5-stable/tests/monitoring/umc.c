
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

#define METADATA_ADDRESSES 65536

int main(int argc, char *argv[]) {
  register int temp;
  // flags for whether memory was initialized
  bool metadata[METADATA_ADDRESSES];

  // Set up monitoring
  INIT_MONITOR;

  // Main loop, loop until main core signals done
  while(1) {
    // Skip if fifo is empty
    if ((temp = READ_FIFO_EMPTY))
        continue;

    // Skip if fifo is empty
    if ((temp = READ_FIFO_VALID) == 0) {
      POP_FIFO;
      continue;
    }

    // If main core has finished, exit
    if (temp = READ_FIFO_DONE) {
      printf("Finished monitoring\n");
      return 0;
    }
    
    printf("pc = %x, m[%x:%x] = %d %s\n", READ_FIFO_PC, READ_FIFO_MEMADDR, READ_FIFO_MEMEND, READ_FIFO_DATA, READ_FIFO_STORE ? "store" : "load");
    // Store
    if (temp = READ_FIFO_STORE) {
      // Write metadata
      register int memend = (READ_FIFO_MEMEND >> 2) % METADATA_ADDRESSES;
      for (temp = (READ_FIFO_MEMADDR >> 2) % METADATA_ADDRESSES; temp <= memend; ++temp){
        metadata[temp % METADATA_ADDRESSES] = 1;
      }
    // Load
    } else {
      register int memend = (READ_FIFO_MEMEND >> 2) % METADATA_ADDRESSES;
      for (temp = (READ_FIFO_MEMADDR >> 2) % METADATA_ADDRESSES; temp <= memend; ++temp){
        if (metadata[temp % METADATA_ADDRESSES] == 0) {
            printf("UMC error: pc = %x, m[%x] = %d\n", READ_FIFO_PC, READ_FIFO_MEMADDR, READ_FIFO_DATA);
            // Exit if UMC error
            return 1;
        }
      }
    }
    // Next entry
    POP_FIFO;

  } // while (1)

  return 1;
}
