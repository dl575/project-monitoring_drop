
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
  INIT_MONITOR

  // Main loop, loop until main core signals done
  while(1) {
    // Skip if fifo is empty
    if ((temp = READ_FIFO_VALID) == 0)
      continue;

    // If main core has finished, exit
    if (temp = READ_FIFO_DONE) {
      printf("Finished monitoring\n");
      return 0;
    }
    
    // Store
    if (temp = READ_FIFO_STORE) {
      // Write metadata
      metadata[(READ_FIFO_STORE >> 2) % METADATA_ADDRESSES] = 1;
    // Load
    } else {
      if (metadata[(READ_FIFO_STORE >> 2) % METADATA_ADDRESSES] == 0) {
        printf("UMC error\n");
        // Exit if UMC error
        return 1;
      }
    }
  }

  return 1;
}
