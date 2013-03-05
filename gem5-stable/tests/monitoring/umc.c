
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

#include "monitoring.h"

#define METADATA_ADDRESSES 1024*1024*128

// flags for whether memory was initialized
char metadata[METADATA_ADDRESSES];

int main(int argc, char *argv[]) {
  register int temp;

  // Set up monitoring
  INIT_MONITOR;

  // Main loop, loop until main core signals done
  while(1) {
  
    // Store
    if (temp = READ_FIFO_STORE) {
      // Write metadata
      register int memend = (READ_FIFO_MEMEND >> 2);
      for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp){
        // We use masks to store at bit locations based on last three bits
        metadata[temp >> 3] = metadata[temp >> 3] | (1<<(temp&0x7));
      }
    // Load
    } else {
      register int memend = (READ_FIFO_MEMEND >> 2);
      for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp){
        // We use masks to get value at bit location
        if ((metadata[temp >> 3] & (1<<(temp&0x7))) == 0) {
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
