#ifdef UMC_SWDROP

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "timer.h"
#include "monitoring_wcet.h"

#define METADATA_ADDRESSES 1024*1024*128

// flags for whether memory was initialized
char metadata[METADATA_ADDRESSES];

int main(int argc, char *argv[]) {
  register unsigned int temp;

  // Set up monitoring
  INIT_MONITOR
  // set up timer interface for reading
  INIT_TIMER
  // set drop threshold in timer
  SET_THRES(MON_WCET - MON_DROP_WCET);

  while(1) {
    // Run full monitoring
    if (READ_SLACK_DROP == 1) {
        temp = READ_FIFO_MEMADDR;
        // Store
        if (READ_FIFO_STORE) {
          register unsigned int memend = (READ_FIFO_MEMEND >> 2);
          // Write metadata
          for (temp = (temp >> 2); temp <= memend; ++temp){
            // We use masks to store at bit locations based on last three bits
            metadata[temp >> 3] = metadata[temp >> 3] | (1<<(temp&0x7));
          }
        // Load
        } else if (READ_FIFO_LOAD) {
          register unsigned int memend = (READ_FIFO_MEMEND >> 2);
          for (temp = (temp >> 2); temp <= memend; ++temp){
            // We use masks to get value at bit location
            if (((metadata[temp >> 3] & (1<<(temp&0x7))) == 0)) {
                printf("UMC error: pc = %x, m[%x] = %d\n", READ_FIFO_PC, (temp << 2), READ_FIFO_DATA);
                // Exit if UMC error
                return 1;
            }
          }
        }
    }
    // Not enough slack, drop
    else {
        // Write to prevent false positives
        // Shift by 2 to store tag per word
        // Shift by additional 3 because we will not be bit masking
        temp = READ_FIFO_MEMADDR;
        metadata[(temp >> 5)] = 0xFF;
    }
  }

  return 1;
}

#endif
