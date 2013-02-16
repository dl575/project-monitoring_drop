
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "timer.h"
#include "monitoring.h"

#define METADATA_ADDRESSES 65536

  // flags for whether memory was initialized
  bool metadata[METADATA_ADDRESSES];

int main(int argc, char *argv[]) {
  register int temp;

  // Set up monitoring
  INIT_MONITOR
  // set up timer interface for reading
  INIT_TIMER
  // set drop threshold in timer
  SET_THRES(MON_WCET - MON_DROP_WCET);

  while(1) {
    // Check if packet is valid
    if ((temp = READ_FIFO_VALID) == 0) {
      POP_FIFO;
      continue;
    }
     
    // Run full monitoring
    if (READ_SLACK_DROP == 1) {
        // Store
        if (temp = READ_FIFO_STORE) {
          // Write metadata
          register int memend = (READ_FIFO_MEMEND >> 2);
          for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp){
            metadata[temp % METADATA_ADDRESSES] = 1;
          }
        // Load
        } else {
          register int memend = (READ_FIFO_MEMEND >> 2);
          for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp){
            if (metadata[temp % METADATA_ADDRESSES] == 0) {
                printf("UMC error: pc = %x, m[%x] = %d\n", READ_FIFO_PC, READ_FIFO_MEMADDR, READ_FIFO_DATA);
                // Exit if UMC error
                return 1;
            }
          }
        }
    }     
    // Not enough slack, drop
    else {
        // Write to prevent false positives
        register int memend = (READ_FIFO_MEMEND >> 2);
        for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp){
            metadata[temp % METADATA_ADDRESSES] = 1;
        }
    }
    
    POP_FIFO;

  }

  return 1;
}
