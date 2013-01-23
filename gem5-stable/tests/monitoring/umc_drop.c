
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "timer.h"
#include "monitoring.h"

#define METADATA_ADDRESSES 65536
#define DEBUG 1

int main(int argc, char *argv[]) {
  register int temp;
  register int drops = 0;
  register int not_drops = 0;
  // flags for whether memory was initialized
  bool metadata[METADATA_ADDRESSES];

  // Set up monitoring
  INIT_MONITOR
  // set up timer interface for reading
  INIT_TIMER
  int slack;

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

    // If there's not enough slack and the fifo's not full,
    // can just stall
    // while (READ_SLACK < MON_WCET && !READ_FIFO_FULL); -> Can cause deadlock comment out for now

    // Not enough slack, drop
    if (READ_SLACK < MON_WCET) {
      // Write to prevent false positives
      register int memend = (READ_FIFO_MEMEND >> 2);
      for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp){
        metadata[temp % METADATA_ADDRESSES] = 1;
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
    
    POP_FIFO;

  }

  return 1;
}
