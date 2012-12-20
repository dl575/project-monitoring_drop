
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "timer.h"
#include "monitoring.h"

#define TICKS_PER_CYCLE 500
// cycles needed to perform full monitoring
#define MON_WCET 58*TICKS_PER_CYCLE 
#define METADATA_ADDRESSES 65536

int main(int argc, char *argv[]) {
  register int temp;
  // flags for whether memory was initialized
  bool metadata[METADATA_ADDRESSES];

  // Set up monitoring
  INIT_MONITOR
  // set up timer interface for reading
  INIT_TIMER_READ
  int slack;

  while(1) {
    // Skip if fifo is empty
    if ((temp = READ_FIFO_VALID) == 0)
      continue;

    // If main core has finished, exit
    if (temp = READ_FIFO_DONE) {
      printf("Finished monitoring\n");
      return 0;
    }

    // Not enough slack
    if (READ_SLACK < MON_WCET) {
      // Write to prevent false positives
      metadata[(READ_FIFO_STORE >> 2) % METADATA_ADDRESSES] = 1;
      // Finish here, next loop iteration
      continue;
    }

    // Store
    if (temp = READ_FIFO_STORE) {
      // Write metadata
      metadata[(READ_FIFO_MEMADDR >> 2) % METADATA_ADDRESSES] = 1;
    // Load
    } else {
      if (metadata[(READ_FIFO_MEMADDR >> 2) % METADATA_ADDRESSES] == 0) {
        printf("UMC error: pc = %x, m[%x] = %d\n", READ_FIFO_PC, READ_FIFO_MEMADDR, READ_FIFO_DATA);
        // Exit if UMC error
        return 1;
      }
    }

  }

  return 1;
}
