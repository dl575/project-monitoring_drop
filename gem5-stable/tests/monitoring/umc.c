
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "timer.h"
#include "monitoring.h"

#define TICKS_PER_CYCLE 500

int main(int argc, char *argv[]) {
  register int i;

  // Set up monitoring
  INIT_MONITOR
  // set up timer interface for reading
  INIT_TIMER_READ
  int slack;

  // Data structure for holding monitoring data
  struct monitoring_packet fifo_data;
  // flags for whether memory was initialized
  bool metadata[1024];

  while(1) {
    // Not enough slack
    if (READ_SLACK < 40*TICKS_PER_CYCLE) {
      // Read just the PC from Fifo (reads off an entry)
      int instAddr = READ_PC;
      // If no more monitoring packets, exit
      if (instAddr == 0) {
        printf("Finished monitoring\n");
        return 0;
      } else {
        // Write to prevent false positives
        metadata[(fifo_data.memAddr >> 2) % 1024] = 1;
      }
      // No other operation, next loop iteration 
      continue;
    }
    // Read FIFO data
    READ_FIFO(fifo_data);

    // If no more monitoring packets, exit
    if (fifo_data.instAddr == 0) {
      printf("Finished monitoring\n");
      return 0;
    }

    //printf("%x : m[%08x] = %x ", fifo_data.instAddr, fifo_data.memAddr, fifo_data.data);
    // Store
    if (fifo_data.store) {
      //printf("store\n");
      // Write metadata
      metadata[(fifo_data.memAddr >> 2) % 1024] = 1;
    // Load
    } else {
      //printf("load\n");
      if (metadata[(fifo_data.memAddr >> 2) % 1024] == 0) {
        printf("UMC error\n");
        // Exit if UMC error
        return 1;
      }
    }
  }

  return 1;
}
