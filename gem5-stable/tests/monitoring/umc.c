
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

#define METADATA_ADDRESSES 1024

int main(int argc, char *argv[]) {
  register int i;

  // Set up monitoring
  INIT_MONITOR

  // Data structure for holding monitoring data
  struct monitoring_packet fifo_data;
  // flags for whether memory was initialized
  bool metadata[METADATA_ADDRESSES];

  while(1) {
    // Read FIFO data
    READ_FIFO(fifo_data);

    // If no more monitoring packets, exit
    if (fifo_data.done) {
      //printf("%d, %d, : %x : m[%08x] = %x \n", fifo_data.valid, fifo_data.done, fifo_data.instAddr, fifo_data.memAddr, fifo_data.data);
      printf("Finished monitoring\n");
      return 0;
    }

    printf("%x : m[%08x] = %x %s \n", fifo_data.instAddr, fifo_data.memAddr, fifo_data.data, (fifo_data.store ? "store" : "load"));
    // Store
    if (fifo_data.store) {
      // Write metadata
      metadata[(fifo_data.memAddr >> 2) % METADATA_ADDRESSES] = 1;
    // Load
    } else {
      if (metadata[(fifo_data.memAddr >> 2) % METADATA_ADDRESSES] == 0) {
        printf("UMC error\n");
        // Exit if UMC error
        return 1;
      }
    }
  }

  return 1;
}
