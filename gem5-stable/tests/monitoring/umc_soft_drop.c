#ifdef UMC_FULL
/*
 * umc.c
 *
 * Unitialized memory check. Sets the metadata for a memory address to a 1
 * when written to. On a read, checks that the memory address is 1. If not
 * indicates an error. 
 *
 * This version is meant to be used with the soft dropping scheme running
 * a core-based (SW) monitor. It is similar to the full monitoring scheme
 * but also revalidates metadata.
 *
 * Author: Daniel Lo
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "monitoring_wcet.h"
#include "flagcache.h"

#define METADATA_ADDRESSES 1024*1024*128

// flags for whether memory was initialized
char metadata[METADATA_ADDRESSES];

int main(int argc, char *argv[]) {
  register int temp;
  volatile register int error;

  // Set up monitoring
  INIT_MONITOR;
  // Set up interface to flagcache
  INIT_FC;

  // Main loop, loop until main core signals done
  while(1) {
  
    POP_FIFO;
    // Store
    if (temp = READ_FIFO_STORE) {
        // Write metadata
        //printf("st [0x%x:0x%x]\n", READ_FIFO_MEMADDR, READ_FIFO_MEMEND);
        register int memend = (READ_FIFO_MEMEND >> 2);
        for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp){
          // We use masks to store at bit locations based on last three bits
          metadata[temp >> 3] = metadata[temp >> 3] | (1<<(temp&0x7));
          // Revalidate in flag cache
          FC_CACHE_REVALIDATE(temp);
        }
    // Load
    } else {
        //printf("ld [0x%x:0x%x]\n", READ_FIFO_MEMADDR, READ_FIFO_MEMEND);
        register int memend = (READ_FIFO_MEMEND >> 2);
        for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp){
          // We use masks to get value at bit location
          if ((metadata[temp >> 3] & (1<<(temp&0x7))) == 0) {
              error = 1;
              //printf("UMC error: pc = %x, m[0x%x] = %d\n", READ_FIFO_PC, READ_FIFO_MEMADDR, READ_FIFO_DATA);
              // Exit if UMC error
              //return 1;
          }
        }
    }

  } // while (1)

  return 1;
}

#endif
