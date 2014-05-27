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
  register int idx;
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
      // Write metadata, we don't differentiate between settag and regular operations
      //printf("st [0x%x:0x%x]\n", READ_FIFO_MEMADDR, READ_FIFO_MEMEND);
      register int memend = READ_FIFO_MEMEND;
      register int i = 0;
      for (idx = READ_FIFO_MEMADDR; idx <= memend; ++idx) {
        // We use masks to store at bit locations based on last three bits
        metadata[idx >> 3] = metadata[idx >> 3] | (1 << (idx & 0x7));
        // Only do once per word
        if ((i++%4)==0){
          // Get the four bits tags for the word
          uint8_t word_tag = (uint8_t)metadata[idx >> 3] >> (idx & 4);
          // All the bits are set
          if (word_tag == 0xF) {
            // Revalidate in invalidation RF and update FADE flag
            FC_SET_ADDR(idx >> 2);
            FC_SET_ARRAY_VALUE(2);
          // Not all the bits are set
          } else {
            // Revalidate in invalidation RF and clear FADE flag
            FC_SET_ADDR(idx >> 2);
            FC_SET_ARRAY_VALUE(0);
          }
          // Revalidate in flag cache
          // FC_CACHE_REVALIDATE(idx);
        }
      }
    // Load
    } else if (temp = READ_FIFO_LOAD) {
      idx = READ_FIFO_MEMADDR;
      if (metadata[idx >> 3] & (1 << (idx & 0x7)) == 0) {
        error = 1;
      }
    } else if ((READ_FIFO_SETTAG) && (READ_FIFO_SYSCALLNBYTES > 0)) {
      register int i = 0;
      // syscall read instruction
      for (idx = (READ_FIFO_SYSCALLBUFPTR); idx < (READ_FIFO_SYSCALLBUFPTR + READ_FIFO_SYSCALLNBYTES); ++idx) {
        // We use masks to store at bit locations based on last three bits
        metadata[idx >> 3] = metadata[idx >> 3] | (1 << (idx & 0x7));
        // Only do once per word
        if ((i++%4)==0){
          // Get the four bits tags for the word
          uint8_t word_tag = (uint8_t)metadata[idx >> 3] >> (idx & 4);
          // All the bits are set
          if (word_tag == 0xF) {
            // Revalidate in invalidation RF and update FADE flag
            FC_SET_ADDR(idx >> 2);
            FC_SET_ARRAY_VALUE(2);
          // Not all the bits are set
          } else {
            // Revalidate in invalidation RF and clear FADE flag
            FC_SET_ADDR(idx >> 2);
            FC_SET_ARRAY_VALUE(0);
          }
          // Revalidate in flag cache
          // FC_CACHE_REVALIDATE(idx);
        }
      }
    }

  } // while (1)

  return 1;
}

#endif
