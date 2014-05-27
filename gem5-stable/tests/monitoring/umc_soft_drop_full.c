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

typedef unsigned char uint8_t;

#define METADATA_ADDRESSES 1024*1024*512

// flags for whether memory was initialized
char metadata[METADATA_ADDRESSES];

int main(int argc, char *argv[]) {
  register unsigned addr;
  volatile register int error;

  // Set up monitoring
  INIT_MONITOR;

  // Main loop, loop until main core signals done
  while(1) {
  
    POP_FIFO;
    // Store
    if (READ_FIFO_STORE) {
      // Write metadata, we don't differentiate between settag and regular operations
      //printf("st [0x%x:0x%x]\n", READ_FIFO_MEMADDR, READ_FIFO_MEMEND);
      register int memend = READ_FIFO_MEMEND;
      register int i = 0;
      for (addr = READ_FIFO_MEMADDR; addr <= memend; ++addr) {
        // We use masks to store at bit locations based on last three bits
        metadata[addr >> 3] = metadata[addr >> 3] | (1 << (addr & 0x7));
      }
    // Load
    } else if (READ_FIFO_LOAD) {
      addr = READ_FIFO_MEMADDR;
      if (metadata[addr >> 3] & (1 << (addr & 0x7)) == 0) {
        error = 1;
      }
    } else if ((READ_FIFO_SETTAG) && (READ_FIFO_SYSCALLNBYTES > 0)) {
      register int i = 0;
      // syscall read instruction
      for (addr = (READ_FIFO_SYSCALLBUFPTR); addr < (READ_FIFO_SYSCALLBUFPTR + READ_FIFO_SYSCALLNBYTES); ++addr) {
        // We use masks to store at bit locations based on last three bits
        metadata[addr >> 3] = metadata[addr >> 3] | (1 << (addr & 0x7));
      }
    }

  } // while (1)

  return 1;
}

#endif
