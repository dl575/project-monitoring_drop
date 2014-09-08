#ifdef INSTTYPE_FULL
/*
 * insttype.c
 *
 * Instruction type counter
 * 
 * Count the different types of instructions observed
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "timer.h"
#include "monitoring_wcet.h"
#include "flagcache.h"

#define MONITOR "[INSTTYPE] "

unsigned counts[OPCODE_NUM];

int main(int argc, char *argv[]) {
  register unsigned opcode;
  register unsigned num_processed = 0;
  register unsigned i;

  // Set up monitoring
  INIT_MONITOR;
  // Set up interface to flag cache
  INIT_FC;

  // Main loop, loop until main core signals done
  while(1) {

    // Grab new packet from FIFO. Block until packet available.
    // Read opcode of packet.
    opcode = READ_POP_FIFO_OPCODE_CUSTOM;
    if ((++num_processed % 1000)==0) {
      for (i = 0; i < OPCODE_NUM; ++i){
        counts[i] = 0;
        FC_SET_ADDR(i);
        FC_SET_ARRAY_VALUE(FC_VALID_NULL);
      }
    }
    counts[opcode]++;
    
  } // while(1)

  return 1;
}
#endif