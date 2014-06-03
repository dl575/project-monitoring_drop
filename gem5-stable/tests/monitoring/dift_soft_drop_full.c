#ifdef DIFT_FULL
/*
 * dift.c
 *
 * Dynamic information flow tracking on monitoring core.
 * 
 * This version is meant to be used with the core-based
 * monitor. It is similar to dift_full.c but also
 * performs revalidation of metadata.
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "timer.h"
#include "monitoring_wcet.h"

#define METADATA_ADDRESSES 1024*1024*128

#define ISA_ARM

#ifdef ISA_ARM
  #define NUM_REGS 32
#else
  #define NUM_REGS 32
#endif

#define MONITOR "[DIFT] "

char tagmem[METADATA_ADDRESSES];
bool tagrf[NUM_REGS];

int main(int argc, char *argv[]) {
  register unsigned int temp;
  register unsigned int rd;
  register unsigned int rs;
  volatile register int error;

  // Set up monitoring
  INIT_MONITOR;

  // Main loop, loop until main core signals done
  while(1) {

    // Grab new packet from FIFO. Block until packet available.
    POP_FIFO;

    // Store
    if (READ_FIFO_STORE) {
      if (!(READ_FIFO_SETTAG)) {
        // Get source register
        rs = READ_FIFO_RS1;
        // Propagate to destination memory addresses
        temp = (READ_FIFO_MEMADDR >> 2);
        if (tagrf[rs] == 1) {
          // Bit mask to set taint
          tagmem[temp >> 3] = tagmem[temp >> 3] | (1 << (temp&0x07));
        } else {
          // Bit mask to clear tag
          tagmem[temp >> 3] = tagmem[temp >> 3] & ~(1 << (temp&0x07));
        }
      } else {
        // settag operation
        register int memend = (READ_FIFO_MEMEND >> 2);
        for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp) {
          // Bit mask to set taint
          tagmem[temp >> 3] = tagmem[temp >> 3] | (1 << (temp&0x07));
        }
      }
    // Load
    } else if (READ_FIFO_LOAD) {
      // on load, propagate tag from memory to RF
      // Get destination register
      rd = READ_FIFO_RD;
      // Propagate from memory addresses
      register bool tresult = false;
      temp = READ_FIFO_MEMADDR >> 2;
      // Pull out correct bit in memory to store int tag register file
      if ((tagmem[temp >> 3]) & (1 << (temp&0x7))) {
        tresult = true;
      }
      tagrf[rd] = tresult;
    // Indirect control
    } else if (READ_FIFO_INDCTRL) {
      rs = READ_FIFO_RS1;
      // on indirect jump, check tag taint
      if (tagrf[rs]) {
        // printf(MONITOR "fatal : indirect jump on tainted value r%d, PC=%x\n", rs, READ_FIFO_PC);
        // return -1;
        error = 1;
      }
    // integer ALU
    } else if (READ_FIFO_INTALU) {
      // on ALU instructions, propagate tag between registers
      // Read source tags and determine taint of destination
      register bool tresult = false;
      rs = READ_FIFO_RS1;
      if (rs < NUM_REGS){
        tresult |= tagrf[rs];
      }
      rs = READ_FIFO_RS2;
      if (rs < NUM_REGS){
        tresult |= tagrf[rs];
      }
      // Destination register
      rd = READ_FIFO_RD;
      // Set destination taint
      if (rd < NUM_REGS) {
        tagrf[rd] = tresult;
      }
    } else if ((READ_FIFO_SETTAG) && (READ_FIFO_SYSCALLNBYTES > 0)) {
      // syscall read instruction
      for (temp = (READ_FIFO_SYSCALLBUFPTR >> 2); temp < (READ_FIFO_SYSCALLBUFPTR + READ_FIFO_SYSCALLNBYTES) >> 2; ++temp) {
        // We use masks to store at bit locations based on last three bits
        tagmem[temp >> 3] = tagmem[temp >> 3] | (1 << (temp & 0x7));
      }
    } // inst type
    
  } // while(1)

  return 1;
}
#endif
