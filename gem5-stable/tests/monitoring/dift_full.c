
/*
 * dift.c
 *
 * Dynamic information flow tracking on monitoring core.
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "timer.h"
#include "monitoring.h"

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
  register int temp;
  register int rd;
  register int rs;

  // Set up monitoring
  INIT_MONITOR;
  // Set up timer
  INIT_TIMER;

  // Main loop, loop until main core signals done
  while(1) {

    // Grab new packet from FIFO. Block until packet available.
    POP_FIFO;

    // Store
    if (temp = READ_FIFO_STORE) {
//        bool settag = READ_FIFO_SETTAG;
      register bool settag = false; // FIXME: Allow setting tag from software
      // Get source register
      rd = READ_FIFO_RD;
      // Propagate to destination memory addresses
      register int memend = (READ_FIFO_MEMEND >> 2);
      for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp) {
        if (tagrf[rd] == 1) {
          // Bit mask to set taing
          tagmem[temp >> 3] = tagmem[temp >> 3] | (1 << (temp&0x07));
        } else {
          // Bit mask to clear tag
          tagmem[temp >> 3] = tagmem[temp >> 3] & ~(1 << (temp&0x07));
        }
      }
    // Load
    } else if (temp = READ_FIFO_LOAD) {
      // on load, propagate tag from memory to RF
      // Get destination register
      rd = READ_FIFO_RD;
      // Propagate from memory addresses
      register int memend = (READ_FIFO_MEMEND >> 2);
      for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp) {
        // Pull out correct bit in memory to store int tag register file
        if ((tagmem[temp >> 3]) & (1 >> (temp&0x7))) {
          tagrf[rd] = true;
        } else {
          tagrf[rd] = false;
        }
      }
    // Indirect control
    } else if (temp = READ_FIFO_INDCTRL) {
      rs = READ_FIFO_SRCREG(0);
      // on indirect jump, check tag taint
      if (tagrf[rs]) {
        printf(MONITOR "fatal : indirect jump on tainted value r%d, PC=%x\n", rs, READ_FIFO_PC);
        return -1;
      }
    // integer ALU
    } else { 
      // on ALU instructions, propagate tag between registers
      // Read source tags and determine taint of destination
      register bool tresult = false;
      for (temp = 0; temp < READ_FIFO_NUMSRCREGS; ++temp) {
        tresult |= tagrf[READ_FIFO_SRCREG(temp)];
      }
      // Destination register
      rd = READ_FIFO_RD;
      // Set destination taint
      if (rd < 0x21) { // FIXME: remove hardcoded value
        tagrf[rd] = tresult;
      }
    } // inst type
  } // while(1)

  return 1;
}
