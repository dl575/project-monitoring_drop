#ifdef DIFT_FULL
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
  register unsigned int temp;
  register unsigned int rd;
  register unsigned int rs;

  // Set up monitoring
  INIT_MONITOR;
  // Set up timer
  INIT_TIMER;

  // Main loop, loop until main core signals done
  while(1) {

    // Grab new packet from FIFO. Block until packet available.
    POP_FIFO;

    // Store
    if (READ_FIFO_STORE) {
      // bool settag = READ_FIFO_SETTAG;
      // register bool settag = false; // FIXME: Allow setting tag from software
      // Get source register
      rs = READ_FIFO_RS1;
      // Propagate to destination memory addresses
      register unsigned int memend = (READ_FIFO_MEMEND >> 2);
      for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp) {
        if (tagrf[rs] == 1) {
          // Bit mask to set taint
          tagmem[temp >> 3] = tagmem[temp >> 3] | (1 << (temp&0x07));
        } else {
          // Bit mask to clear tag
          tagmem[temp >> 3] = tagmem[temp >> 3] & ~(1 << (temp&0x07));
        }
      }
    // Load
    } else if (READ_FIFO_LOAD) {
      // on load, propagate tag from memory to RF
      // Get destination register
      rd = READ_FIFO_RD;
      // Propagate from memory addresses
      register unsigned int memend = (READ_FIFO_MEMEND >> 2);
      register bool tresult = false;
      for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp) {
        // Pull out correct bit in memory to store int tag register file
        if ((tagmem[temp >> 3]) & (1 >> (temp&0x7))) {
          tresult = true;
        } 
      }
      tagrf[rd] = tresult;
    // Indirect control
    } else if (READ_FIFO_INDCTRL) {
      rs = READ_FIFO_RS1;
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
      if (rd < NUM_REGS) { // FIXME: remove hardcoded value
        tagrf[rd] = tresult;
      }
    } // inst type
    
  } // while(1)

  return 1;
}
#endif
