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
  // Registers range from 1 to 36
  #define NUM_REGS 37
  // Exclude register 33 which is the constant zero register
  #define ZERO_REG 33
  //#define isISAReg(x) (x < NUM_REGS && x != ZERO_REG)
  #define isISAReg(x) (x != ZERO_REG)
#else
  #define NUM_REGS 32
  #define isISAReg(x) (x < NUM_REGS)
#endif

#define MONITOR "[DIFT] "

char tagmem[METADATA_ADDRESSES];
bool tagrf[NUM_REGS];


int main(int argc, char *argv[]) {
  register unsigned int temp;
  register unsigned int rd;
  register unsigned int rs;
  volatile register int error;
  register bool tresult;

  // Set up monitoring
  INIT_MONITOR;

  // Main loop, loop until main core signals done
  while(1) {

    // Grab new packet from FIFO. Block until packet available.
    POP_FIFO;

    // ALU
    if (READ_FIFO_INTALU) {
      // on ALU instructions, propagate tag between registers
      // Read source tags and determine taint of destination
      rs = READ_FIFO_RS1;
      tresult = tagrf[rs];
      rs = READ_FIFO_RS2;
      tresult |= tagrf[rs];
      // Destination register
      rd = READ_FIFO_RD;
      // Set destination taint
      tagrf[rd] = tresult;
      tagrf[ZERO_REG] = false; // zero reg should always have 0 taint
    // Load
    } else if (READ_FIFO_LOAD) {
      // on load, propagate tag from memory to RF
      // Get destination register
      rd = READ_FIFO_RD;
      // Propagate from memory addresses
      temp = READ_FIFO_MEMADDR >> 2;
      // Pull out correct bit in memory to store int tag register file
      if ((tagmem[temp >> 3]) & (1 << (temp&0x7))) {
        tagrf[rd] = true;
      } else {
        tagrf[rd] = false;
      }
    // Store
    } else if (READ_FIFO_STORE) {
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
    // Indirect control
    } else if (READ_FIFO_INDCTRL) {
      rs = READ_FIFO_RS1;
      // on indirect jump, check tag taint
      if (tagrf[rs]) {
        // printf(MONITOR "fatal : indirect jump on tainted value r%d, PC=%x\n", rs, READ_FIFO_PC);
        // return -1;
        error = 1;
      }
    } else if (READ_FIFO_SETTAG) {
      // syscall read instruction
      rs = READ_FIFO_SYSCALLBUFPTR >> 2;
      rd = (READ_FIFO_SYSCALLNBYTES + rs) >> 2;
      for (temp = rs; temp < rd; temp++) {
        tagmem[temp >> 3] = tagmem[temp >> 3] | (1 << (temp & 0x7));
      }
    } // inst type
    
  } // while(1)

  return 1;
}
#endif
