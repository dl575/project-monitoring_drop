#ifdef DIFT_RF_HWFILTER
/*
 * dift_rf_hwfilter.c
 *
 * Dynamic information flow tracking on monitoring core.
 * This version assumes that the dropping hardware is used and performs
 * filtering in addition to invalidation. Also, the MISP/Flag array stores
 * actual DIFT taints rather than invalidation flags.
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "timer.h"
#include "monitoring_wcet.h"
#include "flagcache.h"

#define METADATA_ADDRESSES 1024*1024*128

#define ISA_ARM

#ifdef ISA_ARM
  #define NUM_REGS 32
#else
  #define NUM_REGS 32
#endif

#define MONITOR "[DIFT] "

char tagmem[METADATA_ADDRESSES];
// Flag cache used for register file tags
// bool tagrf[NUM_REGS];

int main(int argc, char *argv[]) {
  register unsigned int temp;
  register unsigned int rd;
  register unsigned int rs;

  // Set up monitoring
  INIT_MONITOR;
  // Set up timer
  INIT_TIMER;
  // set up flag cache
  INIT_FC
  // set drop threshold in timer
  SET_THRES(MON_WCET - MON_DROP_WCET);

  // Main loop, loop until main core signals done
  while(1) {

    // Run full monitoring
    if (READ_SLACK_DROP == 1) {

        // Store
        if (READ_FIFO_STORE) {
            // bool settag = READ_FIFO_SETTAG;
            // register bool settag = false; // FIXME: Allow setting tag from software
            // Get source register
            rs = READ_FIFO_RS1;
            // Get tag
            FC_SET_ADDR(rs);
            register bool tag = FC_ARRAY_GET;
            // Propagate to destination memory addresses
            register unsigned int memend = (READ_FIFO_MEMEND >> 2);
            for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp) {
                if (tag) {
                  // Bit mask to set taint
                  tagmem[temp >> 3] = tagmem[temp >> 3] | (1 << (temp&0x07));
                } else {
                  // Bit mask to clear tag
                  tagmem[temp >> 3] = tagmem[temp >> 3] & ~(1 << (temp&0x07));
                }
                // Set flag cache as valid (clear invalidation)
                FC_SET_ADDR(temp)
                FC_CACHE_CLEAR
            }
        // Load
        } else if (READ_FIFO_LOAD) {
          // on load, propagate tag from memory to RF

          // Resulting tag
          register bool tresult = false;
          // Propagate from memory addresses
          register unsigned int memend = (READ_FIFO_MEMEND >> 2);
          for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp) {
            // Pull out correct bit in memory to store int tag register file
            if ((tagmem[temp >> 3]) & (1 >> (temp&0x7))) {
              // If any memory bit is tainted, then resulting tag is tainted
              tresult = true;
            }
          }

          // Get destination register
          rd = READ_FIFO_RD;
          // Set array address
          FC_SET_ADDR(rd);
          // Set taint
          if (tresult) {
            FC_ARRAY_SET;
          } else {
            FC_ARRAY_CLEAR;
          }
        // Indirect control
        } else if (READ_FIFO_INDCTRL) {
          // Untainted indirect control flow is all filtered out, must be tainted
          printf(MONITOR "fatal : indirect jump on tainted value r%d, PC=%x\n", rs, READ_FIFO_PC);
          return -1;
        // integer ALU - handled completely using filter table + invalidation hardware
        // } else { 
        } // inst type
    
    } // READ_SLACK_DROP
  } // while(1)

  return 1;
}
#endif
