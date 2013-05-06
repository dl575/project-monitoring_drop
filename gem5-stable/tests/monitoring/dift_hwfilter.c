
/*
 * dift_hwdrop.c
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
#include "flagcache.h"

#define METADATA_ADDRESSES 0x400000

#define ISA_ARM

#ifdef ISA_ARM
  #define NUM_REGS 32
#else
  #define NUM_REGS 32
#endif

#define MONITOR "[DIFT] "

bool tagmem[METADATA_ADDRESSES];
// Flag cache used for register file tags
// bool tagrf[NUM_REGS];

int main(int argc, char *argv[]) {
  register int temp;
  // flags for whether memory was initialized
  unsigned i;
  bool tag;
  unsigned long total_insts = 0;
  unsigned long tainted_insts = 0;

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

      //printf("pc = %x, m[%x] = %d\n", READ_FIFO_PC, READ_FIFO_MEMADDR, READ_FIFO_DATA);
      // Store
      if (temp = READ_FIFO_STORE) {
        // on store, propagate tag from RF to memory
        // printf("store\n");
//        bool settag = READ_FIFO_SETTAG;
        bool settag = false; // FIXME
        unsigned rt = READ_FIFO_RD;
        // Read in from flag cache
        FC_SET_ADDR(rt);
        bool tagrf_rt = FC_ARRAY_GET;
        // For each memory address
        for (temp = (READ_FIFO_MEMADDR >> 2); temp <= (READ_FIFO_MEMEND >> 2); ++temp) {
          /*
          if (settag || tagrf[rt] || tagmem[temp % METADATA_ADDRESSES])
            tainted_insts++;
            */
          // Clear tag in flag cache to indicate valid
          FC_SET_ADDR(temp % METADATA_ADDRESSES);
          FC_CACHE_CLEAR;
          // Set tag for memory address
          bool set = settag ? true : (bool)tagrf_rt;
          tagmem[temp % METADATA_ADDRESSES] = set;
        }
        /*
        for (temp = (READ_FIFO_MEMADDR >> 2); temp <= (READ_FIFO_MEMEND >> 2); ++temp) {
          if (settag || tagrf[rt] || tagmem[temp % METADATA_ADDRESSES])
            tainted_insts++;
          tagmem[temp % METADATA_ADDRESSES] = settag ? true : (bool)tagrf[rt];
        }
        */
        if (settag)
          printf(MONITOR "set tag\n");
        // else
        //   printf(MONITOR "propagate tag from reg to mem - r%d = %d\n", rt, tagrf[rt]);
        total_insts++;
      } else if (temp = READ_FIFO_LOAD) {
        // on load, propagate tag from memory to RF
        // printf("load\n");
        unsigned rd = READ_FIFO_RD;
        // if (rd >= 0x21)
        //   printf(MONITOR "load - rd : r%d\n", rd);
        for (temp = (READ_FIFO_MEMADDR >> 2); temp <= (READ_FIFO_MEMEND >> 2); ++temp) {
          // if (!tagrf[rd] && tagmem[temp % METADATA_ADDRESSES])
          //   printf(MONITOR "set taint r%d\n", rd);
          // else if (tagrf[rd] && !tagmem[temp % METADATA_ADDRESSES])
          //   printf(MONITOR "clear taint r%d\n", rd);
          /*
          if (tagrf[rd] || tagmem[temp % METADATA_ADDRESSES])
            tainted_insts++;
            */
          //tag = tagrf[rd] = tagmem[temp % METADATA_ADDRESSES];

          // Read tag from flag cache
//          FC_SET_ADDR(temp % METADATA_ADDRESSES);
//          bool fc_tag = FC_CACHE_GET;
          // We know valid if got past filtering

          // Marked as untainted in flag cache, use untainted
//          if (fc_tag == 0) {
            // Update in memory
            tag = tagmem[temp % METADATA_ADDRESSES];
            // Write tag to destination in flag cache
            FC_SET_ADDR(rd);
            if (tag) { 
              FC_ARRAY_SET;
            } else {
              FC_ARRAY_CLEAR;
            }
          // Invalid handled by filtering
//          } else {
//          }
        }
      } else if (temp = READ_FIFO_INDCTRL) {
        /*
        printf("indctrl\n");
        unsigned rs = READ_FIFO_SRCREG(0);
        // printf(MONITOR "indirect jump on r%d=%d\n", rs, tagrf[rs]);
        // on indirect jump, check tag taint
        FC_SET_ADDR(rs);
        bool tagrf_rs = FC_ARRAY_GET;
        if (tagrf_rs) {
        */
        // Filtering handles all valid (untainted) cases
   //       printf(MONITOR "fatal : indirect jump on tainted value r%d, PC=%x\n", rs, READ_FIFO_PC);
          printf(MONITOR "fatal : indirect jump on tainted value, PC=%x\n", READ_FIFO_PC);
          // return -1;
        //}
      } else { // (temp = READ_FIFO_INTALU)
        // All handled in hardware

      }
    // Not enough slack
    } // READ_SLACK_DROP
  } // while(1)

  return 1;
}
