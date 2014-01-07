#ifdef DIFT_HWDROP
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
bool tagrf[NUM_REGS];

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

    // If enough slack to perform full monitoring
    if (READ_SLACK_DROP == 1) {
        
        // Store
        if (READ_FIFO_STORE) {
          // bool settag = READ_FIFO_SETTAG;
          // register bool settag = false; // FIXME: Allow setting tag from software
          // Get source register
          rs = READ_FIFO_RS1;
          // Get source tag from flag cache array
          FC_SET_ADDR(rs);
          // Check if invalid
          if (!FC_ARRAY_GET){
            // Get tag
            register bool tag = tagrf[rs];
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
          } else {
            // Propagate to destination memory addresses
            register unsigned int memend = (READ_FIFO_MEMEND >> 2);
            for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp) {
                // Set flag cache as invalid
                FC_SET_ADDR(temp)
                FC_CACHE_SET
            }
          }
        // Load
        } else if (READ_FIFO_LOAD) {
          // on load, propagate tag from memory to RF
          // Get destination register
          rd = READ_FIFO_RD;
          // Propagate from memory addresses
          register unsigned int memend = (READ_FIFO_MEMEND >> 2);
          register bool tinv = false;
          register bool tresult = false;
          for (temp = (READ_FIFO_MEMADDR >> 2); temp <= memend; ++temp) {
            // Check if the memory metadata is invalid
            FC_SET_ADDR(temp);
            register bool invalid = FC_CACHE_GET;

            if (invalid) {
              // If any tag is invalid, then result is invalid
              tinv = true;
            } else {
              // Pull out correct bit in memory to store int tag register file
              if ((tagmem[temp >> 3]) & (1 >> (temp&0x7))) {
                // If any taint is set, then resulting tag is tainted
                tresult = true;
              }
            }
          }

          // Set array address
          FC_SET_ADDR(rd);
          // Invalid metadata, mark register metadata as invalid
          if (tinv) {
              FC_ARRAY_SET;
          // Valid metadata, handle normally
          } else {
              // Set taint
              tagrf[rd] = tresult;
              // Set register as valid
              FC_ARRAY_CLEAR;
          }

        // Indirect control
        } else if (READ_FIFO_INDCTRL) {
          rs = READ_FIFO_RS1;
          // Get source register invalid
          FC_SET_ADDR(rs);
          // on indirect jump, check tag taint
          if (!FC_ARRAY_GET && tagrf[rs]){
            printf(MONITOR "fatal : indirect jump on tainted value r%d, PC=%x\n", rs, READ_FIFO_PC);
            return -1;
          }
        // integer ALU
        } else { 
          // on ALU instructions, propagate tag between registers
          // Read source tags and determine taint of destination
          register bool tresult = false;
          register bool tinv = false;
          // Get tags from rs1
          rs = READ_FIFO_RS1;
          if (rs < NUM_REGS){
            FC_SET_ADDR(rs);
            tinv |= FC_ARRAY_GET;
            tresult |= tagrf[rs];
          }
          // Get tags from rs2
          rs = READ_FIFO_RS2;
          if (rs < NUM_REGS){
            FC_SET_ADDR(rs);
            tinv |= FC_ARRAY_GET;
            tresult |= tagrf[rs];
          }
          // Propagate tags
          rd = READ_FIFO_RD;
          if (rd < NUM_REGS){
            FC_SET_ADDR(rd);
            if (tinv){
                FC_ARRAY_SET;
            } else {
                FC_ARRAY_CLEAR;
                tagrf[rd] = tresult;
            }
          }
        } // inst type
        
  
    } // READ_SLACK_DROP
    
  } // while(1)

  return 1;
}
#endif
