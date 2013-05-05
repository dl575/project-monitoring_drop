
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

#define METADATA_ADDRESSES 0x400000

#define ISA_ARM

#ifdef ISA_ARM
  #define NUM_REGS 32
#else
  #define NUM_REGS 32
#endif

#define MONITOR "[DIFT] "

bool tagmem[METADATA_ADDRESSES];
bool tagrf[NUM_REGS];

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
  // set drop threshold in timer
  SET_THRES(MON_WCET - MON_DROP_WCET);

  // Main loop, loop until main core signals done
  while(1) {

    if (READ_SLACK > threshold) {

      //printf("pc = %x, m[%x] = %d\n", READ_FIFO_PC, READ_FIFO_MEMADDR, READ_FIFO_DATA);
      // Store
      if (temp = READ_FIFO_STORE) {
        // on store, propagate tag from RF to memory
        // printf("store\n");
//        bool settag = READ_FIFO_SETTAG;
        bool settag = false; // FIXME
        unsigned rt = READ_FIFO_RD;
        for (temp = (READ_FIFO_MEMADDR >> 2); temp <= (READ_FIFO_MEMEND >> 2); ++temp) {
          if (settag || tagrf[rt] || tagmem[temp % METADATA_ADDRESSES])
            tainted_insts++;
          tagmem[temp % METADATA_ADDRESSES] = settag ? true : (bool)tagrf[rt];
        }
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
          if (tagrf[rd] || tagmem[temp % METADATA_ADDRESSES])
            tainted_insts++;
          tag = tagrf[rd] = tagmem[temp % METADATA_ADDRESSES];
        }
          if (tag)
            printf(MONITOR "read tainted tag [0x%x], set taint r%d=%d\n", temp, rd, tagrf[rd]);
          total_insts++;
      } else if (temp = READ_FIFO_INDCTRL) {
        printf("indctrl\n");
        unsigned rs = READ_FIFO_SRCREG(0);
        // printf(MONITOR "indirect jump on r%d=%d\n", rs, tagrf[rs]);
        // on indirect jump, check tag taint
        if (tagrf[rs]) {
          printf(MONITOR "fatal : indirect jump on tainted value r%d, PC=%x\n", rs, READ_FIFO_PC);
          // return -1;
        }
      } else { // (temp = READ_FIFO_INTALU)
        // on ALU instructions, propagate tag between registers
        bool tresult = false;
        for (i = 0; i < READ_FIFO_NUMSRCREGS; ++i) {
          tresult |= tagrf[READ_FIFO_SRCREG(i)];
          // if (tagrf[READ_FIFO_SRCREG(i)])
          //   printf(MONITOR "r%d = %d\n", READ_FIFO_SRCREG(i), tagrf[READ_FIFO_SRCREG(i)]);
        }
        unsigned rd = READ_FIFO_RD;
        if (rd < 0x21) { // FIXME: remove hardcoded value
          if (tresult || tagrf[rd])
            tainted_insts++;
          if (!tagrf[rd] && tresult)
            printf(MONITOR "set taint r%d\n", rd);
          else if (tagrf[rd] && !tresult)
            printf(MONITOR "clear taint r%d\n", rd);
          tagrf[rd] = tresult;
        }
        total_insts++;
      }
    // Not enough slack
    } 
  } // while(1)

  return 1;
}
