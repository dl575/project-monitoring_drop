#ifdef HB_FULL
/*
 * hb_soft_drop.c
 *
 * HardBound on monitoring core.
 *
 * This version is meant to be used with the core-based
 * monitor.
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "timer.h"
#include "monitoring_wcet.h"
#include "flagcache.h"

#define METADATA_ADDRESSES 1024*1024*64

#define ISA_ARM

#ifdef ISA_ARM
  #define NUM_REGS 32
#else
  #define NUM_REGS 32
#endif

#define MONITOR "[HB] "

typedef unsigned long long int HBTag;

HBTag tagmem[METADATA_ADDRESSES];
HBTag tagrf[NUM_REGS];

#define toBoundTag(t) ((int)(t >> 32))
#define toBaseTag(t)  (t & 0xffffffff)

int main(int argc, char *argv[]) {
  register unsigned int temp;
  register unsigned int rd;
  register unsigned int rs1;
  register unsigned int rs2;
  volatile register int error;
  int opcode;
  HBTag tresult;
  HBTag trs1;
  HBTag trs2;
  HBTag setTagData;

  // Set up monitoring
  INIT_MONITOR;
  // Set up interface to flag cache
  INIT_FC;

  // Main loop, loop until main core signals done
  while(1) {

    // Grab new packet from FIFO. Block until packet available.
    POP_FIFO;

    // integer ALU
    if (READ_FIFO_INTALU) {
      opcode = READ_FIFO_OPCODE;
      if (opcode == ALUMov) {
        rs1 = READ_FIFO_RS1;
        if (rs1 < NUM_REGS) {
          if (rd < NUM_REGS) {
            rd = READ_FIFO_RD;
            tagrf[rd] = tagrf[rs1];
            FC_ARRAY_REVALIDATE(rd);
          }
        } else {
          // mov immediate
          if (rd < NUM_REGS) {
            tagrf[rd] = 0;
            FC_ARRAY_REVALIDATE(rd);
          }
        }
      } else if ((opcode == ALUAdd1) && (opcode == ALUAdd2) && (opcode == ALUSub)) {
        tresult = 0;
        rs1 = READ_FIFO_RS1;
        rs2 = READ_FIFO_RS2;
        if ((rs1 < NUM_REGS) && (rs2 <NUM_REGS)) {
          trs1 = tagrf[rs1];
          trs2 = tagrf[rs2];
          if (toBoundTag(trs1)) {
            tresult = trs1;
          } else {
            tresult = trs2;
          }
          rd = READ_FIFO_RD;
          if (rd < NUM_REGS) {
            tagrf[rd] = tresult;
            FC_ARRAY_REVALIDATE(rd);
          }
        } else if (rs1 < NUM_REGS) {
          // add immediate
          trs1 = tagrf[rs1];
          rd = READ_FIFO_RD;
          if (rd < NUM_REGS) {
            tagrf[rd] = trs1;
            FC_ARRAY_REVALIDATE(rd);
          }
        }
      } else {
        // other ALU operations
        rd = READ_FIFO_RD;
        if (rd < NUM_REGS) {
          tagrf[rd] = trs1;
          FC_ARRAY_REVALIDATE(rd);
        }
      }
    } else if (READ_FIFO_STORE) {
      if (!READ_FIFO_SETTAG) {
        rs1 = READ_FIFO_RS1;
        rs2 = READ_FIFO_RS2;
        if ((rs1 < NUM_REGS) && (rs2 < NUM_REGS)) {
          trs1 = tagrf[rs1];
          trs2 = tagrf[rs2];
          temp = READ_FIFO_MEMADDR;
          if (!((trs2 == 0) || (temp >= toBaseTag(trs2)) && (temp < toBoundTag(trs2))))
            error = 1;
          // update destination pointer tag
          if (READ_FIFO_MEMSIZE == 4) {
            tagmem[READ_FIFO_PHYSADDR >> 2] = trs1;
          }
          // revalidate memory tags
          FC_CACHE_REVALIDATE(temp >> 2);
        }
      } else {
        // settag operations
        unsigned int size = READ_FIFO_MEMSIZE;
        if (size == 0) {
          // set base address
          setTagData = READ_FIFO_DATA;
        } else if (size == 1) {
          // set bound address
          setTagData = setTagData | (((HBTag)READ_FIFO_DATA) << 32);
          tagmem[READ_FIFO_PHYSADDR >> 2] = setTagData;
          FC_CACHE_REVALIDATE(READ_FIFO_MEMADDR >> 2);
        }
      }
    } else if (READ_FIFO_LOAD) {
      rs1 = READ_FIFO_RS1;
      rs2 = READ_FIFO_RS2;
      if ((rs1 < NUM_REGS) && (rs2 < NUM_REGS)) {
        trs1 = tagrf[rs1];
        if (!((trs1 == 0) || (temp >= toBaseTag(trs1)) && (temp < toBoundTag(trs1))))
          error = 1;
        if (READ_FIFO_MEMSIZE == 4) {
          rd = READ_FIFO_RD;
          tagrf[rd] = tagmem[READ_FIFO_PHYSADDR >> 2];
          FC_ARRAY_REVALIDATE(rd);
        }
      }
    }
  } // while(1)

  return 1;
}
#endif
