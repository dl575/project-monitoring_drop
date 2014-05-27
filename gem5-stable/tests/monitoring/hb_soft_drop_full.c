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
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>

#include "timer.h"
#include "monitoring_wcet.h"

/* 1GB tag space, 4kB pages */
#define PAGE_OFFSET_BITS  12
#define PAGE_SIZE (1 << PAGE_OFFSET_BITS)
#define NUM_PAGES 256*1024

#define ISA_ARM

#ifdef ISA_ARM
  // Registers range from 1 to 36
  #define NUM_REGS 37
#else
  #define NUM_REGS 37
#endif

#define MONITOR "[HB] "

typedef unsigned long long int HBTag;

void *pagetable[NUM_PAGES];

HBTag tagrf[NUM_REGS];

#define toBoundTag(t) ((int)(t >> 32))
#define toBaseTag(t)  (t & 0xffffffff)

// Exclude register 33 which is the constant zero register
#define ZERO_REG 33
#define isISAReg(x) (x < NUM_REGS && x != ZERO_REG)

/**
 * Convert address to page index
 */
inline unsigned getPageIndex(unsigned addr) {
  return addr >> PAGE_OFFSET_BITS;
}

/**
 * Convert address to page offset
 */
inline unsigned getPageOffset(unsigned addr) {
  return addr & (PAGE_SIZE - 1);
}

/**
 * Whether an address in allocated in page table
 */
inline bool allocated(unsigned addr) {
  return pagetable[getPageIndex(addr)] != NULL;
}

/**
 * Allocate a page
 */
void* allocatePage(unsigned addr)
{
  void *page = malloc(PAGE_SIZE);
  if (page == NULL) {
    printf("Failed to allocate page for tag memory!\n");
    return NULL;
  }
  /* clear page */
  memset(page, 0, PAGE_SIZE);
  pagetable[getPageIndex(addr)] = page;
  return page;
}

inline HBTag readTag(unsigned physAddr)
{
  char *page;
  unsigned tagAddr = physAddr << 1;
  if (allocated(tagAddr)) {
    page = pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = allocatePage(tagAddr);
  }
  return *((HBTag*)&(page[getPageOffset(tagAddr)]));
}

inline void writeTag(unsigned physAddr, HBTag tag)
{
  char *page;
  unsigned tagAddr = physAddr << 1;
  if (allocated(tagAddr)) {
    page = pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = allocatePage(tagAddr);
  }
  *((HBTag*)&(page[getPageOffset(tagAddr)])) = tag;
}

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

  // Main loop, loop until main core signals done
  while(1) {

    // Grab new packet from FIFO. Block until packet available.
    POP_FIFO;

    // integer ALU
    if (READ_FIFO_INTALU) {
      opcode = READ_FIFO_OPCODE;
      if (opcode == ALUMov) {
        rs1 = READ_FIFO_RS1;
        if (isISAReg(rs1)) {
          rd = READ_FIFO_RD;
          if (isISAReg(rd)) {
            tagrf[rd] = tagrf[rs1];
          }
        } else {
          rd = READ_FIFO_RD;
          // mov immediate
          if (isISAReg(rd)) {
            tagrf[rd] = 0;
          }
        }
      } else if ((opcode == ALUAdd1) && (opcode == ALUAdd2) && (opcode == ALUSub)) {
        tresult = 0;
        rs1 = READ_FIFO_RS1;
        rs2 = READ_FIFO_RS2;
        if (isISAReg(rs1) && isISAReg(rs2)) {
          trs1 = tagrf[rs1];
          trs2 = tagrf[rs2];
          if (toBoundTag(trs1)) {
            tresult = trs1;
          } else {
            tresult = trs2;
          }
          rd = READ_FIFO_RD;
          if (isISAreg(rd)) {
            tagrf[rd] = tresult;
          }
        } else if isISAReg(rs1) {
          // add immediate
          trs1 = tagrf[rs1];
          rd = READ_FIFO_RD;
          if (isISAReg(rd)) {
            tagrf[rd] = trs1;
          }
        }
      } else {
        // other ALU operations
        rd = READ_FIFO_RD;
        if (isISAReg(rd)) {
          tagrf[rd] = trs1;
        }
      }
    } else if (READ_FIFO_STORE) {
      if (!READ_FIFO_SETTAG) {
        rs1 = READ_FIFO_RS1;
        rs2 = READ_FIFO_RS2;
        if (isISAReg(rs1) && isISAReg(rs2)) {
          trs1 = tagrf[rs1];
          trs2 = tagrf[rs2];
          temp = READ_FIFO_MEMADDR;
          if (!((trs2 == 0) || (temp >= toBaseTag(trs2)) && (temp < toBoundTag(trs2)))) {
            error = 1;
          }
          // update destination pointer tag
          if (READ_FIFO_MEMSIZE == 4) {
            writeTag(READ_FIFO_PHYSADDR, trs1);
          }
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
          writeTag(READ_FIFO_PHYSADDR, setTagData);
        }
      }
    } else if (READ_FIFO_LOAD) {
      rs1 = READ_FIFO_RS1;
      rs2 = READ_FIFO_RS2;
      if (isISAReg(rs1) && !isISAReg(rs2)) {
        trs1 = tagrf[rs1];
        if (!((trs1 == 0) || (temp >= toBaseTag(trs1)) && (temp < toBoundTag(trs1))))
          error = 1;
        if (READ_FIFO_MEMSIZE == 4) {
          rd = READ_FIFO_RD;
          tagrf[rd] = readTag(READ_FIFO_PHYSADDR);
        }
      } else {
        // should not reach here
        // if we reach here for some reason, conservatively clear dest reg tag
        rd = READ_FIFO_RD;
        if (isISAReg(rd)) {
            tagrf[rd] = 0;
        }
      }
    }
  } // while(1)

  return 1;
}
#endif
