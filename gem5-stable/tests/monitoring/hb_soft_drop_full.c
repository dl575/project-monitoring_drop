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
  // Exclude register 33 which is the constant zero register
  #define ZERO_REG 33
  #define isISAReg(x) (x != ZERO_REG)
#else
  #define NUM_REGS 32
  #define isISAReg(x) (x < NUM_REGS)
#endif

#define MONITOR "[HB] "

typedef unsigned long long int HBTag;

void *pagetable[NUM_PAGES];

HBTag tagrf[NUM_REGS];

#define toBoundTag(t) ((int)(t >> 32))
#define toBaseTag(t)  (t & 0xffffffff)

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
  // keep double-word tag per memory word
  unsigned tagAddr = (physAddr & 0xfffffffc) << 1;
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
  // keep double-word tag per memory word
  unsigned tagAddr = (physAddr & 0xfffffffc) << 1;
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
  register int opcode;
  register HBTag trs1;
  register HBTag trs2;
  register HBTag setTagData;

  // Set up monitoring
  INIT_MONITOR;
  tagrf[ZERO_REG] = 0;

  // Main loop, loop until main core signals done
  while(1) {

    // Grab new packet from FIFO. Block until packet available.
    //POP_FIFO;
    opcode = READ_POP_FIFO_OPCODE_CUSTOM;

    switch (opcode) {
      // integer ALU
      // Single source
      case OPCODE_INTALU_SINGLESRC:
        rs1 = READ_FIFO_RS1;
        rd = READ_FIFO_RD;
        // Note: if rs1 is invalid (i.e., immediate) then
        //   rs1 = ZERO_REG, tagrf[rs1] = 0
        tagrf[rd] = tagrf[rs1];
        break;
      // Dual source
      case OPCODE_INTALU_DUALSRC:
        rs1 = READ_FIFO_RS1;
        rs2 = READ_FIFO_RS2;
        rd = READ_FIFO_RD;
        trs1 = tagrf[rs1];
        if (!toBoundTag(trs1)) {
          trs1 = tagrf[rs2];
        }
        tagrf[rd] = trs1;
        break;
      case OPCODE_INTALU_OTHER:
        // other ALU operations
        tagrf[rd] = 0;
        break;
      // Store: str rs1, [rs2, #c]
      case OPCODE_STORE:
        rs1 = READ_FIFO_RS1;
        rs2 = READ_FIFO_RS2;
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
        break;
      // Load
      case OPCODE_LOAD:
        // Check tag
        rs1 = READ_FIFO_RS1;
        trs1 = tagrf[rs1];
        temp = READ_FIFO_MEMADDR;
        if (!((trs1 == 0) || (temp >= toBaseTag(trs1)) && (temp < toBoundTag(trs1))))
          error = 1;
        // Propagate tag
        if (READ_FIFO_MEMSIZE == 4) {
          rd = READ_FIFO_RD;
          HBTag tmem = readTag(READ_FIFO_PHYSADDR);
          tagrf[rd] = tmem;
        }
        break;
      // Set pointer tag
      case OPCODE_CUSTOM_DATA:
        // settag operations
        temp = READ_FIFO_MEMSIZE;
        if (temp == 0) {
          // set base address
          setTagData = READ_FIFO_DATA;
        } else if (temp == 1) {
          // set bound address
          setTagData = setTagData | (((HBTag)READ_FIFO_DATA) << 32);
          writeTag(READ_FIFO_PHYSADDR, setTagData);
        }
        break;
    } // switch

  } // while(1)

  return 1;
}
#endif
