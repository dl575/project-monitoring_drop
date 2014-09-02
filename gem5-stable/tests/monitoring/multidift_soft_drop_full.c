#ifdef MULTIDIFT_FULL
/*
 * multidift_soft_drop.c
 *
 * Multiple-bit Dynamic information flow tracking on monitoring core.
 * Tag size is 32-bit.
 *
 * This version is meant to be used with the core-based monitor.
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>

#include "timer.h"
#include "monitoring_wcet.h"

/* 4GB tag space, 4kB pages */
#define PAGE_OFFSET_BITS  12
#define PAGE_SIZE (1 << PAGE_OFFSET_BITS)
#define NUM_PAGES 1024*1024

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

#define MONITOR "[MULTIDIFT] "

typedef unsigned long int DIFTTag;

void *pagetable[NUM_PAGES];

DIFTTag tagrf[NUM_REGS];

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

inline DIFTTag readTag(unsigned physAddr)
{
  char *page;
  unsigned tagAddr = physAddr & 0xfffffffc;
  if (allocated(tagAddr)) {
    page = pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = allocatePage(tagAddr);
  }
  return *((DIFTTag*)&(page[getPageOffset(tagAddr)]));
}

inline void writeTag(unsigned physAddr, DIFTTag tag)
{
  char *page;
  unsigned tagAddr = physAddr & 0xfffffffc;
  if (allocated(tagAddr)) {
    page = pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = allocatePage(tagAddr);
  }
  *((DIFTTag*)&(page[getPageOffset(tagAddr)])) = tag;
}

int main(int argc, char *argv[]) {
  register unsigned int temp;
  register unsigned int rd;
  register unsigned int rs;
  volatile register int error;
  register int opcode;

  // Set up monitoring
  INIT_MONITOR;
  // zero reg tag held at 0
  tagrf[ZERO_REG] = 0;

  // Main loop, loop until main core signals done
  while(1) {

    // Grab new packet from FIFO. Block until packet available.
    // Read opcode of packet.
    opcode = READ_POP_FIFO_OPCODE_CUSTOM;

    // Based on opcode,
    switch (opcode) {
      // ALU
      case OPCODE_INTALU:
        // on ALU instructions, propagate tag between registers
        // Read source tags and determine taint of destination
        rs = READ_FIFO_RS1;
        temp = tagrf[rs];
        rs = READ_FIFO_RS2;
        temp |= tagrf[rs];
        // Destination register
        rd = READ_FIFO_RD;
        // Set destination taint
        tagrf[rd] = temp;
        tagrf[ZERO_REG] = 0; // zero reg should always have 0 taint
        break;
      // Load
      case OPCODE_LOAD:
        // on load, propagate tag from memory to RF
        // Get destination register
        rd = READ_FIFO_RD;
        // Propagate from memory addresses
        tagrf[rd] = readTag(READ_FIFO_MEMADDR);
        break;
      // Store
      case OPCODE_STORE:
        // Get source register
        rs = READ_FIFO_RS1;
        // Propagate to destination memory addresses
        temp = READ_FIFO_MEMADDR;
        writeTag(temp, tagrf[rs]);
        break;
      // Indirect control
      case OPCODE_INDCTRL:
        rs = READ_FIFO_RS1;
        // on indirect jump, check tag taint
        if (tagrf[rs]) {
          // printf(MONITOR "fatal : indirect jump on tainted value r%d, PC=%x\n", rs, READ_FIFO_PC);
          // return -1;
          error = 1;
        }
        break;
      // Syscall read
      case OPCODE_SYSCALLREAD:
        // syscall read instruction
        rs = READ_FIFO_SYSCALLBUFPTR;
        rd = READ_FIFO_SYSCALLNBYTES + rs;
        for (temp = rs; temp < rd; temp += 4) {
          writeTag(temp, 1);
        }
        break;
    } // switch

  } // while(1)

  return 1;
}
#endif
