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
  #define NUM_REGS 32
#else
  #define NUM_REGS 32
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
  unsigned tagAddr = physAddr;
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
  unsigned tagAddr = physAddr;
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

  // Set up monitoring
  INIT_MONITOR;

  // Main loop, loop until main core signals done
  while(1) {

    // Grab new packet from FIFO. Block until packet available.
    POP_FIFO;

    // Store
    if (READ_FIFO_STORE) {
      if (!(READ_FIFO_SETTAG)) {
        // Get source register
        rs = READ_FIFO_RS1;
        // Propagate to destination memory addresses
        temp = READ_FIFO_MEMADDR;
        writeTag(temp, tagrf[rs]);
      } else {
        // settag operation
        register int memend = READ_FIFO_MEMEND;
        for (temp = READ_FIFO_MEMADDR; temp <= memend; temp+=4) {
          writeTag(temp, 1);
        }
      }
    // Load
    } else if (READ_FIFO_LOAD) {
      // on load, propagate tag from memory to RF
      // Get destination register
      rd = READ_FIFO_RD;
      // Propagate from memory addresses
      tagrf[rd] = readTag(READ_FIFO_MEMADDR);
    // Indirect control
    } else if (READ_FIFO_INDCTRL) {
      rs = READ_FIFO_RS1;
      // on indirect jump, check tag taint
      if (tagrf[rs]) {
        // printf(MONITOR "fatal : indirect jump on tainted value r%d, PC=%x\n", rs, READ_FIFO_PC);
        // return -1;
        error = 1;
      }
    // integer ALU
    } else if (READ_FIFO_INTALU) {
      // on ALU instructions, propagate tag between registers
      // Read source tags and determine taint of destination
      unsigned int tresult = 0;
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
      if (rd < NUM_REGS) {
        tagrf[rd] = tresult;
      }
    } else if ((READ_FIFO_SETTAG) && (READ_FIFO_SYSCALLNBYTES > 0)) {
      // syscall read instruction
      for (temp = READ_FIFO_SYSCALLBUFPTR; temp < READ_FIFO_SYSCALLBUFPTR + READ_FIFO_SYSCALLNBYTES; temp+=4) {
        writeTag(temp, 1);
      }
    } // inst type

  } // while(1)

  return 1;
}
#endif
