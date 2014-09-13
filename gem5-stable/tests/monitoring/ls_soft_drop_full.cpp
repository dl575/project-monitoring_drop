#ifdef LS_FULL
/*
 * ls_soft_drop.c
 *
 * LockSet race detection on monitoring core.
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
#include <vector>
#include <set>

#include "timer.h"
#include "monitoring_wcet.h"
#include "flagcache.h"

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

#define MONITOR "[LS] "

/* state definitions for lockset */
#define STATE_VIRGIN    0
#define STATE_EXCLUSIVE 1
#define STATE_SHARED_RO 2
#define STATE_SHARED_RW 3

/* Operations for custom instructions */
#define CUSTOM_SETTAG 0x0
#define CUSTOM_LOCK   0x0100
#define CUSTOM_UNLOCK 0x0101

/* extract state bits from 32-bit tags */
#define EXTRACT_STATE(x)  (x & 0x3)
/* write state bits to 32-bit tags */
#define WRITE_STATE(t, s)  ((t & 0xfffffffc) | s)
/* extract thread ID from 32-bit tag */
#define EXTRACT_THREADID(x)  (x >> 2)
/* write thread ID to 32-bit tags */
#define WRITE_THREADID(t, id)  ((t & 0x3) | (id << 2))
/* bit pack thread ID and state into 32-bit tag */
#define PACK_TID_STATE(t, s)  ((t << 2) | s)
/* bit pack LockSet index and state into 32-bit tag */
#define PACK_LOCKSET_STATE(l, s)  ((l << 2) | s)


typedef unsigned long int LSTag;

void *pagetable[NUM_PAGES];

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

inline LSTag readTag(unsigned physAddr)
{
  char *page;
  unsigned tagAddr = (physAddr & 0xfffffffc);
  if (allocated(tagAddr)) {
    page = (char*)pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = (char*)allocatePage(tagAddr);
  }
  return *((LSTag*)&(page[getPageOffset(tagAddr)]));
}

inline void writeTag(unsigned physAddr, LSTag tag)
{
  char *page;
  unsigned tagAddr = (physAddr & 0xfffffffc);
  if (allocated(tagAddr)) {
    page = (char*)pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = (char*)allocatePage(tagAddr);
  }
  *((LSTag*)&(page[getPageOffset(tagAddr)])) = tag;
}

/***********************************************
 * Structures to support LockSet race detection
 ***********************************************/
// locks held by current thread
static std::set<unsigned int> thread_locks;
// LockSet index table
static std::vector<std::set<unsigned int>*> lockset_index_table;

int main(int argc, char *argv[]) {
  unsigned int temp;
  unsigned int rd;
  unsigned int rs1;
  unsigned int rs2;
  unsigned int addr;
  unsigned int memend;
  int opcode;
  LSTag tag;
  unsigned int tid;

  // Set up monitoring
  INIT_MONITOR;
  // Set up interface to flag cache
  INIT_FC;

  // Main loop, loop until main core signals done
  while(1) {

    // Grab new packet from FIFO. Block until packet available.
    opcode = READ_POP_FIFO_OPCODE_CUSTOM;

    switch (opcode) {
      // Load
      case OPCODE_LOAD:
        addr = READ_FIFO_MEMADDR;
        /* LockSet only monitors heap addresses. For stack addresses,
           metadata never gets allocated. Loads to unallocated metadata
           addresses indicate the address is on stack, thus no monitoring
           is performed. */
        if (!allocated(addr))
          break;
        tag = readTag(addr);
        switch (EXTRACT_STATE(tag)) {
          case STATE_VIRGIN:
            /* Memory location is in STATE_VIRGIN, i.e. no data have been
               written to this memory location since allocation. Technically
               loads to the memory location are uninitialized memory accesses.
               But since we are not doing UMC, we just ignore these loads. */
            break;
          case STATE_EXCLUSIVE:
            tid = READ_FIFO_THREADID;
            if (tid != EXTRACT_THREADID(tag)) {
              /* Transition from STATE_EXCLUSIVE to STATE_SHARED_RO.
                 Set C(v) to the set of locks currently held by the thread. */
              // int locks = getHeldLocks()...
              // tag = PACK_LOCKSET_STATE(locks, STATE_SHARED_RO);
              // writeTag(addr, tag);
            }
            break;
          case STATE_SHARED_RO:
            /* In STATE_SHARED_RO, C(v) is updated, but data races are not
               reported. */
            break;
          case STATE_SHARED_RW:
            // TODO: check LockSet
            break;
        }

        break;
      // Custom instruction to set tags / lock / unlock
      case OPCODE_CUSTOM_DATA:
        addr = READ_FIFO_MEMADDR;
        memend = READ_FIFO_MEMEND;
        switch (READ_FIFO_DATA) {
          case CUSTOM_SETTAG:
            for (; addr <= memend; addr+=4) {
              writeTag(addr, STATE_VIRGIN);
            }
            break;
          case CUSTOM_LOCK:
            // add addr to lockset of current thread
            thread_locks.insert(addr);
            break;
          case CUSTOM_UNLOCK:
            // remove addr from lockset of current thread
            thread_locks.erase(addr);
            break;
        }
        break;
      // Store
      case OPCODE_STORE:
        addr = READ_FIFO_MEMADDR;
        /* LockSet only monitors heap addresses. For stack addresses,
           metadata never gets allocated. Loads to unallocated metadata
           addresses indicate the address is on stack, thus no monitoring
           is performed. */
        if (!allocated(addr))
          break;
        tag = readTag(addr);
        switch (EXTRACT_STATE(tag)) {
          case STATE_VIRGIN:
            writeTag(addr, PACK_TID_STATE(READ_FIFO_THREADID, STATE_EXCLUSIVE));
            break;
          case STATE_EXCLUSIVE:
            tid = READ_FIFO_THREADID;
            if (tid != EXTRACT_THREADID(tag)) {
              /* Transition from STATE_EXCLUSIVE to STATE_SHARED_RW.
                 Set C(v) to the set of locks currently held by the thread. */
              // int locks = getHeldLocks()...
              // tag = PACK_LOCKSET_STATE(locks, STATE_SHARED_RW);
              // writeTag(addr, tag);
            }
            break;
          case STATE_SHARED_RO:
            /* Transition from STATE_SHARED_RO to STATE_SHARED_RW.
               Update C(v) and check for data races. */
            // int locks = getHeldLocks()...
            // tag = PACK_LOCKSET_STATE(locks, STATE_SHARED_RW);
            // writeTag(addr, tag);
            break;
          case STATE_SHARED_RW:
            /* update C(v) and check for data races */
            break;
        }
        break;
    } // switch

  } // while(1)

  return 1;
}
#endif
