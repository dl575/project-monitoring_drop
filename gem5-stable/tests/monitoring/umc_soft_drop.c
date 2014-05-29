#ifdef UMC_FULL
/*
 * umc.c
 *
 * Unitialized memory check. Sets the metadata for a memory address to a 1
 * when written to. On a read, checks that the memory address is 1. If not
 * indicates an error. 
 *
 * This version is meant to be used with the soft dropping scheme running
 * a core-based (SW) monitor. It is similar to the full monitoring scheme
 * but also revalidates metadata.
 *
 * Author: Daniel Lo
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "monitoring_wcet.h"
#include "flagcache.h"

/* 512MB tag space, 4kB pages */
#define PAGE_OFFSET_BITS  12
#define PAGE_SIZE (1 << PAGE_OFFSET_BITS)
#define NUM_PAGES 1024*128

typedef unsigned char uint8_t;

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

/*
 * Read tag for a word (4 bytes).
 */
inline uint8_t readWordTag(unsigned addr) {
  uint8_t *page;
  unsigned tagAddr = addr >> 3;
  if (allocated(tagAddr)) {
    page = pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = allocatePage(tagAddr);
  }
  uint8_t octet = page[getPageOffset(tagAddr)];
  // return (addr & 0x4) ? (octet >> 4) : (octet & 0xf);
  // this might be more efficient
  return (octet >> (addr & 0x4)) & 0xf;
}

/*
 * Read tag for a half word (2 bytes).
 */
inline uint8_t readHalfWordTag(unsigned addr) {
  uint8_t *page;
  unsigned tagAddr = addr >> 3;
  if (allocated(tagAddr)) {
    page = pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = allocatePage(tagAddr);
  }
  uint8_t octet = page[getPageOffset(tagAddr)];
  return (octet >> (addr & 0x6)) & 0x3;
}

/*
 * Read tag for a byte.
 */
inline bool readByteTag(unsigned addr) {
  uint8_t *page;
  unsigned tagAddr = addr >> 3;
  if (allocated(tagAddr)) {
    page = pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = allocatePage(tagAddr);
  }
  uint8_t octet = page[getPageOffset(tagAddr)];
  return (octet >> (addr & 0x7)) & 0x1;
}

/*
 * Read tag.
 * @param size Size should be less or equal to 4.
 */
inline uint8_t readTag(unsigned addr, unsigned size) {
  uint8_t *page;
  unsigned tagAddr = addr >> 3;
  if (allocated(tagAddr)) {
    page = pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = allocatePage(tagAddr);
  }
  uint8_t octet = page[getPageOffset(tagAddr)];
  return (octet >> (addr & (0x8-size))) & ((1 << size)-1);
}

/*
 * Write tag for a word (4 bytes).
 * @param tag Tag is stored in LSB bits.
 */
inline void writeWordTag(unsigned addr, uint8_t tag) {
  uint8_t *page;
  unsigned tagAddr = addr >> 3;
  if (allocated(tagAddr)) {
    page = pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = allocatePage(tagAddr);
  }
  uint8_t octet = page[getPageOffset(tagAddr)];
  // tag to be stored
  //   tag << (addr & 0x4)
  // the other 4 bits
  //   octet & (0xf0 >> (addr & 0x4));
  page[getPageOffset(tagAddr)] = (tag << (addr & 0x4)) | (octet & (0xf0 >> (addr & 0x4)));
}

/*
 * Write tag for a half word (2 bytes).
 * @param tag Tag is stored in LSB bits.
 */
inline void writeHalfWordTag(unsigned addr, uint8_t tag) {
  uint8_t *page;
  unsigned tagAddr = addr >> 3;
  if (allocated(tagAddr)) {
    page = pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = allocatePage(tagAddr);
  }
  uint8_t octet = page[getPageOffset(tagAddr)];
  // tag to be stored
  //   tag << (addr & 0x6)
  // the other 6 bits
  //   octet & ~(0x03 << (addr & 0x6));
  page[getPageOffset(tagAddr)] = (tag << (addr & 0x6)) | (octet & ~(0x03 << (addr & 0x6)));
}

/*
 * Write tag for a byte.
 * @param tag Tag is a bool value.
 */
inline void writeByteTag(unsigned addr, bool tag) {
  uint8_t *page;
  unsigned tagAddr = addr >> 3;
  if (allocated(tagAddr)) {
    page = pagetable[getPageIndex(tagAddr)];
  } else {
    /* allocate page */
    page = allocatePage(tagAddr);
  }
  uint8_t octet = page[getPageOffset(tagAddr)];
  // tag to be stored
  //   tag << (addr & 0x7)
  // the other 6 bits
  //   octet & ~(0x01 << (addr & 0x7));
  page[getPageOffset(tagAddr)] = (tag << (addr & 0x7)) | (octet & ~(0x01 << (addr & 0x7)));
}

int main(int argc, char *argv[]) {
  register unsigned idx;
  volatile uint8_t error;

  // Set up monitoring
  INIT_MONITOR;
  // Set up interface to flagcache
  INIT_FC;

  // Main loop, loop until main core signals done
  while(1) {
  
    POP_FIFO;
    // Store
    if (READ_FIFO_STORE) {
      if (!(READ_FIFO_SETTAG)) {
        unsigned addr = READ_FIFO_MEMADDR;
        unsigned size = READ_FIFO_MEMSIZE;
        switch (size) {
          case 4:
            writeWordTag(addr, 0xf);
            FC_SET_ADDR(addr >> 2);
            FC_SET_CACHE_VALUE(2);
            break;
          case 2:
            writeHalfWordTag(addr, 0x3);
            if (addr & 0x3 == 0) {
              if (readWordTag(addr) == 0xf) {
                FC_SET_ADDR(addr >> 2);
                FC_SET_CACHE_VALUE(2);
              }
            }
            break;
          case 1:
            writeByteTag(addr, 0x1);
            if (addr & 0x3 == 0) {
              if (readWordTag(addr) == 0xf) {
                FC_SET_ADDR(addr >> 2);
                FC_SET_CACHE_VALUE(2);
              }
            }
            break;
        }
      } else {
        unsigned addr = READ_FIFO_MEMADDR;
        unsigned memend = READ_FIFO_MEMEND;
        // set tag are used for initialization of large chunks of data
        for (; addr <= memend; addr+=4) {
          writeWordTag(addr, 0xf);
          FC_SET_ADDR(addr >> 2);
          FC_SET_CACHE_VALUE(2);
        }
      }
    // Load
    } else if (READ_FIFO_LOAD) {
      unsigned addr = READ_FIFO_MEMADDR;
      unsigned size = READ_FIFO_MEMSIZE;
      switch (size) {
        case 4:
          error = readWordTag(addr);
          break;
        case 2:
          error = readHalfWordTag(addr);
          break;
        case 1:
          error = readByteTag(addr);
          break;
      }
    } else if ((READ_FIFO_SETTAG) && (READ_FIFO_SYSCALLNBYTES > 0)) {
      register int i = 0;
      // syscall read instruction
      unsigned addr = READ_FIFO_SYSCALLBUFPTR;
      unsigned memend = READ_FIFO_SYSCALLBUFPTR + READ_FIFO_SYSCALLNBYTES - 1;
      for (; addr <= memend; addr+=4) {
        writeWordTag(addr, 0xf);
        FC_SET_ADDR(addr >> 2);
        FC_SET_CACHE_VALUE(2);
      }
    }
  } // while (1)

  return 1;
}

#endif
