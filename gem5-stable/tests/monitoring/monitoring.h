/*
 * Include file for interface for reading from monitoring fifo.
 *
 * Author: Daniel Lo
 */

#ifndef __MONITORING_H__
#define __MONITORING_H__

#define MAXNUMSRCREGS 27
#define PCREG 0xf

#define ISA_PAGE_SIZE 4096

// Address of monitoring fifo (for read)
#define MONITOR_ADDR 0x30000000

// Fifo Size
#define FIFO_SIZE 16

// initialize fifo so it can be enabled/disabled
#define INIT_MONITOR register volatile unsigned int *fifo = (unsigned int *)MONITOR_ADDR; 
// enable monitoring
#define ENABLE_MONITOR *fifo = 1;
// disable monitoring
#define DISABLE_MONITOR *fifo = 0;
// main core has finished
#define MAIN_DONE *fifo = 2; while(1);

// make custom FIFO packets
#define WRITE_FIFO_START(x) *(fifo + 1) = x;
#define WRITE_FIFO_END(x) *(fifo + 2) = x;
#define WRITE_FIFO_RANGE(x1,x2) { *(fifo + 1) = x1; *(fifo + 2) = x2; }
// This is the correct implementation that prevents compiler optimizations, but the above one will generate less packets.
// #define WRITE_FIFO_RANGE(x1,x2) { unsigned int tmp[2] = { x1, x2 }; memcpy( (void *)(fifo + 1), tmp, 2*sizeof(int) ); }

// bss initialization
#define INIT_BSS extern void * __bss_start__; \
  extern void * __bss_end__; \
  WRITE_FIFO_RANGE((unsigned int)&__bss_start__, (unsigned int)&__bss_end__) \
  while (!READ_FIFO_EMPTY);

// code region initializatoin
#define INIT_CODE extern void * _init , * _end; \
  WRITE_FIFO_RANGE((unsigned int)&_init, (unsigned int) &_end) \
  while (!READ_FIFO_EMPTY);
  
// pop fifo
#define POP_FIFO *(fifo + 3) = 1;

// Structure for storing monitoring packet data
struct monitoring_packet {
  int valid;
  int instAddr;
  int memAddr;
  int memEnd;
  int data;
  int store;
  int done;
  int rs1;
  int rs2;
  int rs3;
  int rd;
  int control; // Flag for control instruction 
  int call;    // Flag for call instruction
  int ret;     // Flag for return instruction
  int lr;      // Link register value
  int pcreg;   // Program counter register value
  int load;
  int intalu;
  int indctrl;
};

// Read from fifo into x (which should be struct monitoring_packet)
#define READ_FIFO_ALL(x) memcpy(&x, (void *)(MONITOR_ADDR), sizeof(x))
// Read individual data from fifo (note: either READ_FIFO_ALL or READ_VALID should occur first to read new fifo packet into buffer)
#define READ_FIFO_VALID       *fifo
#define READ_FIFO_PC          *(fifo + 1)
#define READ_FIFO_MEMADDR     *(fifo + 2)
#define READ_FIFO_MEMEND      *(fifo + 3)
#define READ_FIFO_DATA        *(fifo + 4)
#define READ_FIFO_STORE       *(fifo + 5)
#define READ_FIFO_DONE        *(fifo + 6)
#define READ_FIFO_RS1         *(fifo + 7)
#define READ_FIFO_RS2         *(fifo + 8)
#define READ_FIFO_RS3         *(fifo + 9)
#define READ_FIFO_RD          *(fifo + 10)
#define READ_FIFO_CONTROL     *(fifo + 11)
#define READ_FIFO_CALL        *(fifo + 12)
#define READ_FIFO_RET         *(fifo + 13)
#define READ_FIFO_LR          *(fifo + 14)
#define READ_FIFO_NEXTPC      *(fifo + 15)
#define READ_FIFO_LOAD        *(fifo + 16)
#define READ_FIFO_INTALU      *(fifo + 17)
#define READ_FIFO_INDCTRL     *(fifo + 18)

// Fifo flags
#define READ_FIFO_FULL    *(fifo + 0x400)
#define READ_FIFO_EMPTY   *(fifo + 0x400 + 1)

inline void set_tag(unsigned addr, unsigned size, unsigned value);
inline void set_tag_base(unsigned addr, unsigned value);
inline void set_tag_bound(unsigned addr, unsigned value);

void init_section_tags();

#endif // __MONITORING_H__
