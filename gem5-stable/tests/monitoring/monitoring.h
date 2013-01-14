/*
 * Include file for interface for reading from monitoring fifo.
 *
 * Author: Daniel Lo
 */

#ifndef __MONITORING_H__
#define __MONITORING_H__

#include <stdbool.h>
#include <stdint.h>

// Address of monitoring fifo (for read)
#define MONITOR_ADDR 0x30000000

// initialize fifo so it can be enabled/disabled
#define INIT_MONITOR register unsigned int *fifo = (unsigned int *)MONITOR_ADDR; 
// enable monitoring
#define ENABLE_MONITOR *fifo = 1;
// disable monitoring
#define DISABLE_MONITOR *fifo = 0;
// main core has finished
#define MAIN_DONE *fifo = 2; while(1);

// make custom FIFO packets
#define WRITE_FIFO_START(x) *(fifo + 1) = x;
#define WRITE_FIFO_END(x) *(fifo + 2) = x;
#define WRITE_FIFO_RANGE(x1,x2) *(fifo + 1) = x1; *(fifo + 2) = x2;

// bss initialization
#define INIT_BSS extern void * __bss_start__; \
  extern void * __bss_end__; \
  WRITE_FIFO_RANGE((unsigned int)&__bss_start__, (unsigned int)&__bss_end__)

// Structure for storing monitoring packet data
struct monitoring_packet {
  int valid;
  int instAddr;
  int memAddr;
  int memEnd;
  int data;
  int store;
  int done;
};

// Read from fifo into x (which should be struct monitoring_packet)
#define READ_FIFO_ALL(x) memcpy(&x, (void *)(MONITOR_ADDR), sizeof(x))
// Read individual data from fifo (note: either READ_FIFO or READ_VALID should occur first to read new fifo packet into buffer)
#define READ_FIFO_VALID   *fifo
#define READ_FIFO_PC      *(fifo + 1)
#define READ_FIFO_MEMADDR *(fifo + 2)
#define READ_FIFO_MEMEND  *(fifo + 3)
#define READ_FIFO_DATA    *(fifo + 4)
#define READ_FIFO_STORE   *(fifo + 5)
#define READ_FIFO_DONE    *(fifo + 6)

// Fifo flags
#define READ_FIFO_FULL    *(fifo + 0x400)
#define READ_FIFO_EMPTY   *(fifo + 0x400 + 1)

#endif // __MONITORING_H__
