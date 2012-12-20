/*
 * Include file for interface for reading from monitoring fifo.
 *
 * Author: Daniel Lo
 */

#ifndef __MONITORING_H_
#define MONITORING_H

#include <stdbool.h>
#include <stdint.h>

// Address of monitoring fifo (for read)
#define MONITOR_ADDR 0x30000000

// initialize fifo so it can be enabled/disabled
#define INIT_MONITOR int *fifo; fifo = (int *)MONITOR_ADDR; 
// enable monitoring
#define ENABLE_MONITOR *fifo = 1;
// disable monitoring
#define DISABLE_MONITOR *fifo = 0;
// main core has finished
#define MAIN_DONE *fifo = 2;

// Structure for storing monitoring packet data
struct monitoring_packet {
  int valid;
  int instAddr;
  int memAddr;
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
#define READ_FIFO_DATA    *(fifo + 3)
#define READ_FIFO_STORE   *(fifo + 4)
#define READ_FIFO_DONE    *(fifo + 5)

// Fifo flags
#define READ_FIFO_FULL    *(fifo + 0x400)
#define READ_FIFO_EMPTY   *(fifo + 0x400 + 1)

#endif // MONITORING_H
