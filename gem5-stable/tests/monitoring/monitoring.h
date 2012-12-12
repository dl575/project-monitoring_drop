/*
 * Include file for interface for reading from monitoring fifo.
 *
 * Author: Daniel Lo
 */

#ifndef __MONITORING_H_
#define MONITORING_H

#include <stdbool.h>

// Address of monitoring fifo (for read)
#define MONITOR_ADDR 0x30000000

// initialize fifo so it can be enabled/disabled
#define INIT_MONITOR int *fifo; fifo = (int *)MONITOR_ADDR; 
// enable monitoring
#define ENABLE_MONITOR *fifo = 1;
// disable monitoring
#define DISABLE_MONITOR *fifo = 0;

// Structure for storing monitoring packet data
struct monitoring_packet {
  int instAddr;
  int memAddr;
  int data;
  bool store;
};

// Read from fifo into x (which should be struct monitoring_packet)
#define READ_FIFO(x) memcpy(&x, (void *)(MONITOR_ADDR), sizeof(x))
// Only reads PC, does not perform full memcpy (faster but information lost)
#define READ_PC *fifo

#endif // MONITORING_H
