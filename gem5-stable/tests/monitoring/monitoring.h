/*
 * Include file for interface for reading from monitoring fifo.
 *
 * Author: Daniel Lo
 */

#ifndef __MONITORING_H_
#define MONITORING_H

// Address of monitoring fifo (for read)
#define MONITOR_ADDR 0x30000000

// Structure for storing monitoring packet data
struct monitoring_packet {
  int instAddr;
  int memAddr;
  int data;
  bool store;
};

// Read from fifo into x (which should be struct monitoring_packet)
#define READ_FIFO(x) memcpy(&x, (void *)(MONITOR_ADDR), sizeof(x))

#endif // MONITORING_H
