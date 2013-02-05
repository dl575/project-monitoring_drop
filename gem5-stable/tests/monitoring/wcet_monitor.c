
/*
 * umc.c
 *
 * Unitialized memory check. Sets the metadata for a memory address to a 1
 * when written to. On a read, checks that the memory address is 1. If not
 * indicates an error. This version does not take into account slack.
 *
 * Author: Daniel Lo
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "monitoring.h"

int main(int argc, char *argv[]) {

  INIT_MONITOR;

  // Main loop, loop until main core signals done
  while(!READ_FIFO_DONE || !READ_FIFO_VALID) {
    POP_FIFO;    
  }

  printf("Finished\n");
  return 0;
}
