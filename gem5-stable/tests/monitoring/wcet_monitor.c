
/*
 *
 * A monitor that does nothing. 
 * Used to calculate wcet of a program.
 * Give it an argument of cycles to wait.
 *
 * Author: Mohamed Ismail
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "monitoring.h"

#define IMPLICIT_DELAY 1

int main(int argc, char *argv[]) {

  INIT_MONITOR;
  
  register volatile i, j;
  register delay;
  
  if (argc == 2) {
    delay = atoi(argv[1]) - IMPLICIT_DELAY;
  }else{
    delay = 0;
  }

  // Main loop, loop until main core signals done
  while(READ_FIFO_VALID) {
    //Delays monitor to emulate real monitor
    for (i = delay; i > 0; i--){
        j = 0; j = 0; //Just takes some cycles to get a better multiple
    }
    POP_FIFO; //It takes 8 cycles to get here
  }
  
  return 1;
}
