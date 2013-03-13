/*
 *
 * A monitor that just generates packets.
 * Used to calculate wcet of a monitoring program.
 *
 * Author: Mohamed Ismail
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "monitoring.h"
#include "timer.h"

#define ARRSIZE 1000

int __attribute__((noinline)) add(int a, int b) {
  return a + b;
}

int main(int argc, char *argv[]) {

  INIT_MONITOR;
  INIT_TIMER;
  INIT_CODE;
  
  ENABLE_MONITOR;
  
  unsigned volatile int arr[ARRSIZE];
  unsigned int i;
  register sum = 0;
  
  for (i = 0; i < ARRSIZE; ++i){
    arr[i] = i;
    //sum += arr[i];
    sum = add(sum, arr[i]);
  }
  
  START_TASK(0);
  for (i = 0; i < ARRSIZE; ++i){
    arr[i] = i;
    //sum += arr[i];
    sum = add(sum, arr[i]);
  }
  END_TASK(ARRSIZE*200);
  
  DISABLE_MONITOR;

  printf("Finished: %d\n", sum);
  
  MAIN_DONE;
  
  return 0;
}
