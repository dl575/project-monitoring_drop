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

#include "monitoring_wcet.h"
#include "timer.h"

#define ARRSIZE 200

int __attribute__((noinline)) add(int a, int b, int depth) {
  return (depth > 0)? a + b + add(a, b, depth-1) : 0;
}

int main(int argc, char *argv[]) {

  INIT_MONITOR;
  INIT_TIMER;
  INIT_CODE;
  
  ENABLE_MONITOR;
  
  START_TASK(0);
  
  unsigned volatile int arr[ARRSIZE];
  unsigned int i;
  register volatile int sum = 0;
  
  for (i = 0; i < ARRSIZE; ++i){
    arr[i] = i;
  }
  for (i = 0; i < ARRSIZE; ++i){
    //sum += arr[i];
    sum = add(sum, arr[i], i%100);
  }
  
  END_TASK(ARRSIZE*200);
  
  DISABLE_MONITOR;

  printf("Finished: %d\n", sum);
  
  MAIN_DONE;
  
  return 0;
}
