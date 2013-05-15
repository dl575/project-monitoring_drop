
/*
 * Sum variable is not initialized. 
 * This test passes with -O2 flag. The compiler assigns sum to a
 * register which is not initialized but also never loads from memory.
 */

#include <stdio.h>

#include "monitoring.h"

int main(int argc, char *argv[]) {
  INIT_MONITOR
  INIT_CODE

  int i;
  int array[10];
  int sum;

  ENABLE_MONITOR
  
  // Initialize array
  for (i = 0; i < 10; i++)
     array[i] = i;
  // Initalize sum
  //sum = 0;

  for (i = 0; i < 10; i++)
    sum += array[i];

  // Main core finished
  DISABLE_MONITOR;

  printf("sum = %d\n", sum);

  MAIN_DONE

  return 0;
}
