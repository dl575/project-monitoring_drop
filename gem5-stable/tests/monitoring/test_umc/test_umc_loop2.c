
/*
 * First two elements of array are not initialized. Test should fail.
 */

#include <stdio.h>

#include "monitoring_wcet.h"

int main(int argc, char *argv[]) {
  INIT_MONITOR
  INIT_CODE

  int i;
  int array[10];
  int sum;

  ENABLE_MONITOR
  
  // Initialize array, don't initialize i=0,1
  for (i = 2; i < 10; i++)
     array[i] = i;
  // Initalize sum
  sum = 0;

  for (i = 0; i < 10; i++)
    sum += array[i];

  // Main core finished
  DISABLE_MONITOR;

  printf("sum = %d\n", sum);

  MAIN_DONE

  return 0;
}
