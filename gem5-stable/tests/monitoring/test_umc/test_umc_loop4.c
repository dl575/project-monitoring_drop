
/*
 * Varables are global. INIT_BSS should handle initialization.
 * No error should occur.
 */

#include <stdio.h>

#include "monitoring.h"

int array[10];
int sum;

int main(int argc, char *argv[]) {
  INIT_MONITOR
  INIT_CODE

  int i;

  ENABLE_MONITOR
  
  /*
  // Initialize array
  for (i = 0; i < 10; i++)
     array[i] = i;
  // Initalize sum
  sum = 0;
  */

  for (i = 0; i < 10; i++)
    sum += array[i];

  // Main core finished
  DISABLE_MONITOR;

  printf("sum = %d\n", sum);

  MAIN_DONE

  return 0;
}
