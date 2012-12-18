
#include <stdio.h>

#include "monitoring.h"

int main(int argc, char *argv[]) {
  INIT_MONITOR

  register int i;
  int sum;
  int array[10];

  ENABLE_MONITOR

  // Initialize array
  for (i = 0; i < 10; i++)
    array[i] = 0x10;
  // Initalize sum
  sum = 0;

  for (i = 0; i < 10; i++)
    sum += array[i];

  // Main core finished
  MAIN_DONE
  DISABLE_MONITOR;

  printf("sum = %d\n", sum);

  // Spin until monitoring is finished
  while(1);

  return 0;
}
