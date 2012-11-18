
#include <stdio.h>

int main(int argc, char *argv[]) {
  int *fifo;
  fifo = (int *)0x30000000;

  register int i;
  int sum;
  int array[10];

  *fifo = 1;

  // Initialize array
  for (i = 0; i < 10; i++)
    array[i] = 0x10;
  // Initalize sum
  sum = 0;

  for (i = 0; i < 10; i++)
    sum += array[i];

  *fifo = 0;

  while(1);

  return 0;
}
