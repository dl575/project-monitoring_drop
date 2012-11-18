
#include <stdio.h>

int main(int argc, char *argv[]) {
  register int i;
  int *fifo;
  fifo = (int *)0x30000000;
  int data = 0;
  int array[10] = {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10};

  *fifo = 1;
  for (i = 0; i < 10; i++)
    data += array[i];
  *fifo = 0;

  //printf("%d\n", data);

  while(1);

//  for (i = 0; i < 20; i++)
//    *fifo = i % 2;

  return 0;
}
