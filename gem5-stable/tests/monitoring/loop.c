
#include <stdio.h>

int main(int argc, char *argv[]) {
  register int i;
  int *fifo;
  fifo = (int *)0x30000000;
  int data = 0;
  int array[20] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

  *fifo = 1;
  for (i = 0; i < 20; i++)
    data += array[i];
  *fifo = 0;

  //printf("%d\n", data);

  while(1);

//  for (i = 0; i < 20; i++)
//    *fifo = i % 2;

  return 0;
}
