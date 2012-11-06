
#include <stdio.h>

int main(int argc, char *argv[]) {
  int i;
  int *fifo;
  fifo = (int *)0x30000000;

  *fifo = 1;
  for (i = 0; i < 20; i++);
  *fifo = 0;

  while(1);

//  for (i = 0; i < 20; i++)
//    *fifo = i % 2;

  return 0;
}
