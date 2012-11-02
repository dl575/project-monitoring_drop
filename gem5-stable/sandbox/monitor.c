
#include <stdio.h>

int main(int argc, char *argv[]) {
  register int i;
  register int data;
  volatile int *fifo;
  fifo = (int *)0x30000000;

  for(i = 0; ; i++) {
    //printf("%x points to %d\n", (unsigned int)fifo, *fifo);
    data = *fifo;
    if (i % 100 == 0 ) {
      printf("%x\n", data);
    }
  }

  return 0;
}
