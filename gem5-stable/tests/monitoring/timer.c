
#include <stdio.h>

int main(int argc, char *argv[]) {
  int *fifo;
  fifo = (int *)0x30000000;

  int *timer;
  timer = (int *)0x31000000;

  printf("write\n");
  *timer = 0x1337;
  printf("read\n");
  printf("%x\n", *timer);


  /*
  register int i;
  int sum;
  int array[10];

  // Initialize array
  for (i = 0; i < 10; i++)
    array[i] = 0x10;
  // Initalize sum
  sum = 0;

  for (i = 0; i < 10; i++) {
    sum += array[i];
    printf("%d\n", sum);
  }
  */

  return 0;
}
