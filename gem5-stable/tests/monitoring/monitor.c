
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

int main(int argc, char *argv[]) {
  register int i;
//  register int data;
//  unsigned long long data;
//  volatile int *fifo;
//  fifo = (int *)0x30000000;
//  unsigned long long *fifo;
//  fifo = (unsigned long long *)0x30000000;

  struct {
    int instAddr;
    int memAddr;
    int data;
    bool store;
  } data;

  for(i = 0; i < 30; i++) {
    //printf("%x points to %d\n", (unsigned int)fifo, *fifo);
    //data = *fifo;
    memcpy(&data, (void *)(0x30000000), 3*sizeof(int) + sizeof(bool));
//      printf("%llu\n", data);
    printf("%x : m[%x] = %x ", data.instAddr, data.memAddr, data.data);
    if (data.store) {
      printf("store\n");
    } else {
      printf("load\n");
    }
  }

  return 0;
}
