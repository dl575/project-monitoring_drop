#include <stdio.h>


extern void * __bss_start__;
extern void * __bss_end__;
extern void * _start;
extern void * _end;

int main(int argc, char *argv[]) {

  printf("Code Start = %x\n", (unsigned int)&_start);
  printf("BSS Start = %x\n", (unsigned int)&__bss_start__);
  printf("BSS End = %x\n", (unsigned int)&__bss_end__);
  printf("Code End = %x\n", (unsigned int)&_end);

  return 0;
}
