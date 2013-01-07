#include <stdio.h>


extern void * __bss_start__;
extern void * __bss_end__;

int main(int argc, char *argv[]) {

  printf("BSS Start = %x\n", (unsigned int)&__bss_start__);
  printf("BSS End = %x\n", (unsigned int)&__bss_end__);

  return 0;
}
