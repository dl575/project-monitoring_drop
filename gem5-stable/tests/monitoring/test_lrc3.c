
/*
 * Test recursive function. On certain iteration, calls a function which 
 * modifies the link register for return. Test should fail.
 */

#include <stdio.h>

#include "monitoring.h"

// Function attempts to overwrite link register
void __attribute__((noinline)) hax() {
  // 0x8998 = 35224 is the address of the hax function
  __asm__("ldr lr,=#35225"); 
  return;
}

// Recursive Fibonacci function
int __attribute__((noinline)) fib(int n) {
  if (n <= 1) {
    return 1;
  } else if (n == 8) {
    hax();
    return 0;
  } else {
    return fib(n-1) + fib(n-2);
  }
}

int main(int argc, char *argv[]) {
  INIT_MONITOR
  INIT_BSS
  INIT_CODE

  int i;
  int sum = 0;

  ENABLE_MONITOR
  
  // Find some fibonacci numbers
  for (i = 0; i < 10; i++) {
    sum += fib(i);
  }

  // Main core finished
  DISABLE_MONITOR;

  // sum should equal 143
  printf("sum = %d\n", sum);

  MAIN_DONE

  return 0;
}
