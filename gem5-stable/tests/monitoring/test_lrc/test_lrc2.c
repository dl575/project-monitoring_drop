
/*
 * Test recursive function. Test should pass.
 */

#include <stdio.h>

#include "monitoring_wcet.h"

// Recursive Fibonacci function
int __attribute__((noinline)) fib(int n) {
  if (n <= 1) {
    return 1;
  } else {
    return fib(n-1) + fib(n-2);
  }
}

int main(int argc, char *argv[]) {
  INIT_MONITOR
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
