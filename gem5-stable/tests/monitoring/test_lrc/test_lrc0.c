
/*
 * No funny business. This test should pass.
 */

#include <stdio.h>

#include "monitoring.h"

int __attribute__((noinline)) add(int a, int b) {
  return a + b;
}

int __attribute__((noinline)) sub(int a, int b) {
  return a - b;
}

int __attribute__((noinline)) mult(int a, int b) {
  return a*b;
}

// Error function attempts to overwrite link register
void __attribute__((noinline)) error() {
  // load function address
  int (*func)() = add;
  // load address to link register
  __asm__(
    "mov lr, %0"
    :
    : "r" (func)
  ); 
  return;
}

int main(int argc, char *argv[]) {
  INIT_MONITOR
  INIT_CODE

  int i;
  int array[10];
  int sum;

  ENABLE_MONITOR
  
  // Initialize array
  for (i = 0; i < 10; i++)
     array[i] = i+1;
  // Initalize sum
  sum = 1;

  // Call some functions
  for (i = 0; i < 10; i++) {
    sum = add(sum, array[i]);
    sum = sub(sum, array[i]);
    sum = mult(sum, array[i]);
  }
  //error();

  // Main core finished
  DISABLE_MONITOR;

  printf("sum = %d\n", sum);

  MAIN_DONE

  return 0;
}
