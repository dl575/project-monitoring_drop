
/*
 * Test multiple different nested functions. Test should pass.
 */

#include <stdio.h>
#include <math.h>

#include "monitoring.h"

#define N 10

int __attribute__((noinline)) add(int a, int b) {
  return a + b;
}

int __attribute__((noinline)) mul(int a, int b) {
  int result = 0;
  int i;
  for (i = 0; i < a; i++) {
    result = add(result, b);
  }
  return result;
}

int __attribute__((noinline)) norm(int a, int b) {
  return add(mul(a, a), mul(b, b));
}

void __attribute__((noinline)) innerloop(int i, int (*array)[N]) {
  int j;
  for (j = 0; j < N; j++) { 
    array[i][j] = norm(i, j);
  }
}

void __attribute__((noinline)) outerloop(int (*array)[N]) {
  int i;
  for (i = 0; i < N; i++) {
    innerloop(i, array);
  }
}

int main(int argc, char *argv[]) {
  INIT_MONITOR
  INIT_BSS
  INIT_CODE

  int i, j;
  int array[N][N];

  ENABLE_MONITOR
  
  // Find some norms
  outerloop(array);

  // Main core finished
  DISABLE_MONITOR;

  for (i = 0; i < N; i++) {
    for (j = 0; j < N; j++) {
      printf("%d ", array[i][j]);
    }
    printf("\n");
  }

  MAIN_DONE

  return 0;
}
