#include <stdio.h>

#include "monitoring_wcet.h"
#include "increment.h"

int main( void ) {
  INIT_MONITOR
  INIT_BSS

  ENABLE_MONITOR
        
  increment_init();
  increment();

  // Main core finished
  DISABLE_MONITOR;

  printf("sum = %d\n", sum);

  MAIN_DONE

  return 0;
}
