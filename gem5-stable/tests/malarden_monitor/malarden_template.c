
#include <stdio.h>

#include "../../monitoring/monitoring.h"
#include "../../monitoring/timer.h"
#include "../include/malarden.h"

int main(int argc, char* argv[]) {

  INIT_MONITOR;
  INIT_CODE;
  INIT_TIMER;
  
  ENABLE_MONITOR;

  int i;
  for (i = 0; i < 5; i++) {
    <INSERT FUNCTIONS>
  }

  DISABLE_MONITOR;
  MAIN_DONE;

  return 0;
}
