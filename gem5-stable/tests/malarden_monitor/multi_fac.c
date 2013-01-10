
#include <stdio.h>
#include <unistd.h>

#include "../monitoring/monitoring.h"
#include "../monitoring/timer.h"

#define TICKS_PER_CYCLE 500
#ifndef WCET_CYCLES1
  #define WCET_CYCLES1 50
#endif
#ifndef WCET_CYCLES2
  #define WCET_CYCLES2 50
#endif
#define WCET1 WCET_CYCLES1*TICKS_PER_CYCLE
#define WCET2 WCET_CYCLES2*TICKS_PER_CYCLE

int fac(int n);
int factorial(void);
int factorial2(void);

int *timer;

int main(int argc, char* argv[]) {

  INIT_MONITOR;
  ENABLE_MONITOR;

  timer = (int *)TIMER_ADDR;

  int i;
  int r1, r2;

  for (i = 0; i < 5; i++) {
    r1 = factorial();
    r2 = factorial2();
  }

  DISABLE_MONITOR;

  printf("results = (%d, %d)\n", r1, r2);

  MAIN_DONE;

  return 0;
}

int 
fac(int n)
{
	if (n == 0)
		return 1;
	else
		return (n * fac(n - 1));
}

int 
factorial(void)
{
  START_TASK(0);

	int             i;
	int             s = 0;

  START_SUBTASK(WCET1);
	for (i = 0; i <= 5; i++) {
		s += fac(i);
    ENDSTART_SUBTASK(WCET1);
  }
  END_SUBTASK(WCET1);

  END_TASK;

	return (s);
}

int 
factorial2(void)
{
  START_TASK(0);

	int             i;
	int             s = 0;

  START_SUBTASK(WCET2);
	for (i = 0; i <= 10; i++) {
		s += fac(i);
    ENDSTART_SUBTASK(WCET2);
  }
  END_SUBTASK(WCET2);

  END_TASK;

	return (s);
}
