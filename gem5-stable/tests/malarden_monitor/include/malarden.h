
#ifndef __MALARDEN_H__
#define __MALARDEN_H__

#include "../../monitoring/timer.h"

extern int *timer;

#define TICKS_PER_CYCLE 500

// WCETs in cycles
// fac.c
#ifndef WCETC_FAC
  #define WCETC_FAC 50
#endif
// insertsort.c
#ifndef WCETC_IS
  #define WCETC_IS 50
#endif
// fibcall.c
#ifndef WCETC_FC
  #define WCETC_FC 50
#endif

// WCETs in ticks
#define WCET_FAC WCETC_FAC*TICKS_PER_CYCLE
#define WCET_IS  WCETC_IS*TICKS_PER_CYCLE
#define WCET_FC  WCETC_FC*TICKS_PER_CYCLE

int fac(int n);
int factorial();

int insertsort();

int fib(int n);
int fibcall();

#endif // __MALARDEN_H__

