
#ifndef __MALARDEN_H__
#define __MALARDEN_H__

#include "../../monitoring/timer.h"
#include "../../monitoring/monitoring.h"


#ifndef WCET_SCALE
    #define WCET_SCALE 1
#endif

// WCETs in cycles
// fac.c
#define WCETC_FAC 22

// insertsort.c
#define WCETC_IS 1292

// fibcall.c
#define WCETC_FC 4

// Scaled WCETs 
#define WCET_FAC WCETC_FAC*WCET_SCALE
#define WCET_IS  WCETC_IS*WCET_SCALE
#define WCET_FC  WCETC_FC*WCET_SCALE


int fac(int n);
int factorial();

int insertsort();

int fib(int n);
int fibcall();

#endif // __MALARDEN_H__

