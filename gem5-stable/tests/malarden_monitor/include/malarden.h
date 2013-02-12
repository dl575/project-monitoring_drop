
#ifndef __MALARDEN_H__
#define __MALARDEN_H__

#include "../../monitoring/timer.h"
#include "../../monitoring/monitoring.h"


#ifndef WCET_SCALE
    #define WCET_SCALE 1
#endif

// WCETs in cycles
// fac.c
#define WCET_FAC_SUB 178

// fibcall.c
#define WCET_FC_SUB 27

// Scaled WCETs 
#define WCET_FAC 1*(WCET_SCALE-1)
#define WCET_FC  1*(WCET_SCALE-1)

int fac(int n);
int factorial();

int fib(int n);
int fibcall();


/** insertsort.c **/
#define WCET_IS  5833*(WCET_SCALE-1)
#define WCET_IS_1 171
#define WCET_IS_2 1064

int insertsort();

/** crc.c **/
#define WCET_CRC 38229*(WCET_SCALE-1)
#define WCET_CRC_1 500
#define WCET_CRC_2 499
#define WCET_CRC_3 118
#define WCET_CRC_4 30
#define WCET_CRC_5 88
#define WCET_CRC_6 306


typedef unsigned char uchar;
#define LOBYTE(x) ((uchar)((x) & 0xFF))
#define HIBYTE(x) ((uchar)((x) >> 8))

unsigned short icrc1(unsigned short crc, unsigned char onech);
unsigned short icrc(unsigned short crc, unsigned long len, short jinit, int jrev, unsigned char * lin);
int crc (void);

#endif // __MALARDEN_H__

