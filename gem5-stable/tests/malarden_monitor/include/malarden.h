
#ifndef __MALARDEN_H__
#define __MALARDEN_H__

#include "../../monitoring/timer.h"
#include "../../monitoring/monitoring.h"


#ifndef WCET_SCALE
    #define WCET_SCALE 1
#endif

/** insertsort.c **/
#define WCET_IS  5772*(WCET_SCALE-1)
#define WCET_IS_1 271
#define WCET_IS_2 1013
int insertsort();

/** crc.c **/
#define WCET_CRC 38110*(WCET_SCALE-1)
#define WCET_CRC_1 412
#define WCET_CRC_2 471
#define WCET_CRC_3 118
#define WCET_CRC_4 29
#define WCET_CRC_5 89
#define WCET_CRC_6 307
int crc(void);

/** edn.c **/
#define WCET_EDN 331126*(WCET_SCALE-1)
#define WCET_EDN_1 13127
#define WCET_EDN_2 84
#define WCET_EDN_3 390
#define WCET_EDN_4 56
#define WCET_EDN_5 359
#define WCET_EDN_6 2831
#define WCET_EDN_7 526
#define WCET_EDN_8 1932
#define WCET_EDN_9 555
#define WCET_EDN_10 84
#define WCET_EDN_11 727
#define WCET_EDN_12 224
#define WCET_EDN_13 1175
#define WCET_EDN_14 2912
#define WCET_EDN_15 253
int edn(void);



#endif // __MALARDEN_H__

