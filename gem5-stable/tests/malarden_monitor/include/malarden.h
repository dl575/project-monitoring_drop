
#ifndef __MALARDEN_H__
#define __MALARDEN_H__

#include "../../monitoring/timer.h"
#include "../../monitoring/monitoring.h"


#ifndef WCET_SCALE
    #define WCET_SCALE 1
#endif

/** insertsort.c **/
// No monitoring: 733
#define WCET_IS  2269*(WCET_SCALE-1)
#define WCET_IS_1 19
#define WCET_IS_2 412
int insertsort();

/** crc.c **/
// No monitoring: 31246
#define WCET_CRC 32765*(WCET_SCALE-1)
#define WCET_CRC_1 67
#define WCET_CRC_2 165
#define WCET_CRC_3 116
#define WCET_CRC_4 20
#define WCET_CRC_5 35
#define WCET_CRC_6 103
int crc(void);

/** edn.c **/
// No monitoring: 35870
#define WCET_EDN 130326*(WCET_SCALE-1)
#define WCET_EDN_1 4379
#define WCET_EDN_2 33
#define WCET_EDN_3 143
#define WCET_EDN_4 25
#define WCET_EDN_5 136
#define WCET_EDN_6 1118
#define WCET_EDN_7 195
#define WCET_EDN_8 765
#define WCET_EDN_9 216
#define WCET_EDN_10 39
#define WCET_EDN_11 256
#define WCET_EDN_12 87
#define WCET_EDN_13 423
#define WCET_EDN_14 1090
#define WCET_EDN_15 90
int edn(void);



#endif // __MALARDEN_H__

