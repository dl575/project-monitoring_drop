
#ifndef __MALARDEN_H__
#define __MALARDEN_H__

#include "../../monitoring/timer.h"
#include "../../monitoring/monitoring.h"


#ifndef WCET_SCALE
    #define WCET_SCALE 5
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

/** compress.c **/
// No monitoring: 5453
#define WCET_CMP 5453*(WCET_SCALE-1)
#define WCET_CMP_1 703
#define WCET_CMP_2 736
#define WCET_CMP_3 61
#define WCET_CMP_4 25
#define WCET_CMP_5 17
#define WCET_CMP_6 19
int compress_main(void);

/** fir.c **/
// No monitoring: 6847
#define WCET_FIR 6847*(WCET_SCALE-1)
#define WCET_FIR_1 44
#define WCET_FIR_2 16
#define WCET_FIR_3 10
#define WCET_FIR_4 109
#define WCET_FIR_5 3602
int fir_main(void);

/** jfdcint.c **/
// No monitoring: xx
#define WCET_JFDC 0*(WCET_SCALE-1)
#define WCET_JFDC_1 1000
#define WCET_JFDC_2 1000
#define WCET_JFDC_3 1000
#define WCET_JFDC_4 1000
#define WCET_JFDC_5 1000
#define WCET_JFDC_6 1000
int jfdc(void);

#endif // __MALARDEN_H__

