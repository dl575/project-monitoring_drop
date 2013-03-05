
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

/** compress.c **/
// No monitoring: 5213
#define WCET_CMP 18189*(WCET_SCALE-1)
#define WCET_CMP_1 923
#define WCET_CMP_2 3239
#define WCET_CMP_3 222
#define WCET_CMP_4 81
#define WCET_CMP_5 50
#define WCET_CMP_6 99
int compress_main(void);

/** fir.c **/
// No monitoring: 3208
#define WCET_FIR 7120*(WCET_SCALE-1)
#define WCET_FIR_1 93
#define WCET_FIR_2 32
#define WCET_FIR_3 26
#define WCET_FIR_4 180
int fir_main(void);

/** jfdcint.c **/
// No monitoring: 3304
#define WCET_JFDC 9760*(WCET_SCALE-1)
#define WCET_JFDC_1 1202
#define WCET_JFDC_2 190
#define WCET_JFDC_3 575
#define WCET_JFDC_4 12
#define WCET_JFDC_5 519
#define WCET_JFDC_6 88
int jfdc(void);

/** nsichneu.c **/
// No monitoring: 6157
#define WCET_NSI 22052*(WCET_SCALE-1)
#define WCET_NSI_1 4
#define WCET_NSI_2 403
#define WCET_NSI_3 1771
#define WCET_NSI_4 1771
#define WCET_NSI_5 1771
#define WCET_NSI_6 1771
#define WCET_NSI_7 1773
#define WCET_NSI_8 1803
int nsichneu();

/** statemate.c **/
// No monitoring: 1809
#define WCET_STA 6649*(WCET_SCALE-1)
#define WCET_STA_1 414
#define WCET_STA_2 188
#define WCET_STA_3 290
#define WCET_STA_4 64
#define WCET_STA_5 420
#define WCET_STA_6 2417
int statemate(void);

#endif // __MALARDEN_H__

