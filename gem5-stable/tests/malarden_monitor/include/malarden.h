
#ifndef __MALARDEN_H__
#define __MALARDEN_H__

#include "../../monitoring/timer.h"
#include "../../monitoring/monitoring.h"


#ifndef WCET_SCALE
    #define WCET_SCALE 3
#endif

/** Function definitions **/
int insertsort();
int crc(void);
int edn(void);
int compress(void);
int fir(void);
int jfdc(void);
int nsichneu();
int statemate(void);

/** WCET for UMC **/
#ifdef UMC
  /** insertsort.c **/
  // #define WCET_IS  956*(WCET_SCALE-1)
  // #define WCET_IS_1 19
  // #define WCET_IS_2 170
  // No overhead
  // #define WCET_IS  46500*(WCET_SCALE-1)
  // #define WCET_IS_1 180
  // #define WCET_IS_2 7980
  // SW based
  #define WCET_IS  46500*(WCET_SCALE-1)
  #define WCET_IS_1 180
  #define WCET_IS_2 7980
  
  /** crc.c **/
  // #define WCET_CRC 31253*(WCET_SCALE-1)
  // #define WCET_CRC_1 37
  // #define WCET_CRC_2 32
  // #define WCET_CRC_3 116
  // #define WCET_CRC_4 12
  // #define WCET_CRC_5 19
  // #define WCET_CRC_6 33
  // No overhead
  // #define WCET_CRC 89400*(WCET_SCALE-1)
  // #define WCET_CRC_1 960
  // #define WCET_CRC_2 1260
  // #define WCET_CRC_3 720
  // #define WCET_CRC_4 1140
  // #define WCET_CRC_5 960
  // SW based
  #define WCET_CRC 89460*(WCET_SCALE-1)
  #define WCET_CRC_1 960
  #define WCET_CRC_2 1260
  #define WCET_CRC_3 720
  #define WCET_CRC_4 1140
  #define WCET_CRC_5 960
  
  /** edn.c **/
  // #define WCET_EDN 35861*(WCET_SCALE-1)
  // #define WCET_EDN_1 751
  // #define WCET_EDN_2 9
  // #define WCET_EDN_3 31
  // #define WCET_EDN_4 9
  // #define WCET_EDN_5 32
  // #define WCET_EDN_6 310
  // #define WCET_EDN_7 43
  // #define WCET_EDN_8 213
  // #define WCET_EDN_9 56
  // #define WCET_EDN_10 15
  // #define WCET_EDN_11 47
  // #define WCET_EDN_12 23
  // #define WCET_EDN_13 88
  // #define WCET_EDN_14 258
  // #define WCET_EDN_15 18
  // No overhead
  // #define WCET_EDN 2795700*(WCET_SCALE-1)
  // #define WCET_EDN_1 1080
  // #define WCET_EDN_2 900
  // #define WCET_EDN_3 2100
  // #define WCET_EDN_4 660
  // #define WCET_EDN_5 1860
  // #define WCET_EDN_6 24720
  // #define WCET_EDN_7 2820
  // #define WCET_EDN_8 15960
  // #define WCET_EDN_9 3300
  // #define WCET_EDN_10 1500
  // #define WCET_EDN_11 3420
  // #define WCET_EDN_12 1860
  // #define WCET_EDN_13 6480
  // #define WCET_EDN_14 21240
  // #define WCET_EDN_15 960
  // SW based
  #define WCET_EDN 2795700*(WCET_SCALE-1)
  #define WCET_EDN_1 1080
  #define WCET_EDN_2 900
  #define WCET_EDN_3 2100
  #define WCET_EDN_4 660
  #define WCET_EDN_5 1860
  #define WCET_EDN_6 24720
  #define WCET_EDN_7 2820
  #define WCET_EDN_8 15960
  #define WCET_EDN_9 3300
  #define WCET_EDN_10 1500
  #define WCET_EDN_11 3420
  #define WCET_EDN_12 1860
  #define WCET_EDN_13 6480
  #define WCET_EDN_14 21240
  #define WCET_EDN_15 960
  
  /** compress.c **/
  // #define WCET_CMP 5161*(WCET_SCALE-1)
  // #define WCET_CMP_1 706
  // #define WCET_CMP_2 735
  // #define WCET_CMP_3 55
  // #define WCET_CMP_4 18
  // #define WCET_CMP_5 25
  // #define WCET_CMP_6 19
  // No overhead
  // #define WCET_CMP 377880*(WCET_SCALE-1)
  // #define WCET_CMP_1 60180
  // #define WCET_CMP_2 4920
  // #define WCET_CMP_3 1920
  // #define WCET_CMP_4 1380
  // #define WCET_CMP_5 1020
  // SW based
  #define WCET_CMP 377940*(WCET_SCALE-1)
  #define WCET_CMP_1 60180
  #define WCET_CMP_2 4920
  #define WCET_CMP_3 1920
  #define WCET_CMP_4 1380
  #define WCET_CMP_5 1020
  
  /** fir.c **/
  // #define WCET_FIR 3209*(WCET_SCALE-1)
  // #define WCET_FIR_1 46
  // #define WCET_FIR_2 16
  // #define WCET_FIR_3 10
  // #define WCET_FIR_4 103
  // No overhead
  // #define WCET_FIR 199920*(WCET_SCALE-1)
  // #define WCET_FIR_1 3180
  // #define WCET_FIR_2 960
  // #define WCET_FIR_3 600
  // #define WCET_FIR_4 6780
  // SW based
  #define WCET_FIR 199920*(WCET_SCALE-1)
  #define WCET_FIR_1 3180
  #define WCET_FIR_2 960
  #define WCET_FIR_3 600
  #define WCET_FIR_4 6780
  
  /** jfdcint.c **/
  // #define WCET_JFDC 3554*(WCET_SCALE-1)
  // #define WCET_JFDC_1 1218
  // #define WCET_JFDC_2 45
  // #define WCET_JFDC_3 157
  // #define WCET_JFDC_4 3
  // #define WCET_JFDC_5 143
  // #define WCET_JFDC_6 16
  // No overhead
  // #define WCET_JFDC 197880*(WCET_SCALE-1)
  // #define WCET_JFDC_1 3240
  // #define WCET_JFDC_2 13320
  // #define WCET_JFDC_3 240
  // #define WCET_JFDC_4 12600
  // #define WCET_JFDC_5 960
  // SW based
  #define WCET_JFDC 197880*(WCET_SCALE-1)
  #define WCET_JFDC_1 3240
  #define WCET_JFDC_2 13320
  #define WCET_JFDC_3 240
  #define WCET_JFDC_4 12600
  #define WCET_JFDC_5 960
  
  /** nsichneu.c **/
  // #define WCET_NSI 6155*(WCET_SCALE-1)
  // #define WCET_NSI_1 4
  // #define WCET_NSI_2 115
  // #define WCET_NSI_3 491
  // #define WCET_NSI_4 491
  // #define WCET_NSI_5 493
  // #define WCET_NSI_6 492
  // #define WCET_NSI_7 493
  // #define WCET_NSI_8 499
  // No overhead
  // #define WCET_NSI 489540*(WCET_SCALE-1)
  // #define WCET_NSI_1 360
  // #define WCET_NSI_2 9120
  // #define WCET_NSI_3 39180
  // #define WCET_NSI_4 39120
  // #define WCET_NSI_5 39120
  // #define WCET_NSI_6 39180
  // #define WCET_NSI_7 39120
  // #define WCET_NSI_8 39540
  // SW based
  #define WCET_NSI 488820*(WCET_SCALE-1)
  #define WCET_NSI_1 420
  #define WCET_NSI_2 9060
  #define WCET_NSI_3 39060
  #define WCET_NSI_4 39120
  #define WCET_NSI_5 39060
  #define WCET_NSI_6 39060
  #define WCET_NSI_7 39120
  #define WCET_NSI_8 39540
  
  /** statemate.c **/
  // #define WCET_STA 1785*(WCET_SCALE-1)
  // #define WCET_STA_1 134
  // #define WCET_STA_2 36
  // #define WCET_STA_3 57
  // #define WCET_STA_4 23
  // #define WCET_STA_5 106
  // #define WCET_STA_6 654
  // No overhead
  // #define WCET_STA 132420*(WCET_SCALE-1)
  // #define WCET_STA_1 3300
  // #define WCET_STA_2 4980
  // #define WCET_STA_3 1560
  // #define WCET_STA_4 9240
  // #define WCET_STA_5 51120
  // SW based
  #define WCET_STA 132480*(WCET_SCALE-1)
  #define WCET_STA_1 3300
  #define WCET_STA_2 4980
  #define WCET_STA_3 1680
  #define WCET_STA_4 9240
  #define WCET_STA_5 51600
#endif // UMC

/** WCET for LRC **/
#ifdef LRC  
  /** insertsort.c **/
  // #define WCET_IS   733*(WCET_SCALE-1)
  // #define WCET_IS_1 19
  // #define WCET_IS_2 124
  // SW based
  #define WCET_IS   46500*(WCET_SCALE-1)
  #define WCET_IS_1 180
  #define WCET_IS_2 7980
  
  /** crc.c **/
  // #define WCET_CRC 31246*(WCET_SCALE-1)
  // #define WCET_CRC_1 35
  // #define WCET_CRC_2 29
  // #define WCET_CRC_3 116
  // #define WCET_CRC_4 12
  // #define WCET_CRC_5 19
  // #define WCET_CRC_6 31
  // SW based
  #define WCET_CRC 89460*(WCET_SCALE-1)
  #define WCET_CRC_1 960
  #define WCET_CRC_2 1260
  #define WCET_CRC_3 720
  #define WCET_CRC_4 1140
  #define WCET_CRC_5 960
  
  /** edn.c **/
  // #define WCET_EDN 35868*(WCET_SCALE-1)
  // #define WCET_EDN_1 755
  // #define WCET_EDN_2 9
  // #define WCET_EDN_3 31
  // #define WCET_EDN_4 9
  // #define WCET_EDN_5 32
  // #define WCET_EDN_6 310
  // #define WCET_EDN_7 43
  // #define WCET_EDN_8 213
  // #define WCET_EDN_9 56
  // #define WCET_EDN_10 15
  // #define WCET_EDN_11 47
  // #define WCET_EDN_12 23
  // #define WCET_EDN_13 88
  // #define WCET_EDN_14 258
  // #define WCET_EDN_15 18
  // SW based
  #define WCET_EDN 2795700*(WCET_SCALE-1)
  #define WCET_EDN_1 1080
  #define WCET_EDN_2 900
  #define WCET_EDN_3 2100
  #define WCET_EDN_4 660
  #define WCET_EDN_5 1860
  #define WCET_EDN_6 24720
  #define WCET_EDN_7 2820
  #define WCET_EDN_8 15960
  #define WCET_EDN_9 3300
  #define WCET_EDN_10 1500
  #define WCET_EDN_11 3420
  #define WCET_EDN_12 1860
  #define WCET_EDN_13 6480
  #define WCET_EDN_14 21240
  #define WCET_EDN_15 960
  
  /** compress.c **/
  // #define WCET_CMP 5214*(WCET_SCALE-1)
  // #define WCET_CMP_1 755
  // #define WCET_CMP_2 735
  // #define WCET_CMP_3 55
  // #define WCET_CMP_4 25
  // #define WCET_CMP_5 18
  // #define WCET_CMP_6 19
  // SW based
  #define WCET_CMP 377940*(WCET_SCALE-1)
  #define WCET_CMP_1 60180
  #define WCET_CMP_2 4920
  #define WCET_CMP_3 1920
  #define WCET_CMP_4 1380
  #define WCET_CMP_5 1020
  
  /** fir.c **/
  // #define WCET_FIR 3208*(WCET_SCALE-1)
  // #define WCET_FIR_1 45
  // #define WCET_FIR_2 16
  // #define WCET_FIR_3 10
  // #define WCET_FIR_4 103
  // SW based
  #define WCET_FIR 199920*(WCET_SCALE-1)
  #define WCET_FIR_1 3180
  #define WCET_FIR_2 960
  #define WCET_FIR_3 600
  #define WCET_FIR_4 6780
  
  /** jfdcint.c **/
  // #define WCET_JFDC 3237*(WCET_SCALE-1)
  // #define WCET_JFDC_1 898
  // #define WCET_JFDC_2 46
  // #define WCET_JFDC_3 157
  // #define WCET_JFDC_4 3
  // #define WCET_JFDC_5 143
  // #define WCET_JFDC_6 16
  // SW based
  #define WCET_JFDC 197880*(WCET_SCALE-1)
  #define WCET_JFDC_1 3240
  #define WCET_JFDC_2 13320
  #define WCET_JFDC_3 240
  #define WCET_JFDC_4 12600
  #define WCET_JFDC_5 960
  
  /** nsichneu.c **/
  // #define WCET_NSI 6148*(WCET_SCALE-1)
  // #define WCET_NSI_1 6
  // #define WCET_NSI_2 115
  // #define WCET_NSI_3 491
  // #define WCET_NSI_4 491
  // #define WCET_NSI_5 492
  // #define WCET_NSI_6 491
  // #define WCET_NSI_7 492
  // #define WCET_NSI_8 496
  // SW based
  #define WCET_NSI 489840*(WCET_SCALE-1)
  #define WCET_NSI_1 360
  #define WCET_NSI_2 9060
  #define WCET_NSI_3 39060
  #define WCET_NSI_4 39060
  #define WCET_NSI_5 39180
  #define WCET_NSI_6 39120
  #define WCET_NSI_7 39180
  #define WCET_NSI_8 39960
  
  /** statemate.c **/
  // #define WCET_STA 1787*(WCET_SCALE-1)
  // #define WCET_STA_1 135
  // #define WCET_STA_2 35
  // #define WCET_STA_3 58
  // #define WCET_STA_4 23
  // #define WCET_STA_5 106
  // #define WCET_STA_6 654
  // SW based
  #define WCET_STA 132480*(WCET_SCALE-1)
  #define WCET_STA_1 3300
  #define WCET_STA_2 4980
  #define WCET_STA_3 1680
  #define WCET_STA_4 9240
  #define WCET_STA_5 51600
#endif // LRC

#endif // __MALARDEN_H__

