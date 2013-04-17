
#ifndef __MALARDEN_H__
#define __MALARDEN_H__

#include "../../monitoring/timer.h"
#include "../../monitoring/monitoring.h"


#ifndef WCET_SCALE
    #define WCET_SCALE 1
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
  #define WCET_IS  956*(WCET_SCALE-1)
  #define WCET_IS_1 19
  #define WCET_IS_2 170
  
  /** crc.c **/
  #define WCET_CRC 31253*(WCET_SCALE-1)
  #define WCET_CRC_1 37
  #define WCET_CRC_2 32
  #define WCET_CRC_3 116
  #define WCET_CRC_4 12
  #define WCET_CRC_5 19
  #define WCET_CRC_6 33
  
  /** edn.c **/
  #define WCET_EDN 35861*(WCET_SCALE-1)
  #define WCET_EDN_1 751
  #define WCET_EDN_2 9
  #define WCET_EDN_3 31
  #define WCET_EDN_4 9
  #define WCET_EDN_5 32
  #define WCET_EDN_6 310
  #define WCET_EDN_7 43
  #define WCET_EDN_8 213
  #define WCET_EDN_9 56
  #define WCET_EDN_10 15
  #define WCET_EDN_11 47
  #define WCET_EDN_12 23
  #define WCET_EDN_13 88
  #define WCET_EDN_14 258
  #define WCET_EDN_15 18
  
  /** compress.c **/
  #define WCET_CMP 5161*(WCET_SCALE-1)
  #define WCET_CMP_1 706
  #define WCET_CMP_2 735
  #define WCET_CMP_3 55
  #define WCET_CMP_4 18
  #define WCET_CMP_5 25
  #define WCET_CMP_6 19
  
  /** fir.c **/
  #define WCET_FIR 3209*(WCET_SCALE-1)
  #define WCET_FIR_1 46
  #define WCET_FIR_2 16
  #define WCET_FIR_3 10
  #define WCET_FIR_4 103
  
  /** jfdcint.c **/
  #define WCET_JFDC 3554*(WCET_SCALE-1)
  #define WCET_JFDC_1 1218
  #define WCET_JFDC_2 45
  #define WCET_JFDC_3 157
  #define WCET_JFDC_4 3
  #define WCET_JFDC_5 143
  #define WCET_JFDC_6 16
  
  /** nsichneu.c **/
  #define WCET_NSI 6155*(WCET_SCALE-1)
  #define WCET_NSI_1 4
  #define WCET_NSI_2 115
  #define WCET_NSI_3 491
  #define WCET_NSI_4 491
  #define WCET_NSI_5 493
  #define WCET_NSI_6 492
  #define WCET_NSI_7 493
  #define WCET_NSI_8 499
  
  /** statemate.c **/
  #define WCET_STA 1785*(WCET_SCALE-1)
  #define WCET_STA_1 134
  #define WCET_STA_2 36
  #define WCET_STA_3 57
  #define WCET_STA_4 23
  #define WCET_STA_5 106
  #define WCET_STA_6 654
#endif // UMC


/** WCET for LRC **/
#if 0 
  /** insertsort.c **/
  // No monitoring: 733
  #define WCET_IS   957*(WCET_SCALE-1)
  #define WCET_IS_1 19
  #define WCET_IS_2 171
  
  /** crc.c **/
  // No monitoring: 31246
  #define WCET_CRC 31254*(WCET_SCALE-1)
  #define WCET_CRC_1 37
  #define WCET_CRC_2 32
  #define WCET_CRC_3 116
  #define WCET_CRC_4 12
  #define WCET_CRC_5 19
  #define WCET_CRC_6 33
  
  /** edn.c **/
  // No monitoring: 35870
  #define WCET_EDN 35862*(WCET_SCALE-1)
  #define WCET_EDN_1 751
  #define WCET_EDN_2 9
  #define WCET_EDN_3 31
  #define WCET_EDN_4 9
  #define WCET_EDN_5 32
  #define WCET_EDN_6 310
  #define WCET_EDN_7 43
  #define WCET_EDN_8 213
  #define WCET_EDN_9 56
  #define WCET_EDN_10 15
  #define WCET_EDN_11 47
  #define WCET_EDN_12 23
  #define WCET_EDN_13 88
  #define WCET_EDN_14 258
  #define WCET_EDN_15 18
  
  /** compress.c **/
  // No monitoring: 5213
  #define WCET_CMP 5159*(WCET_SCALE-1)
  #define WCET_CMP_1 703
  #define WCET_CMP_2 735
  #define WCET_CMP_3 55
  #define WCET_CMP_4 25
  #define WCET_CMP_5 18
  #define WCET_CMP_6 19
  
  /** fir.c **/
  // No monitoring: 3208
  #define WCET_FIR 3210*(WCET_SCALE-1)
  #define WCET_FIR_1 46
  #define WCET_FIR_2 16
  #define WCET_FIR_3 10
  #define WCET_FIR_4 103
  
  /** jfdcint.c **/
  // No monitoring: 3304
  #define WCET_JFDC 3554*(WCET_SCALE-1)
  #define WCET_JFDC_1 1218
  #define WCET_JFDC_2 45
  #define WCET_JFDC_3 157
  #define WCET_JFDC_4 3
  #define WCET_JFDC_5 143
  #define WCET_JFDC_6 16
  
  /** nsichneu.c **/
  // No monitoring: 6157
  #define WCET_NSI 6158*(WCET_SCALE-1)
  #define WCET_NSI_1 3
  #define WCET_NSI_2 116
  #define WCET_NSI_3 493
  #define WCET_NSI_4 492
  #define WCET_NSI_5 493
  #define WCET_NSI_6 492
  #define WCET_NSI_7 493
  #define WCET_NSI_8 497
  
  /** statemate.c **/
  // No monitoring: 1809
  #define WCET_STA 1785*(WCET_SCALE-1)
  #define WCET_STA_1 134
  #define WCET_STA_2 36
  #define WCET_STA_3 57
  #define WCET_STA_4 23
  #define WCET_STA_5 106
  #define WCET_STA_6 654
#endif // LRC

/** WCET for LRC **/
#ifdef LRC  
  /** insertsort.c **/
  // No monitoring: 733
  #define WCET_IS   733*(WCET_SCALE-1)
  #define WCET_IS_1 19
  #define WCET_IS_2 124
  
  /** crc.c **/
  // No monitoring: 31246
  #define WCET_CRC 31246*(WCET_SCALE-1)
  #define WCET_CRC_1 35
  #define WCET_CRC_2 29
  #define WCET_CRC_3 116
  #define WCET_CRC_4 12
  #define WCET_CRC_5 19
  #define WCET_CRC_6 31
  
  /** edn.c **/
  // No monitoring: 35870
  #define WCET_EDN 35868*(WCET_SCALE-1)
  #define WCET_EDN_1 755
  #define WCET_EDN_2 9
  #define WCET_EDN_3 31
  #define WCET_EDN_4 9
  #define WCET_EDN_5 32
  #define WCET_EDN_6 310
  #define WCET_EDN_7 43
  #define WCET_EDN_8 213
  #define WCET_EDN_9 56
  #define WCET_EDN_10 15
  #define WCET_EDN_11 47
  #define WCET_EDN_12 23
  #define WCET_EDN_13 88
  #define WCET_EDN_14 258
  #define WCET_EDN_15 18
  
  /** compress.c **/
  // No monitoring: 5213
  #define WCET_CMP 5214*(WCET_SCALE-1)
  #define WCET_CMP_1 755
  #define WCET_CMP_2 735
  #define WCET_CMP_3 55
  #define WCET_CMP_4 25
  #define WCET_CMP_5 18
  #define WCET_CMP_6 19
  
  /** fir.c **/
  // No monitoring: 3208
  #define WCET_FIR 3208*(WCET_SCALE-1)
  #define WCET_FIR_1 45
  #define WCET_FIR_2 16
  #define WCET_FIR_3 10
  #define WCET_FIR_4 103
  
  /** jfdcint.c **/
  // No monitoring: 3304
  #define WCET_JFDC 3237*(WCET_SCALE-1)
  #define WCET_JFDC_1 898
  #define WCET_JFDC_2 46
  #define WCET_JFDC_3 157
  #define WCET_JFDC_4 3
  #define WCET_JFDC_5 143
  #define WCET_JFDC_6 16
  
  /** nsichneu.c **/
  // No monitoring: 6157
  #define WCET_NSI 6148*(WCET_SCALE-1)
  #define WCET_NSI_1 6
  #define WCET_NSI_2 115
  #define WCET_NSI_3 491
  #define WCET_NSI_4 491
  #define WCET_NSI_5 492
  #define WCET_NSI_6 491
  #define WCET_NSI_7 492
  #define WCET_NSI_8 496
  
  /** statemate.c **/
  // No monitoring: 1809
  #define WCET_STA 1787*(WCET_SCALE-1)
  #define WCET_STA_1 135
  #define WCET_STA_2 35
  #define WCET_STA_3 58
  #define WCET_STA_4 23
  #define WCET_STA_5 106
  #define WCET_STA_6 654
#endif // LRC



#endif // __MALARDEN_H__

