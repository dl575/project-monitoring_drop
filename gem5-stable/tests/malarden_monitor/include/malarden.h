
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
#ifdef UMC_FULL
#ifdef ATOMIC

    /** insertsort.c **/ 
    #define WCET_IS 4877 *(WCET_SCALE-1)
    #define WCET_IS_1 2 
    #define WCET_IS_2 962 
                
    /** crc.c **/ 
    #define WCET_CRC 5259 *(WCET_SCALE-1)
    #define WCET_CRC_1 15 
    #define WCET_CRC_2 193 
    #define WCET_CRC_3 54 
    #define WCET_CRC_4 61 
    #define WCET_CRC_5 127 
                
    /** edn.c **/ 
    #define WCET_EDN 284808 *(WCET_SCALE-1)
    #define WCET_EDN_1 14 
    #define WCET_EDN_2 75 
    #define WCET_EDN_3 341 
    #define WCET_EDN_4 53 
    #define WCET_EDN_5 317 
    #define WCET_EDN_6 2533 
    #define WCET_EDN_7 462 
    #define WCET_EDN_8 1731 
    #define WCET_EDN_9 492 
    #define WCET_EDN_10 81 
    #define WCET_EDN_11 622 
    #define WCET_EDN_12 199 
    #define WCET_EDN_13 1009 
    #define WCET_EDN_14 2546 
    #define WCET_EDN_15 216 
                
    /** compress.c **/ 
    #define WCET_CMP 39655 *(WCET_SCALE-1)
    #define WCET_CMP_1 7055 
    #define WCET_CMP_2 525 
    #define WCET_CMP_3 183 
    #define WCET_CMP_4 104 
    #define WCET_CMP_5 238 
                
    /** fir.c **/ 
    #define WCET_FIR 13927 *(WCET_SCALE-1)
    #define WCET_FIR_1 199 
    #define WCET_FIR_2 60 
    #define WCET_FIR_3 54 
    #define WCET_FIR_4 351 
                
    /** jfdcint.c **/ 
    #define WCET_JFDC 18860 *(WCET_SCALE-1)
    #define WCET_JFDC_1 155 
    #define WCET_JFDC_2 1253 
    #define WCET_JFDC_3 26 
    #define WCET_JFDC_4 1177 
    #define WCET_JFDC_5 214 
                
    /** nsichneu.c **/ 
    #define WCET_NSI 49883 *(WCET_SCALE-1)
    #define WCET_NSI_1 5 
    #define WCET_NSI_2 907 
    #define WCET_NSI_3 4011 
    #define WCET_NSI_4 4011 
    #define WCET_NSI_5 4011 
    #define WCET_NSI_6 4013 
    #define WCET_NSI_7 4012 
    #define WCET_NSI_8 4082 
                
    /** statemate.c **/ 
    #define WCET_STA 13928 *(WCET_SCALE-1)
    #define WCET_STA_1 168 
    #define WCET_STA_2 696 
    #define WCET_STA_3 134 
    #define WCET_STA_4 952 
    #define WCET_STA_5 5511 

#endif
#ifdef TIMING

    /** insertsort.c **/ 
    #define WCET_IS 6801 *(WCET_SCALE-1)
    #define WCET_IS_1 2 
    #define WCET_IS_2 1331 
                
    /** crc.c **/ 
    #define WCET_CRC 7252 *(WCET_SCALE-1)
    #define WCET_CRC_1 44 
    #define WCET_CRC_2 285 
    #define WCET_CRC_3 76 
    #define WCET_CRC_4 83 
    #define WCET_CRC_5 176 
                
    /** edn.c **/ 
    #define WCET_EDN 408795 *(WCET_SCALE-1)
    #define WCET_EDN_1 35 
    #define WCET_EDN_2 108 
    #define WCET_EDN_3 492 
    #define WCET_EDN_4 75 
    #define WCET_EDN_5 456 
    #define WCET_EDN_6 3652 
    #define WCET_EDN_7 673 
    #define WCET_EDN_8 2483 
    #define WCET_EDN_9 712 
    #define WCET_EDN_10 113 
    #define WCET_EDN_11 917 
    #define WCET_EDN_12 296 
    #define WCET_EDN_13 1497 
    #define WCET_EDN_14 3682 
    #define WCET_EDN_15 304 
                
    /** compress.c **/ 
    #define WCET_CMP 57058 *(WCET_SCALE-1)
    #define WCET_CMP_1 10431 
    #define WCET_CMP_2 780 
    #define WCET_CMP_3 256 
    #define WCET_CMP_4 151 
    #define WCET_CMP_5 337 
                
    /** fir.c **/ 
    #define WCET_FIR 19362 *(WCET_SCALE-1)
    #define WCET_FIR_1 323 
    #define WCET_FIR_2 89 
    #define WCET_FIR_3 83 
    #define WCET_FIR_4 505 
                
    /** jfdcint.c **/ 
    #define WCET_JFDC 27442 *(WCET_SCALE-1)
    #define WCET_JFDC_1 214 
    #define WCET_JFDC_2 1834 
    #define WCET_JFDC_3 37 
    #define WCET_JFDC_4 1739 
    #define WCET_JFDC_5 304 
                
    /** nsichneu.c **/ 
    #define WCET_NSI 74185 *(WCET_SCALE-1)
    #define WCET_NSI_1 4 
    #define WCET_NSI_2 1332 
    #define WCET_NSI_3 6150 
    #define WCET_NSI_4 6091 
    #define WCET_NSI_5 6133 
    #define WCET_NSI_6 6114 
    #define WCET_NSI_7 6115 
    #define WCET_NSI_8 6133 

#endif
#endif

#ifdef UMC_SWDROP
#ifdef ATOMIC

    /** insertsort.c **/ 
    #define WCET_IS 2025 *(WCET_SCALE-1)
    #define WCET_IS_1 2 
    #define WCET_IS_2 386 
                
    /** crc.c **/ 
    #define WCET_CRC 2402 *(WCET_SCALE-1)
    #define WCET_CRC_1 15 
    #define WCET_CRC_2 65 
    #define WCET_CRC_3 22 
    #define WCET_CRC_4 29 
    #define WCET_CRC_5 47 
                
    /** edn.c **/ 
    #define WCET_EDN 103219 *(WCET_SCALE-1)
    #define WCET_EDN_1 14 
    #define WCET_EDN_2 27 
    #define WCET_EDN_3 117 
    #define WCET_EDN_4 21 
    #define WCET_EDN_5 110 
    #define WCET_EDN_6 916 
    #define WCET_EDN_7 157 
    #define WCET_EDN_8 627 
    #define WCET_EDN_9 173 
    #define WCET_EDN_10 33 
    #define WCET_EDN_11 205 
    #define WCET_EDN_12 71 
    #define WCET_EDN_13 337 
    #define WCET_EDN_14 882 
    #define WCET_EDN_15 72 
                
    /** compress.c **/ 
    #define WCET_CMP 13896 *(WCET_SCALE-1)
    #define WCET_CMP_1 2447 
    #define WCET_CMP_2 180 
    #define WCET_CMP_3 67 
    #define WCET_CMP_4 42 
    #define WCET_CMP_5 79 
                
    /** fir.c **/ 
    #define WCET_FIR 6202 *(WCET_SCALE-1)
    #define WCET_FIR_1 87 
    #define WCET_FIR_2 28 
    #define WCET_FIR_3 22 
    #define WCET_FIR_4 162 
                
    /** jfdcint.c **/ 
    #define WCET_JFDC 6940 *(WCET_SCALE-1)
    #define WCET_JFDC_1 75 
    #define WCET_JFDC_2 475 
    #define WCET_JFDC_3 10 
    #define WCET_JFDC_4 425 
    #define WCET_JFDC_5 70 
                
    /** nsichneu.c **/ 
    #define WCET_NSI 18093 *(WCET_SCALE-1)
    #define WCET_NSI_1 5 
    #define WCET_NSI_2 331 
    #define WCET_NSI_3 1451 
    #define WCET_NSI_4 1451 
    #define WCET_NSI_5 1452 
    #define WCET_NSI_6 1453 
    #define WCET_NSI_7 1452 
    #define WCET_NSI_8 1478 
                
    /** statemate.c **/ 
    #define WCET_STA 5015 *(WCET_SCALE-1)
    #define WCET_STA_1 72 
    #define WCET_STA_2 231 
    #define WCET_STA_3 54 
    #define WCET_STA_4 344 
    #define WCET_STA_5 1975 

#endif
#ifdef TIMING

    /** insertsort.c **/ 
    #define WCET_IS 3913 *(WCET_SCALE-1)
    #define WCET_IS_1 2 
    #define WCET_IS_2 756 
                
    /** crc.c **/ 
    #define WCET_CRC 4412 *(WCET_SCALE-1)
    #define WCET_CRC_1 44 
    #define WCET_CRC_2 148 
    #define WCET_CRC_3 44 
    #define WCET_CRC_4 51 
    #define WCET_CRC_5 96 
                
    /** edn.c **/ 
    #define WCET_EDN 227245 *(WCET_SCALE-1)
    #define WCET_EDN_1 26 
    #define WCET_EDN_2 60 
    #define WCET_EDN_3 286 
    #define WCET_EDN_4 43 
    #define WCET_EDN_5 257 
    #define WCET_EDN_6 2037 
    #define WCET_EDN_7 360 
    #define WCET_EDN_8 1379 
    #define WCET_EDN_9 392 
    #define WCET_EDN_10 74 
    #define WCET_EDN_11 492 
    #define WCET_EDN_12 168 
    #define WCET_EDN_13 834 
    #define WCET_EDN_14 2027 
    #define WCET_EDN_15 161 
                
    /** compress.c **/ 
    #define WCET_CMP 31827 *(WCET_SCALE-1)
    #define WCET_CMP_1 5864 
    #define WCET_CMP_2 441 
    #define WCET_CMP_3 148 
    #define WCET_CMP_4 84 
    #define WCET_CMP_5 177 
                
    /** fir.c **/ 
    #define WCET_FIR 11441 *(WCET_SCALE-1)
    #define WCET_FIR_1 210 
    #define WCET_FIR_2 57 
    #define WCET_FIR_3 51 
    #define WCET_FIR_4 297 
                
    /** jfdcint.c **/ 
    #define WCET_JFDC 15385 *(WCET_SCALE-1)
    #define WCET_JFDC_1 152 
    #define WCET_JFDC_2 1034 
    #define WCET_JFDC_3 21 
    #define WCET_JFDC_4 987 
    #define WCET_JFDC_5 159 
                
    /** nsichneu.c **/ 
    #define WCET_NSI 42228 *(WCET_SCALE-1)
    #define WCET_NSI_1 7 
    #define WCET_NSI_2 727 
    #define WCET_NSI_3 3580 
    #define WCET_NSI_4 3573 
    #define WCET_NSI_5 3571 
    #define WCET_NSI_6 3582 
    #define WCET_NSI_7 3545 
    #define WCET_NSI_8 3617 
                
    /** statemate.c **/ 
    #define WCET_STA 11583 *(WCET_SCALE-1)
    #define WCET_STA_1 189 
    #define WCET_STA_2 595 
    #define WCET_STA_3 118 
    #define WCET_STA_4 834 
    #define WCET_STA_5 4607 

#endif
#endif

/** WCET for LRC **/
#if defined LRC_FULL || LRC_SWDROP
#ifdef ATOMIC

    /** insertsort.c **/ 
    #define WCET_IS 714 *(WCET_SCALE-1)
    #define WCET_IS_1 3 
    #define WCET_IS_2 124 
                
    /** crc.c **/ 
    #define WCET_CRC 1321 *(WCET_SCALE-1)
    #define WCET_CRC_1 15 
    #define WCET_CRC_2 17 
    #define WCET_CRC_3 10 
    #define WCET_CRC_4 17 
    #define WCET_CRC_5 17 
                
    /** edn.c **/ 
    #define WCET_EDN 35124 *(WCET_SCALE-1)
    #define WCET_EDN_1 14 
    #define WCET_EDN_2 9 
    #define WCET_EDN_3 33 
    #define WCET_EDN_4 9 
    #define WCET_EDN_5 32 
    #define WCET_EDN_6 310 
    #define WCET_EDN_7 43 
    #define WCET_EDN_8 213 
    #define WCET_EDN_9 53 
    #define WCET_EDN_10 15 
    #define WCET_EDN_11 49 
    #define WCET_EDN_12 23 
    #define WCET_EDN_13 86 
    #define WCET_EDN_14 258 
    #define WCET_EDN_15 18 
                
    /** compress.c **/ 
    #define WCET_CMP 4440 *(WCET_SCALE-1)
    #define WCET_CMP_1 719 
    #define WCET_CMP_2 55 
    #define WCET_CMP_3 25 
    #define WCET_CMP_4 18 
    #define WCET_CMP_5 19 
                
    /** fir.c **/ 
    #define WCET_FIR 3208 *(WCET_SCALE-1)
    #define WCET_FIR_1 45 
    #define WCET_FIR_2 16 
    #define WCET_FIR_3 10 
    #define WCET_FIR_4 103 
                
    /** jfdcint.c **/ 
    #define WCET_JFDC 2337 *(WCET_SCALE-1)
    #define WCET_JFDC_1 46 
    #define WCET_JFDC_2 157 
    #define WCET_JFDC_3 3 
    #define WCET_JFDC_4 143 
    #define WCET_JFDC_5 16 
                
    /** nsichneu.c **/ 
    #define WCET_NSI 6158 *(WCET_SCALE-1)
    #define WCET_NSI_1 5 
    #define WCET_NSI_2 115 
    #define WCET_NSI_3 490 
    #define WCET_NSI_4 491 
    #define WCET_NSI_5 491 
    #define WCET_NSI_6 492 
    #define WCET_NSI_7 493 
    #define WCET_NSI_8 503 
                
    /** statemate.c **/ 
    #define WCET_STA 1651 *(WCET_SCALE-1)
    #define WCET_STA_1 35 
    #define WCET_STA_2 58 
    #define WCET_STA_3 23 
    #define WCET_STA_4 106 
    #define WCET_STA_5 654 

#endif
#ifdef TIMING

    /** insertsort.c **/ 
    #define WCET_IS 784 *(WCET_SCALE-1)
    #define WCET_IS_1 3 
    #define WCET_IS_2 133 
                
    /** crc.c **/ 
    #define WCET_CRC 1542 *(WCET_SCALE-1)
    #define WCET_CRC_1 42 
    #define WCET_CRC_2 22 
    #define WCET_CRC_3 12 
    #define WCET_CRC_4 35 
    #define WCET_CRC_5 15 
                
    /** edn.c **/ 
    #define WCET_EDN 45645 *(WCET_SCALE-1)
    #define WCET_EDN_1 27 
    #define WCET_EDN_2 12 
    #define WCET_EDN_3 51 
    #define WCET_EDN_4 11 
    #define WCET_EDN_5 50 
    #define WCET_EDN_6 420 
    #define WCET_EDN_7 73 
    #define WCET_EDN_8 275 
    #define WCET_EDN_9 76 
    #define WCET_EDN_10 26 
    #define WCET_EDN_11 73 
    #define WCET_EDN_12 31 
    #define WCET_EDN_13 156 
    #define WCET_EDN_14 363 
    #define WCET_EDN_15 17 
                
    /** compress.c **/ 
    #define WCET_CMP 6065 *(WCET_SCALE-1)
    #define WCET_CMP_1 1246 
    #define WCET_CMP_2 96 
    #define WCET_CMP_3 44 
    #define WCET_CMP_4 22 
    #define WCET_CMP_5 27 
                
    /** fir.c **/ 
    #define WCET_FIR 3416 *(WCET_SCALE-1)
    #define WCET_FIR_1 117 
    #define WCET_FIR_2 16 
    #define WCET_FIR_3 19 
    #define WCET_FIR_4 110 
                
    /** jfdcint.c **/ 
    #define WCET_JFDC 3211 *(WCET_SCALE-1)
    #define WCET_JFDC_1 73 
    #define WCET_JFDC_2 238 
    #define WCET_JFDC_3 4 
    #define WCET_JFDC_4 234 
    #define WCET_JFDC_5 15 
                
    /** nsichneu.c **/ 
    #define WCET_NSI 10392 *(WCET_SCALE-1)
    #define WCET_NSI_1 4 
    #define WCET_NSI_2 233 
    #define WCET_NSI_3 1022 
    #define WCET_NSI_4 1012 
    #define WCET_NSI_5 977 
    #define WCET_NSI_6 1021 
    #define WCET_NSI_7 1004 
    #define WCET_NSI_8 1022 
                
    /** statemate.c **/ 
    #define WCET_STA 2661 *(WCET_SCALE-1)
    #define WCET_STA_1 108 
    #define WCET_STA_2 131 
    #define WCET_STA_3 37 
    #define WCET_STA_4 207 
    #define WCET_STA_5 1151 

#endif
#endif

/** WCET common to all monitors **/
#if defined UMC_HWDROP || defined UMC_HWFILTER || defined LRC_HWDROP || defined LRC_HWFILTER
#ifdef ATOMIC

    /** insertsort.c **/ 
    #define WCET_IS 714 *(WCET_SCALE-1)
    #define WCET_IS_1 3 
    #define WCET_IS_2 124 
                
    /** crc.c **/ 
    #define WCET_CRC 1316 *(WCET_SCALE-1)
    #define WCET_CRC_1 13 
    #define WCET_CRC_2 18 
    #define WCET_CRC_3 10 
    #define WCET_CRC_4 17 
    #define WCET_CRC_5 15 
                
    /** edn.c **/ 
    #define WCET_EDN 35126 *(WCET_SCALE-1)
    #define WCET_EDN_1 15 
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
    #define WCET_CMP 4440 *(WCET_SCALE-1)
    #define WCET_CMP_1 718 
    #define WCET_CMP_2 55 
    #define WCET_CMP_3 25 
    #define WCET_CMP_4 18 
    #define WCET_CMP_5 19 
                
    /** fir.c **/ 
    #define WCET_FIR 3209 *(WCET_SCALE-1)
    #define WCET_FIR_1 46 
    #define WCET_FIR_2 16 
    #define WCET_FIR_3 10 
    #define WCET_FIR_4 103 
                
    /** jfdcint.c **/ 
    #define WCET_JFDC 2337 *(WCET_SCALE-1)
    #define WCET_JFDC_1 46 
    #define WCET_JFDC_2 157 
    #define WCET_JFDC_3 3 
    #define WCET_JFDC_4 143 
    #define WCET_JFDC_5 16 
                
    /** nsichneu.c **/ 
    #define WCET_NSI 6155 *(WCET_SCALE-1)
    #define WCET_NSI_1 4 
    #define WCET_NSI_2 115 
    #define WCET_NSI_3 491 
    #define WCET_NSI_4 491 
    #define WCET_NSI_5 493 
    #define WCET_NSI_6 492 
    #define WCET_NSI_7 493 
    #define WCET_NSI_8 499 
                
    /** statemate.c **/ 
    #define WCET_STA 1651 *(WCET_SCALE-1)
    #define WCET_STA_1 35 
    #define WCET_STA_2 58 
    #define WCET_STA_3 23 
    #define WCET_STA_4 106 
    #define WCET_STA_5 654 

#endif
#ifdef TIMING

    /** insertsort.c **/ 
    #define WCET_IS 784 *(WCET_SCALE-1)
    #define WCET_IS_1 3 
    #define WCET_IS_2 133 
       
    /** crc.c **/ 
    #define WCET_CRC 1541 *(WCET_SCALE-1)
    #define WCET_CRC_1 42 
    #define WCET_CRC_2 21 
    #define WCET_CRC_3 12 
    #define WCET_CRC_4 35 
    #define WCET_CRC_5 14 
       
    /** edn.c **/ 
    #define WCET_EDN 45647 *(WCET_SCALE-1)
    #define WCET_EDN_1 27 
    #define WCET_EDN_2 12 
    #define WCET_EDN_3 51 
    #define WCET_EDN_4 11 
    #define WCET_EDN_5 50 
    #define WCET_EDN_6 420 
    #define WCET_EDN_7 73 
    #define WCET_EDN_8 275 
    #define WCET_EDN_9 76 
    #define WCET_EDN_10 26 
    #define WCET_EDN_11 73 
    #define WCET_EDN_12 31 
    #define WCET_EDN_13 156 
    #define WCET_EDN_14 363 
    #define WCET_EDN_15 17 
       
    /** compress.c **/ 
    #define WCET_CMP 6068 *(WCET_SCALE-1)
    #define WCET_CMP_1 1246 
    #define WCET_CMP_2 96 
    #define WCET_CMP_3 44 
    #define WCET_CMP_4 22 
    #define WCET_CMP_5 27 
       
    /** fir.c **/ 
    #define WCET_FIR 3409 *(WCET_SCALE-1)
    #define WCET_FIR_1 108 
    #define WCET_FIR_2 16 
    #define WCET_FIR_3 19 
    #define WCET_FIR_4 107 
       
    /** jfdcint.c **/ 
    #define WCET_JFDC 3222 *(WCET_SCALE-1)
    #define WCET_JFDC_1 82 
    #define WCET_JFDC_2 229 
    #define WCET_JFDC_3 4 
    #define WCET_JFDC_4 243 
    #define WCET_JFDC_5 15 
       
    /** nsichneu.c **/ 
    #define WCET_NSI 10662 *(WCET_SCALE-1)
    #define WCET_NSI_1 3 
    #define WCET_NSI_2 251 
    #define WCET_NSI_3 1022 
    #define WCET_NSI_4 985 
    #define WCET_NSI_5 1004 
    #define WCET_NSI_6 994 
    #define WCET_NSI_7 986 
    #define WCET_NSI_8 1008 
       
    /** statemate.c **/ 
    #define WCET_STA 2672 *(WCET_SCALE-1)
    #define WCET_STA_1 117 
    #define WCET_STA_2 122 
    #define WCET_STA_3 37 
    #define WCET_STA_4 198 
    #define WCET_STA_5 1160 

#endif
#endif



/** Old Data: Needs to be deleted **/

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

