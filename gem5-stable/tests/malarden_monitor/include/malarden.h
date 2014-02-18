
#ifndef __MALARDEN_H__
#define __MALARDEN_H__

#include "../../monitoring/timer.h"
#include "../../monitoring/monitoring_wcet.h"


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

#ifdef ATOMIC
#ifdef UMC_FULL
	#define WCET_IS_1 8
	#define WCET_IS_2 801
	#define WCET_IS 4111*(WCET_SCALE-1)
	#define WCET_CRC_1 16
	#define WCET_CRC_3 48
	#define WCET_CRC_4 55
	#define WCET_CRC_5 111
	#define WCET_CRC_2 170
	#define WCET_CRC 4649*(WCET_SCALE-1)
	#define WCET_EDN_1 17
	#define WCET_EDN_2 66
	#define WCET_EDN_3 150
	#define WCET_EDN_4 46
	#define WCET_EDN_5 169
	#define WCET_EDN_6 2247
	#define WCET_EDN_7 377
	#define WCET_EDN_8 1550
	#define WCET_EDN_9 452
	#define WCET_EDN_10 73
	#define WCET_EDN_11 502
	#define WCET_EDN_12 173
	#define WCET_EDN_13 870
	#define WCET_EDN_14 2308
	#define WCET_EDN_15 189
	#define WCET_EDN 252312*(WCET_SCALE-1)
	#define WCET_CMP_1 5954
	#define WCET_CMP_2 421
	#define WCET_CMP_3 209
	#define WCET_CMP_4 68
	#define WCET_CMP_5 208
	#define WCET_CMP 35071*(WCET_SCALE-1)
	#define WCET_FIR_1 138
	#define WCET_FIR_2 50
	#define WCET_FIR_3 47
	#define WCET_FIR_4 310
	#define WCET_FIR 12313*(WCET_SCALE-1)
	#define WCET_JFDC_1 41
	#define WCET_JFDC_2 961
	#define WCET_JFDC_3 62
	#define WCET_JFDC_4 923
	#define WCET_JFDC_5 187
	#define WCET_JFDC 15308*(WCET_SCALE-1)
	#define WCET_NSI_1 9
	#define WCET_NSI_2 769
	#define WCET_NSI_3 3401
	#define WCET_NSI_4 3401
	#define WCET_NSI_5 3401
	#define WCET_NSI_6 3401
	#define WCET_NSI_7 3421
	#define WCET_NSI_8 3423
	#define WCET_NSI 42276*(WCET_SCALE-1)
	#define WCET_STA_1 299
	#define WCET_STA_2 911
	#define WCET_STA_3 230
	#define WCET_STA_4 940
	#define WCET_STA_5 6904
	#define WCET_STA 17300*(WCET_SCALE-1)
#endif
#ifdef UMC_SWDROP
	#define WCET_IS_1 8
	#define WCET_IS_2 297
	#define WCET_IS 1582*(WCET_SCALE-1)
	#define WCET_CRC_1 16
	#define WCET_CRC_3 20
	#define WCET_CRC_4 27
	#define WCET_CRC_5 41
	#define WCET_CRC_2 58
	#define WCET_CRC 2209*(WCET_SCALE-1)
	#define WCET_EDN_1 17
	#define WCET_EDN_2 24
	#define WCET_EDN_3 52
	#define WCET_EDN_4 18
	#define WCET_EDN_5 57
	#define WCET_EDN_6 814
	#define WCET_EDN_7 125
	#define WCET_EDN_8 598
	#define WCET_EDN_9 158
	#define WCET_EDN_10 31
	#define WCET_EDN_11 167
	#define WCET_EDN_12 61
	#define WCET_EDN_13 282
	#define WCET_EDN_14 782
	#define WCET_EDN_15 63
	#define WCET_EDN 93570*(WCET_SCALE-1)
	#define WCET_CMP_1 1894
	#define WCET_CMP_2 141
	#define WCET_CMP_3 69
	#define WCET_CMP_4 26
	#define WCET_CMP_5 68
	#define WCET_CMP 11367*(WCET_SCALE-1)
	#define WCET_FIR_1 68
	#define WCET_FIR_2 22
	#define WCET_FIR_3 19
	#define WCET_FIR_4 134
	#define WCET_FIR 5195*(WCET_SCALE-1)
	#define WCET_JFDC_1 27
	#define WCET_JFDC_2 359
	#define WCET_JFDC_3 20
	#define WCET_JFDC_4 349
	#define WCET_JFDC_5 61
	#define WCET_JFDC 5760*(WCET_SCALE-1)
	#define WCET_NSI_1 9
	#define WCET_NSI_2 265
	#define WCET_NSI_3 1161
	#define WCET_NSI_4 1161
	#define WCET_NSI_5 1161
	#define WCET_NSI_6 1161
	#define WCET_NSI_7 1167
	#define WCET_NSI_8 1169
	#define WCET_NSI 14458*(WCET_SCALE-1)
	#define WCET_STA_1 117
	#define WCET_STA_2 281
	#define WCET_STA_3 76
	#define WCET_STA_4 296
	#define WCET_STA_5 2102
	#define WCET_STA 5330*(WCET_SCALE-1)
#endif
#ifdef LRC_FULL
	#define WCET_IS_1 8
	#define WCET_IS_2 117
	#define WCET_IS 673*(WCET_SCALE-1)
	#define WCET_CRC_1 16
	#define WCET_CRC_3 10
	#define WCET_CRC_4 17
	#define WCET_CRC_5 16
	#define WCET_CRC_2 18
	#define WCET_CRC 1324*(WCET_SCALE-1)
	#define WCET_EDN_1 17
	#define WCET_EDN_2 9
	#define WCET_EDN_3 17
	#define WCET_EDN_4 8
	#define WCET_EDN_5 17
	#define WCET_EDN_6 309
	#define WCET_EDN_7 35
	#define WCET_EDN_8 258
	#define WCET_EDN_9 53
	#define WCET_EDN_10 16
	#define WCET_EDN_11 47
	#define WCET_EDN_12 21
	#define WCET_EDN_13 72
	#define WCET_EDN_14 237
	#define WCET_EDN_15 18
	#define WCET_EDN 36890*(WCET_SCALE-1)
	#define WCET_CMP_1 444
	#define WCET_CMP_2 45
	#define WCET_CMP_3 19
	#define WCET_CMP_4 11
	#define WCET_CMP_5 18
	#define WCET_CMP 3227*(WCET_SCALE-1)
	#define WCET_FIR_1 43
	#define WCET_FIR_2 12
	#define WCET_FIR_3 9
	#define WCET_FIR_4 70
	#define WCET_FIR 2680*(WCET_SCALE-1)
	#define WCET_JFDC_1 22
	#define WCET_JFDC_2 145
	#define WCET_JFDC_3 5
	#define WCET_JFDC_4 144
	#define WCET_JFDC_5 16
	#define WCET_JFDC 2350*(WCET_SCALE-1)
	#define WCET_NSI_1 2
	#define WCET_NSI_2 85
	#define WCET_NSI_3 361
	#define WCET_NSI_4 361
	#define WCET_NSI_5 361
	#define WCET_NSI_6 361
	#define WCET_NSI_7 362
	#define WCET_NSI_8 365
	#define WCET_NSI 4516*(WCET_SCALE-1)
	#define WCET_STA_1 52
	#define WCET_STA_2 56
	#define WCET_STA_3 21
	#define WCET_STA_4 66
	#define WCET_STA_5 387
	#define WCET_STA 1055*(WCET_SCALE-1)
#endif
#ifdef LRC_SWDROP
	#define WCET_IS_1 8
	#define WCET_IS_2 117
	#define WCET_IS 673*(WCET_SCALE-1)
	#define WCET_CRC_1 16
	#define WCET_CRC_3 10
	#define WCET_CRC_4 17
	#define WCET_CRC_5 16
	#define WCET_CRC_2 18
	#define WCET_CRC 1324*(WCET_SCALE-1)
	#define WCET_EDN_1 17
	#define WCET_EDN_2 9
	#define WCET_EDN_3 17
	#define WCET_EDN_4 8
	#define WCET_EDN_5 17
	#define WCET_EDN_6 309
	#define WCET_EDN_7 35
	#define WCET_EDN_8 258
	#define WCET_EDN_9 53
	#define WCET_EDN_10 16
	#define WCET_EDN_11 47
	#define WCET_EDN_12 21
	#define WCET_EDN_13 72
	#define WCET_EDN_14 237
	#define WCET_EDN_15 18
	#define WCET_EDN 36890*(WCET_SCALE-1)
	#define WCET_CMP_1 444
	#define WCET_CMP_2 45
	#define WCET_CMP_3 19
	#define WCET_CMP_4 11
	#define WCET_CMP_5 18
	#define WCET_CMP 3227*(WCET_SCALE-1)
	#define WCET_FIR_1 43
	#define WCET_FIR_2 12
	#define WCET_FIR_3 9
	#define WCET_FIR_4 70
	#define WCET_FIR 2680*(WCET_SCALE-1)
	#define WCET_JFDC_1 22
	#define WCET_JFDC_2 145
	#define WCET_JFDC_3 5
	#define WCET_JFDC_4 144
	#define WCET_JFDC_5 16
	#define WCET_JFDC 2350*(WCET_SCALE-1)
	#define WCET_NSI_1 2
	#define WCET_NSI_2 85
	#define WCET_NSI_3 361
	#define WCET_NSI_4 361
	#define WCET_NSI_5 361
	#define WCET_NSI_6 361
	#define WCET_NSI_7 362
	#define WCET_NSI_8 365
	#define WCET_NSI 4516*(WCET_SCALE-1)
	#define WCET_STA_1 52
	#define WCET_STA_2 56
	#define WCET_STA_3 21
	#define WCET_STA_4 66
	#define WCET_STA_5 387
	#define WCET_STA 1055*(WCET_SCALE-1)
#endif
#if defined UMC_HWDROP || defined UMC_HWFILTER || defined LRC_HWDROP || defined LRC_HWFILTER || defined DIFT_HWDROP || defined DIFT_HWFILTER || defined DIFT_RF_HWDROP || defined DIFT_RF_HWFILTER
	#define WCET_IS_1 9
	#define WCET_IS_2 117
	#define WCET_IS 673*(WCET_SCALE-1)
	#define WCET_CRC_1 15
	#define WCET_CRC_3 10
	#define WCET_CRC_4 17
	#define WCET_CRC_5 16
	#define WCET_CRC_2 18
	#define WCET_CRC 1323*(WCET_SCALE-1)
	#define WCET_EDN_1 17
	#define WCET_EDN_2 9
	#define WCET_EDN_3 17
	#define WCET_EDN_4 8
	#define WCET_EDN_5 17
	#define WCET_EDN_6 309
	#define WCET_EDN_7 35
	#define WCET_EDN_8 258
	#define WCET_EDN_9 53
	#define WCET_EDN_10 16
	#define WCET_EDN_11 47
	#define WCET_EDN_12 21
	#define WCET_EDN_13 72
	#define WCET_EDN_14 237
	#define WCET_EDN_15 18
	#define WCET_EDN 36890*(WCET_SCALE-1)
	#define WCET_CMP_1 444
	#define WCET_CMP_2 45
	#define WCET_CMP_3 19
	#define WCET_CMP_4 11
	#define WCET_CMP_5 18
	#define WCET_CMP 3227*(WCET_SCALE-1)
	#define WCET_FIR_1 43
	#define WCET_FIR_2 12
	#define WCET_FIR_3 9
	#define WCET_FIR_4 70
	#define WCET_FIR 2680*(WCET_SCALE-1)
	#define WCET_JFDC_1 22
	#define WCET_JFDC_2 145
	#define WCET_JFDC_3 5
	#define WCET_JFDC_4 144
	#define WCET_JFDC_5 16
	#define WCET_JFDC 2350*(WCET_SCALE-1)
	#define WCET_NSI_1 2
	#define WCET_NSI_2 85
	#define WCET_NSI_3 361
	#define WCET_NSI_4 361
	#define WCET_NSI_5 361
	#define WCET_NSI_6 362
	#define WCET_NSI_7 362
	#define WCET_NSI_8 365
	#define WCET_NSI 4519*(WCET_SCALE-1)
	#define WCET_STA_1 52
	#define WCET_STA_2 56
	#define WCET_STA_3 21
	#define WCET_STA_4 66
	#define WCET_STA_5 387
	#define WCET_STA 1055*(WCET_SCALE-1)
#endif
#ifdef DIFT_FULL
	#define WCET_IS_1 8
	#define WCET_IS_2 2847
	#define WCET_IS 15830*(WCET_SCALE-1)
	#define WCET_CRC_1 16
	#define WCET_CRC_3 192
	#define WCET_CRC_4 382
	#define WCET_CRC_5 302
	#define WCET_CRC_2 407
	#define WCET_CRC 28338*(WCET_SCALE-1)
	#define WCET_EDN_1 43
	#define WCET_EDN_2 191
	#define WCET_EDN_3 433
	#define WCET_EDN_4 164
	#define WCET_EDN_5 407
	#define WCET_EDN_6 6991
	#define WCET_EDN_7 893
	#define WCET_EDN_8 6498
	#define WCET_EDN_9 1353
	#define WCET_EDN_10 380
	#define WCET_EDN_11 1190
	#define WCET_EDN_12 515
	#define WCET_EDN_13 1892
	#define WCET_EDN_14 6217
	#define WCET_EDN_15 460
	#define WCET_EDN 878302*(WCET_SCALE-1)
	#define WCET_CMP_1 10662
	#define WCET_CMP_2 981
	#define WCET_CMP_3 435
	#define WCET_CMP_4 219
	#define WCET_CMP_5 408
	#define WCET_CMP 72803*(WCET_SCALE-1)
	#define WCET_FIR_1 719
	#define WCET_FIR_2 272
	#define WCET_FIR_3 191
	#define WCET_FIR_4 1421
	#define WCET_FIR 56214*(WCET_SCALE-1)
	#define WCET_JFDC_1 152
	#define WCET_JFDC_2 3863
	#define WCET_JFDC_3 109
	#define WCET_JFDC_4 3836
	#define WCET_JFDC_5 406
	#define WCET_JFDC 62072*(WCET_SCALE-1)
	#define WCET_NSI_1 4
	#define WCET_NSI_2 1645
	#define WCET_NSI_3 7121
	#define WCET_NSI_4 7121
	#define WCET_NSI_5 7121
	#define WCET_NSI_6 7148
	#define WCET_NSI_7 7148
	#define WCET_NSI_8 7203
	#define WCET_NSI 88681*(WCET_SCALE-1)
	#define WCET_STA_1 754
	#define WCET_STA_2 1434
	#define WCET_STA_3 463
	#define WCET_STA_4 1626
	#define WCET_STA_5 10163
	#define WCET_STA 26613*(WCET_SCALE-1)
#endif
#ifdef DIFT_SWDROP
	#define WCET_IS_1 8
	#define WCET_IS_2 1797
	#define WCET_IS 10000*(WCET_SCALE-1)
	#define WCET_CRC_1 16
	#define WCET_CRC_3 122
	#define WCET_CRC_4 241
	#define WCET_CRC_5 193
	#define WCET_CRC_2 257
	#define WCET_CRC 17948*(WCET_SCALE-1)
	#define WCET_EDN_1 33
	#define WCET_EDN_2 121
	#define WCET_EDN_3 273
	#define WCET_EDN_4 104
	#define WCET_EDN_5 257
	#define WCET_EDN_6 4421
	#define WCET_EDN_7 563
	#define WCET_EDN_8 4098
	#define WCET_EDN_9 853
	#define WCET_EDN_10 240
	#define WCET_EDN_11 750
	#define WCET_EDN_12 325
	#define WCET_EDN_13 1192
	#define WCET_EDN_14 3917
	#define WCET_EDN_15 290
	#define WCET_EDN 554682*(WCET_SCALE-1)
	#define WCET_CMP_1 6732
	#define WCET_CMP_2 621
	#define WCET_CMP_3 275
	#define WCET_CMP_4 139
	#define WCET_CMP_5 258
	#define WCET_CMP 46043*(WCET_SCALE-1)
	#define WCET_FIR_1 459
	#define WCET_FIR_2 172
	#define WCET_FIR_3 121
	#define WCET_FIR_4 901
	#define WCET_FIR 35624*(WCET_SCALE-1)
	#define WCET_JFDC_1 102
	#define WCET_JFDC_2 2433
	#define WCET_JFDC_3 69
	#define WCET_JFDC_4 2416
	#define WCET_JFDC_5 256
	#define WCET_JFDC 39102*(WCET_SCALE-1)
	#define WCET_NSI_1 4
	#define WCET_NSI_2 1045
	#define WCET_NSI_3 4521
	#define WCET_NSI_4 4521
	#define WCET_NSI_5 4521
	#define WCET_NSI_6 4538
	#define WCET_NSI_7 4538
	#define WCET_NSI_8 4573
	#define WCET_NSI 56311*(WCET_SCALE-1)
	#define WCET_STA_1 484
	#define WCET_STA_2 904
	#define WCET_STA_3 293
	#define WCET_STA_4 1026
	#define WCET_STA_5 6403
	#define WCET_STA 16783*(WCET_SCALE-1)
#endif
#endif
#ifdef TIMING
#ifdef UMC_FULL
	#define WCET_IS_1 9
	#define WCET_IS_2 1206
	#define WCET_IS 6216*(WCET_SCALE-1)
	#define WCET_CRC_1 54
	#define WCET_CRC_3 72
	#define WCET_CRC_4 88
	#define WCET_CRC_5 197
	#define WCET_CRC_2 260
	#define WCET_CRC 6848*(WCET_SCALE-1)
	#define WCET_EDN_1 47
	#define WCET_EDN_2 102
	#define WCET_EDN_3 238
	#define WCET_EDN_4 70
	#define WCET_EDN_5 259
	#define WCET_EDN_6 3471
	#define WCET_EDN_7 594
	#define WCET_EDN_8 2365
	#define WCET_EDN_9 713
	#define WCET_EDN_10 107
	#define WCET_EDN_11 809
	#define WCET_EDN_12 277
	#define WCET_EDN_13 1418
	#define WCET_EDN_14 3610
	#define WCET_EDN_15 287
	#define WCET_EDN 387231*(WCET_SCALE-1)
	#define WCET_CMP_1 9737
	#define WCET_CMP_2 709
	#define WCET_CMP_3 369
	#define WCET_CMP_4 135
	#define WCET_CMP_5 317
	#define WCET_CMP 57084*(WCET_SCALE-1)
	#define WCET_FIR_1 223
	#define WCET_FIR_2 81
	#define WCET_FIR_3 78
	#define WCET_FIR_4 455
	#define WCET_FIR 18237*(WCET_SCALE-1)
	#define WCET_JFDC_1 77
	#define WCET_JFDC_2 1558
	#define WCET_JFDC_3 67
	#define WCET_JFDC_4 1456
	#define WCET_JFDC_5 294
	#define WCET_JFDC 23460*(WCET_SCALE-1)
	#define WCET_NSI_1 5
	#define WCET_NSI_2 1260
	#define WCET_NSI_3 5731
	#define WCET_NSI_4 5690
	#define WCET_NSI_5 5681
	#define WCET_NSI_6 5713
	#define WCET_NSI_7 5740
	#define WCET_NSI_8 5697
	#define WCET_NSI 68775*(WCET_SCALE-1)
	#define WCET_STA_1 459
	#define WCET_STA_2 1554
	#define WCET_STA_3 401
	#define WCET_STA_4 1555
	#define WCET_STA_5 11277
	#define WCET_STA 28036*(WCET_SCALE-1)
#endif
#ifdef UMC_SWDROP
	#define WCET_IS_1 9
	#define WCET_IS_2 675
	#define WCET_IS 3489*(WCET_SCALE-1)
	#define WCET_CRC_1 63
	#define WCET_CRC_3 42
	#define WCET_CRC_4 49
	#define WCET_CRC_5 99
	#define WCET_CRC_2 141
	#define WCET_CRC 4216*(WCET_SCALE-1)
	#define WCET_EDN_1 38
	#define WCET_EDN_2 57
	#define WCET_EDN_3 124
	#define WCET_EDN_4 49
	#define WCET_EDN_5 166
	#define WCET_EDN_6 1934
	#define WCET_EDN_7 324
	#define WCET_EDN_8 1336
	#define WCET_EDN_9 407
	#define WCET_EDN_10 71
	#define WCET_EDN_11 440
	#define WCET_EDN_12 148
	#define WCET_EDN_13 763
	#define WCET_EDN_14 1968
	#define WCET_EDN_15 161
	#define WCET_EDN 216861*(WCET_SCALE-1)
	#define WCET_CMP_1 5372
	#define WCET_CMP_2 378
	#define WCET_CMP_3 179
	#define WCET_CMP_4 59
	#define WCET_CMP_5 167
	#define WCET_CMP 30370*(WCET_SCALE-1)
	#define WCET_FIR_1 187
	#define WCET_FIR_2 42
	#define WCET_FIR_3 48
	#define WCET_FIR_4 275
	#define WCET_FIR 10656*(WCET_SCALE-1)
	#define WCET_JFDC_1 71
	#define WCET_JFDC_2 913
	#define WCET_JFDC_3 53
	#define WCET_JFDC_4 881
	#define WCET_JFDC_5 150
	#define WCET_JFDC 13447*(WCET_SCALE-1)
	#define WCET_NSI_1 3
	#define WCET_NSI_2 697
	#define WCET_NSI_3 3308
	#define WCET_NSI_4 3281
	#define WCET_NSI_5 3290
	#define WCET_NSI_6 3299
	#define WCET_NSI_7 3325
	#define WCET_NSI_8 3275
	#define WCET_NSI 39341*(WCET_SCALE-1)
	#define WCET_STA_1 330
	#define WCET_STA_2 847
	#define WCET_STA_3 206
	#define WCET_STA_4 856
	#define WCET_STA_5 6127
	#define WCET_STA 15153*(WCET_SCALE-1)
#endif
#ifdef LRC_FULL
	#define WCET_IS_1 9
	#define WCET_IS_2 126
	#define WCET_IS 745*(WCET_SCALE-1)
	#define WCET_CRC_1 54
	#define WCET_CRC_3 21
	#define WCET_CRC_4 35
	#define WCET_CRC_5 33
	#define WCET_CRC_2 21
	#define WCET_CRC 1576*(WCET_SCALE-1)
	#define WCET_EDN_1 38
	#define WCET_EDN_2 21
	#define WCET_EDN_3 28
	#define WCET_EDN_4 10
	#define WCET_EDN_5 55
	#define WCET_EDN_6 419
	#define WCET_EDN_7 50
	#define WCET_EDN_8 315
	#define WCET_EDN_9 82
	#define WCET_EDN_10 17
	#define WCET_EDN_11 89
	#define WCET_EDN_12 37
	#define WCET_EDN_13 148
	#define WCET_EDN_14 339
	#define WCET_EDN_15 17
	#define WCET_EDN 47004*(WCET_SCALE-1)
	#define WCET_CMP_1 1006
	#define WCET_CMP_2 78
	#define WCET_CMP_3 40
	#define WCET_CMP_4 14
	#define WCET_CMP_5 17
	#define WCET_CMP 5175*(WCET_SCALE-1)
	#define WCET_FIR_1 103
	#define WCET_FIR_2 21
	#define WCET_FIR_3 18
	#define WCET_FIR_4 84
	#define WCET_FIR 2879*(WCET_SCALE-1)
	#define WCET_JFDC_1 47
	#define WCET_JFDC_2 268
	#define WCET_JFDC_3 16
	#define WCET_JFDC_4 256
	#define WCET_JFDC_5 15
	#define WCET_JFDC 3200*(WCET_SCALE-1)
	#define WCET_NSI_1 4
	#define WCET_NSI_2 231
	#define WCET_NSI_3 910
	#define WCET_NSI_4 901
	#define WCET_NSI_5 881
	#define WCET_NSI_6 890
	#define WCET_NSI_7 883
	#define WCET_NSI_8 884
	#define WCET_NSI 9242*(WCET_SCALE-1)
	#define WCET_STA_1 149
	#define WCET_STA_2 172
	#define WCET_STA_3 41
	#define WCET_STA_4 166
	#define WCET_STA_5 973
	#define WCET_STA 2342*(WCET_SCALE-1)
#endif
#ifdef LRC_SWDROP
	#define WCET_IS_1 9
	#define WCET_IS_2 126
	#define WCET_IS 745*(WCET_SCALE-1)
	#define WCET_CRC_1 54
	#define WCET_CRC_3 21
	#define WCET_CRC_4 35
	#define WCET_CRC_5 33
	#define WCET_CRC_2 21
	#define WCET_CRC 1576*(WCET_SCALE-1)
	#define WCET_EDN_1 38
	#define WCET_EDN_2 21
	#define WCET_EDN_3 28
	#define WCET_EDN_4 10
	#define WCET_EDN_5 55
	#define WCET_EDN_6 419
	#define WCET_EDN_7 50
	#define WCET_EDN_8 315
	#define WCET_EDN_9 82
	#define WCET_EDN_10 17
	#define WCET_EDN_11 89
	#define WCET_EDN_12 37
	#define WCET_EDN_13 148
	#define WCET_EDN_14 339
	#define WCET_EDN_15 17
	#define WCET_EDN 47004*(WCET_SCALE-1)
	#define WCET_CMP_1 1006
	#define WCET_CMP_2 78
	#define WCET_CMP_3 40
	#define WCET_CMP_4 14
	#define WCET_CMP_5 17
	#define WCET_CMP 5175*(WCET_SCALE-1)
	#define WCET_FIR_1 103
	#define WCET_FIR_2 21
	#define WCET_FIR_3 18
	#define WCET_FIR_4 84
	#define WCET_FIR 2879*(WCET_SCALE-1)
	#define WCET_JFDC_1 47
	#define WCET_JFDC_2 268
	#define WCET_JFDC_3 16
	#define WCET_JFDC_4 256
	#define WCET_JFDC_5 15
	#define WCET_JFDC 3200*(WCET_SCALE-1)
	#define WCET_NSI_1 4
	#define WCET_NSI_2 231
	#define WCET_NSI_3 910
	#define WCET_NSI_4 901
	#define WCET_NSI_5 881
	#define WCET_NSI_6 890
	#define WCET_NSI_7 883
	#define WCET_NSI_8 884
	#define WCET_NSI 9242*(WCET_SCALE-1)
	#define WCET_STA_1 149
	#define WCET_STA_2 172
	#define WCET_STA_3 41
	#define WCET_STA_4 166
	#define WCET_STA_5 973
	#define WCET_STA 2342*(WCET_SCALE-1)
#endif
#if defined UMC_HWDROP || defined UMC_HWFILTER || defined LRC_HWDROP || defined LRC_HWFILTER || defined DIFT_HWDROP || defined DIFT_HWFILTER || defined DIFT_RF_HWDROP || defined DIFT_RF_HWFILTER
	#define WCET_IS_1 19
	#define WCET_IS_2 126
	#define WCET_IS 745*(WCET_SCALE-1)
	#define WCET_CRC_1 53
	#define WCET_CRC_3 12
	#define WCET_CRC_4 35
	#define WCET_CRC_5 33
	#define WCET_CRC_2 21
	#define WCET_CRC 1566*(WCET_SCALE-1)
	#define WCET_EDN_1 38
	#define WCET_EDN_2 21
	#define WCET_EDN_3 28
	#define WCET_EDN_4 10
	#define WCET_EDN_5 55
	#define WCET_EDN_6 419
	#define WCET_EDN_7 41
	#define WCET_EDN_8 324
	#define WCET_EDN_9 91
	#define WCET_EDN_10 17
	#define WCET_EDN_11 80
	#define WCET_EDN_12 28
	#define WCET_EDN_13 166
	#define WCET_EDN_14 348
	#define WCET_EDN_15 17
	#define WCET_EDN 47013*(WCET_SCALE-1)
	#define WCET_CMP_1 1006
	#define WCET_CMP_2 78
	#define WCET_CMP_3 40
	#define WCET_CMP_4 14
	#define WCET_CMP_5 17
	#define WCET_CMP 5175*(WCET_SCALE-1)
	#define WCET_FIR_1 103
	#define WCET_FIR_2 21
	#define WCET_FIR_3 18
	#define WCET_FIR_4 93
	#define WCET_FIR 2897*(WCET_SCALE-1)
	#define WCET_JFDC_1 55
	#define WCET_JFDC_2 267
	#define WCET_JFDC_3 7
	#define WCET_JFDC_4 265
	#define WCET_JFDC_5 15
	#define WCET_JFDC 3201*(WCET_SCALE-1)
	#define WCET_NSI_1 2
	#define WCET_NSI_2 220
	#define WCET_NSI_3 901
	#define WCET_NSI_4 901
	#define WCET_NSI_5 882
	#define WCET_NSI_6 874
	#define WCET_NSI_7 900
	#define WCET_NSI_8 884
	#define WCET_NSI 9223*(WCET_SCALE-1)
	#define WCET_STA_1 158
	#define WCET_STA_2 163
	#define WCET_STA_3 44
	#define WCET_STA_4 175
	#define WCET_STA_5 964
	#define WCET_STA 2342*(WCET_SCALE-1)
#endif
#ifdef DIFT_FULL
	#define WCET_IS_1 18
	#define WCET_IS_2 7071
	#define WCET_IS 39375*(WCET_SCALE-1)
	#define WCET_CRC_1 55
	#define WCET_CRC_3 448
	#define WCET_CRC_4 882
	#define WCET_CRC_5 809
	#define WCET_CRC_2 996
	#define WCET_CRC 65251*(WCET_SCALE-1)
	#define WCET_EDN_1 109
	#define WCET_EDN_2 439
	#define WCET_EDN_3 1135
	#define WCET_EDN_4 433
	#define WCET_EDN_5 1021
	#define WCET_EDN_6 16096
	#define WCET_EDN_7 2307
	#define WCET_EDN_8 15968
	#define WCET_EDN_9 3382
	#define WCET_EDN_10 988
	#define WCET_EDN_11 3004
	#define WCET_EDN_12 1255
	#define WCET_EDN_13 4600
	#define WCET_EDN_14 15087
	#define WCET_EDN_15 1175
	#define WCET_EDN 2098326*(WCET_SCALE-1)
	#define WCET_CMP_1 25098
	#define WCET_CMP_2 2328
	#define WCET_CMP_3 1006
	#define WCET_CMP_4 503
	#define WCET_CMP_5 1053
	#define WCET_CMP 171641*(WCET_SCALE-1)
	#define WCET_FIR_1 1818
	#define WCET_FIR_2 740
	#define WCET_FIR_3 553
	#define WCET_FIR_4 3363
	#define WCET_FIR 154997*(WCET_SCALE-1)
	#define WCET_JFDC_1 361
	#define WCET_JFDC_2 8930
	#define WCET_JFDC_3 252
	#define WCET_JFDC_4 8928
	#define WCET_JFDC_5 1051
	#define WCET_JFDC 143456*(WCET_SCALE-1)
	#define WCET_NSI_1 5
	#define WCET_NSI_2 3862
	#define WCET_NSI_3 16678
	#define WCET_NSI_4 16723
	#define WCET_NSI_5 16804
	#define WCET_NSI_6 16741
	#define WCET_NSI_7 16813
	#define WCET_NSI_8 16867
	#define WCET_NSI 206683*(WCET_SCALE-1)
	#define WCET_STA_1 1918
	#define WCET_STA_2 3528
	#define WCET_STA_3 1026
	#define WCET_STA_4 3826
	#define WCET_STA_5 24667
	#define WCET_STA 64038*(WCET_SCALE-1)
#endif
#ifdef DIFT_SWDROP
	#define WCET_IS_1 18
	#define WCET_IS_2 4359
	#define WCET_IS 24321*(WCET_SCALE-1)
	#define WCET_CRC_1 46
	#define WCET_CRC_3 266
	#define WCET_CRC_4 520
	#define WCET_CRC_5 526
	#define WCET_CRC_2 608
	#define WCET_CRC 38249*(WCET_SCALE-1)
	#define WCET_EDN_1 92
	#define WCET_EDN_2 257
	#define WCET_EDN_3 723
	#define WCET_EDN_4 279
	#define WCET_EDN_5 624
	#define WCET_EDN_6 9414
	#define WCET_EDN_7 1457
	#define WCET_EDN_8 9762
	#define WCET_EDN_9 2081
	#define WCET_EDN_10 628
	#define WCET_EDN_11 1859
	#define WCET_EDN_12 763
	#define WCET_EDN_13 2786
	#define WCET_EDN_14 9131
	#define WCET_EDN_15 737
	#define WCET_EDN 1259512*(WCET_SCALE-1)
	#define WCET_CMP_1 14884
	#define WCET_CMP_2 1394
	#define WCET_CMP_3 590
	#define WCET_CMP_4 295
	#define WCET_CMP_5 667
	#define WCET_CMP 102171*(WCET_SCALE-1)
	#define WCET_FIR_1 1137
	#define WCET_FIR_2 484
	#define WCET_FIR_3 375
	#define WCET_FIR_4 2052
	#define WCET_FIR 102332*(WCET_SCALE-1)
	#define WCET_JFDC_1 231
	#define WCET_JFDC_2 5238
	#define WCET_JFDC_3 148
	#define WCET_JFDC_4 5236
	#define WCET_JFDC_5 665
	#define WCET_JFDC 83738*(WCET_SCALE-1)
	#define WCET_NSI_1 5
	#define WCET_NSI_2 2257
	#define WCET_NSI_3 10045
	#define WCET_NSI_4 9981
	#define WCET_NSI_5 9981
	#define WCET_NSI_6 9972
	#define WCET_NSI_7 10027
	#define WCET_NSI_8 10064
	#define WCET_NSI 122548*(WCET_SCALE-1)
	#define WCET_STA_1 1229
	#define WCET_STA_2 2117
	#define WCET_STA_3 619
	#define WCET_STA_4 2266
	#define WCET_STA_5 14889
	#define WCET_STA 38547*(WCET_SCALE-1)
#endif
#endif
#ifdef FLEXHW
#if defined UMC_HWDROP || defined UMC_HWFILTER || defined LRC_HWDROP || defined LRC_HWFILTER || defined DIFT_HWDROP || defined DIFT_HWFILTER || defined DIFT_RF_HWDROP || defined DIFT_RF_HWFILTER
	#define WCET_IS_1 9
	#define WCET_IS_2 124
	#define WCET_IS 687*(WCET_SCALE-1)
	#define WCET_CRC_1 44
	#define WCET_CRC_3 10
	#define WCET_CRC_4 29
	#define WCET_CRC_5 16
	#define WCET_CRC_2 25
	#define WCET_CRC 1373*(WCET_SCALE-1)
	#define WCET_EDN_1 32
	#define WCET_EDN_2 13
	#define WCET_EDN_3 24
	#define WCET_EDN_4 8
	#define WCET_EDN_5 32
	#define WCET_EDN_6 316
	#define WCET_EDN_7 54
	#define WCET_EDN_8 272
	#define WCET_EDN_9 73
	#define WCET_EDN_10 16
	#define WCET_EDN_11 69
	#define WCET_EDN_12 21
	#define WCET_EDN_13 121
	#define WCET_EDN_14 251
	#define WCET_EDN_15 25
	#define WCET_EDN 37176*(WCET_SCALE-1)
	#define WCET_CMP_1 664
	#define WCET_CMP_2 55
	#define WCET_CMP_3 29
	#define WCET_CMP_4 11
	#define WCET_CMP_5 18
	#define WCET_CMP 3524*(WCET_SCALE-1)
	#define WCET_FIR_1 99
	#define WCET_FIR_2 19
	#define WCET_FIR_3 16
	#define WCET_FIR_4 80
	#define WCET_FIR 2813*(WCET_SCALE-1)
	#define WCET_JFDC_1 43
	#define WCET_JFDC_2 207
	#define WCET_JFDC_3 5
	#define WCET_JFDC_4 207
	#define WCET_JFDC_5 16
	#define WCET_JFDC 2497*(WCET_SCALE-1)
	#define WCET_NSI_1 2
	#define WCET_NSI_2 170
	#define WCET_NSI_3 656
	#define WCET_NSI_4 662
	#define WCET_NSI_5 663
	#define WCET_NSI_6 662
	#define WCET_NSI_7 663
	#define WCET_NSI_8 665
	#define WCET_NSI 6777*(WCET_SCALE-1)
	#define WCET_STA_1 115
	#define WCET_STA_2 119
	#define WCET_STA_3 28
	#define WCET_STA_4 109
	#define WCET_STA_5 626
	#define WCET_STA 1484*(WCET_SCALE-1)
#endif
#endif

#endif // __MALARDEN_H__

