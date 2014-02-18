/*
 * Include file for information about monitor timing
 * This is AUTO GENERATED.
 * Author: Mohamed Ismail
 */
 
#ifndef __MONITOR_TIME_H__
#define __MONITOR_TIME_H__

#ifdef ATOMIC
#ifdef UMC_FULL
	#define MON_WCET 19
	#define MON_DROP_WCET 19
#endif
#ifdef UMC_SWDROP
	#define MON_WCET 25
	#define MON_DROP_WCET 5
#endif
#ifdef UMC_HWDROP
	#define MON_WCET 27
	#define MON_DROP_WCET 1
#endif
#ifdef UMC_HWFILTER
	#define MON_WCET 24
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_FULL
	#define MON_WCET 17
	#define MON_DROP_WCET 17
#endif
#ifdef LRC_SWDROP
	#define MON_WCET 22
	#define MON_DROP_WCET 22
#endif
#ifdef LRC_HWDROP
	#define MON_WCET 21
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_HWFILTER
	#define MON_WCET 17
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_FULL
	#define MON_WCET 26
	#define MON_DROP_WCET 26
#endif
#ifdef DIFT_SWDROP
	#define MON_WCET 28
	#define MON_DROP_WCET 16
#endif
#ifdef DIFT_HWDROP
	#define MON_WCET 41
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_HWFILTER
	#define MON_WCET 30
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_RF_HWDROP
	#define MON_WCET 39
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_RF_HWFILTER
	#define MON_WCET 31
	#define MON_DROP_WCET 1
#endif
#endif
#ifdef TIMING
#ifdef UMC_FULL
	#define MON_WCET 30
	#define MON_DROP_WCET 30
#endif
#ifdef UMC_SWDROP
	#define MON_WCET 44
	#define MON_DROP_WCET 15
#endif
#ifdef UMC_HWDROP
	#define MON_WCET 55
	#define MON_DROP_WCET 1
#endif
#ifdef UMC_HWFILTER
	#define MON_WCET 43
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_FULL
	#define MON_WCET 18
	#define MON_DROP_WCET 18
#endif
#ifdef LRC_SWDROP
	#define MON_WCET 33
	#define MON_DROP_WCET 33
#endif
#ifdef LRC_HWDROP
	#define MON_WCET 22
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_HWFILTER
	#define MON_WCET 18
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_FULL
	#define MON_WCET 61
	#define MON_DROP_WCET 61
#endif
#ifdef DIFT_SWDROP
	#define MON_WCET 63
	#define MON_DROP_WCET 35
#endif
#ifdef DIFT_HWDROP
	#define MON_WCET 71
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_HWFILTER
	#define MON_WCET 67
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_RF_HWDROP
	#define MON_WCET 58
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_RF_HWFILTER
	#define MON_WCET 50
	#define MON_DROP_WCET 1
#endif
#endif
#ifdef FLEXHW
#ifdef UMC_HWDROP
	#define MON_WCET 8
	#define MON_DROP_WCET 1
#endif
#ifdef UMC_HWFILTER
	#define MON_WCET 8
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_HWDROP
	#define MON_WCET 8
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_HWFILTER
	#define MON_WCET 8
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_HWDROP
	#define MON_WCET 8
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_HWFILTER
	#define MON_WCET 8
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_RF_HWDROP
	#define MON_WCET 8
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_RF_HWFILTER
	#define MON_WCET 8
	#define MON_DROP_WCET 1
#endif
#endif

#endif // __MONITOR_TIME_H__
