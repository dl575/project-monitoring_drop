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
	#define MON_WCET 6
	#define MON_DROP_WCET 6
#endif
#ifdef LRC_SWDROP
	#define MON_WCET 9
	#define MON_DROP_WCET 9
#endif
#ifdef LRC_HWDROP
	#define MON_WCET 13
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_HWFILTER
	#define MON_WCET 13
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
	#define MON_WCET 42
	#define MON_DROP_WCET 3
#endif
#ifdef DIFT_HWFILTER
	#define MON_WCET 30
	#define MON_DROP_WCET 3
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
	#define MON_WCET 16
	#define MON_DROP_WCET 16
#endif
#ifdef LRC_SWDROP
	#define MON_WCET 10
	#define MON_DROP_WCET 10
#endif
#ifdef LRC_HWDROP
	#define MON_WCET 14
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_HWFILTER
	#define MON_WCET 14
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_FULL
	#define MON_WCET 52
	#define MON_DROP_WCET 52
#endif
#ifdef DIFT_SWDROP
	#define MON_WCET 63
	#define MON_DROP_WCET 35
#endif
#ifdef DIFT_HWDROP
	#define MON_WCET 72
	#define MON_DROP_WCET 3
#endif
#ifdef DIFT_HWFILTER
	#define MON_WCET 58
	#define MON_DROP_WCET 3
#endif
#endif
#ifdef FLEX
#ifdef UMC_FULL
	#define MON_WCET 91
	#define MON_DROP_WCET 91
#endif
#ifdef UMC_SWDROP
	#define MON_WCET 118
	#define MON_DROP_WCET 98
#endif
#ifdef UMC_HWDROP
	#define MON_WCET 128
	#define MON_DROP_WCET 1
#endif
#ifdef UMC_HWFILTER
	#define MON_WCET 114
	#define MON_DROP_WCET 2
#endif
#ifdef LRC_FULL
	#define MON_WCET 29
	#define MON_DROP_WCET 29
#endif
#ifdef LRC_SWDROP
	#define MON_WCET 9
	#define MON_DROP_WCET 9
#endif
#ifdef LRC_HWDROP
	#define MON_WCET 14
	#define MON_DROP_WCET 2
#endif
#ifdef LRC_HWFILTER
	#define MON_WCET 14
	#define MON_DROP_WCET 2
#endif
#ifdef DIFT_FULL
	#define MON_WCET 124
	#define MON_DROP_WCET 124
#endif
#ifdef DIFT_SWDROP
	#define MON_WCET 133
	#define MON_DROP_WCET 121
#endif
#ifdef DIFT_HWDROP
	#define MON_WCET 347
	#define MON_DROP_WCET 3
#endif
#ifdef DIFT_HWFILTER
	#define MON_WCET 142
	#define MON_DROP_WCET 3
#endif
#endif

#endif // __MONITOR_TIME_H__
