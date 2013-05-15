/*
 * Include file for information about monitor timing
 * This is AUTO GENERATED.
 * Author: Mohamed Ismail
 */
 
#ifndef __MONITOR_TIME_H__
#define __MONITOR_TIME_H__

#ifdef ATOMIC
#ifdef UMC_FULL
	#define MON_WCET 22
	#define MON_DROP_WCET 22
#endif
#ifdef UMC_SWDROP
	#define MON_WCET 30
	#define MON_DROP_WCET 6
#endif
#ifdef UMC_HWDROP
	#define MON_WCET 29
	#define MON_DROP_WCET 1
#endif
#ifdef UMC_HWFILTER
	#define MON_WCET 26
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_FULL
	#define MON_WCET 15
	#define MON_DROP_WCET 15
#endif
#ifdef LRC_SWDROP
	#define MON_WCET 19
	#define MON_DROP_WCET 12
#endif
#ifdef LRC_HWDROP
	#define MON_WCET 22
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_HWFILTER
	#define MON_WCET 20
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_FULL
	#define MON_WCET 30
	#define MON_DROP_WCET 30
#endif
#ifdef DIFT_SWDROP
	#define MON_WCET 34
	#define MON_DROP_WCET 20
#endif
#ifdef DIFT_HWDROP
	#define MON_WCET 46
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_HWFILTER
	#define MON_WCET 46
	#define MON_DROP_WCET 2
#endif
#endif
#ifdef TIMING
#ifdef UMC_FULL
	#define MON_WCET 32
	#define MON_DROP_WCET 32
#endif
#ifdef UMC_SWDROP
	#define MON_WCET 40
	#define MON_DROP_WCET 16
#endif
#ifdef UMC_HWDROP
	#define MON_WCET 40
	#define MON_DROP_WCET 1
#endif
#ifdef UMC_HWFILTER
	#define MON_WCET 37
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_FULL
	#define MON_WCET 16
	#define MON_DROP_WCET 16
#endif
#ifdef LRC_SWDROP
	#define MON_WCET 21
	#define MON_DROP_WCET 13
#endif
#ifdef LRC_HWDROP
	#define MON_WCET 23
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_HWFILTER
	#define MON_WCET 21
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_FULL
	#define MON_WCET 42
	#define MON_DROP_WCET 40
#endif
#ifdef DIFT_SWDROP
	#define MON_WCET 54
	#define MON_DROP_WCET 26
#endif
#ifdef DIFT_HWDROP
	#define MON_WCET 55
	#define MON_DROP_WCET 1
#endif
#ifdef DIFT_HWFILTER
	#define MON_WCET 55
	#define MON_DROP_WCET 2
#endif
#endif

#endif // __MONITOR_TIME_H__
