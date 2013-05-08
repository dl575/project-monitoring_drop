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
#endif
#ifdef TIMING
#ifdef UMC_FULL
	#define MON_WCET 65
	#define MON_DROP_WCET 65
#endif
#ifdef UMC_SWDROP
	#define MON_WCET 81
	#define MON_DROP_WCET 33
#endif
#ifdef UMC_HWDROP
	#define MON_WCET 77
	#define MON_DROP_WCET 2
#endif
#ifdef UMC_HWFILTER
	#define MON_WCET 75
	#define MON_DROP_WCET 2
#endif
#ifdef LRC_FULL
	#define MON_WCET 33
	#define MON_DROP_WCET 32
#endif
#ifdef LRC_SWDROP
	#define MON_WCET 42
	#define MON_DROP_WCET 26
#endif
#ifdef LRC_HWDROP
	#define MON_WCET 46
	#define MON_DROP_WCET 2
#endif
#ifdef LRC_HWFILTER
	#define MON_WCET 42
	#define MON_DROP_WCET 2
#endif
#endif

#endif // __MONITOR_TIME_H__
