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
	#define MON_WCET 32
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
	#define MON_DROP_WCET 19
#endif
#ifdef LRC_HWDROP
	#define MON_WCET 22
	#define MON_DROP_WCET 1
#endif
#ifdef LRC_HWFILTER
	#define MON_WCET 20
	#define MON_DROP_WCET 20
#endif
#endif
#ifdef TIMING
#ifdef UMC_FULL
	#define MON_WCET 325
	#define MON_DROP_WCET 322
#endif
#ifdef UMC_SWDROP
	#define MON_WCET 389
	#define MON_DROP_WCET 199
#endif
#ifdef UMC_HWDROP
	#define MON_WCET 390
	#define MON_DROP_WCET 8
#endif
#ifdef UMC_HWFILTER
	#define MON_WCET 372
	#define MON_DROP_WCET 8
#endif
#ifdef LRC_FULL
	#define MON_WCET 195
	#define MON_DROP_WCET 128
#endif
#ifdef LRC_SWDROP
	#define MON_WCET 168
	#define MON_DROP_WCET 104
#endif
#ifdef LRC_HWDROP
	#define MON_WCET 184
	#define MON_DROP_WCET 8
#endif
#ifdef LRC_HWFILTER
	#define MON_WCET 168
	#define MON_DROP_WCET 8
#endif
#endif

#endif // __MONITOR_TIME_H__
