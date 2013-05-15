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
	#define MON_WCET 242
	#define MON_DROP_WCET 239
#endif
#ifdef UMC_SWDROP
	#define MON_WCET 384
	#define MON_DROP_WCET 196
#endif
#ifdef UMC_HWDROP
	#define MON_WCET 404
	#define MON_DROP_WCET 8
#endif
#ifdef UMC_HWFILTER
	#define MON_WCET 352
	#define MON_DROP_WCET 7
#endif
#ifdef LRC_FULL
	#define MON_WCET 96
	#define MON_DROP_WCET 64
#endif
#ifdef LRC_SWDROP
	#define MON_WCET 105
	#define MON_DROP_WCET 65
#endif
#ifdef LRC_HWDROP
	#define MON_WCET 138
	#define MON_DROP_WCET 6
#endif
#ifdef LRC_HWFILTER
	#define MON_WCET 105
	#define MON_DROP_WCET 5
#endif
#ifdef DIFT_FULL
	#define MON_WCET 405
	#define MON_DROP_WCET 387
#endif
#ifdef DIFT_SWDROP
	#define MON_WCET 591
	#define MON_DROP_WCET 314
#endif
#ifdef DIFT_HWDROP
	#define MON_WCET 808
	#define MON_DROP_WCET 12
#endif
#ifdef DIFT_HWFILTER
	#define MON_WCET 717
	#define MON_DROP_WCET 12
#endif
#endif

#endif // __MONITOR_TIME_H__
