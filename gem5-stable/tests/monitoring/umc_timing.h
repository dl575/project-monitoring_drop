/*
 * Include file for information about monitor timing
 *
 * Author: Mohamed Ismail
 */

#ifndef __UMC_TIME_H__
#define __UMC_TIME_H__

#ifdef UMC_FULL
    #define MON_DROP_WCET 0
#endif

#ifdef UMC_SWDROP
#ifdef ATOMIC
    #define MON_WCET 30
    #define MON_DROP_WCET 6
#endif
#ifdef TIMING
    #define MON_WCET 40
    #define MON_DROP_WCET 16
#endif
#endif

#ifdef UMC_HWDROP
#ifdef ATOMIC
    #define MON_WCET 29
    #define MON_DROP_WCET 1
#endif
#ifdef TIMING
    #define MON_WCET 77
    #define MON_DROP_WCET 2
#endif
#endif

#ifdef UMC_HWFILTER
#ifdef ATOMIC
    #define MON_WCET 26
    #define MON_DROP_WCET 1
#endif
#ifdef TIMING
    #define MON_WCET 37
    #define MON_DROP_WCET 1
#endif
#endif

#endif // __UMC_TIME_H__
