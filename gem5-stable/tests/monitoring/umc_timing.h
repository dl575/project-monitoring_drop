/*
 * Include file for information about monitor timing
 *
 * Author: Mohamed Ismail
 */

#ifndef __UMC_TIME_H__
#define __UMC_TIME_H__

#ifdef UMC_SWDROP
#ifdef ATOMIC

#endif
#ifdef TIMING
    #define MON_WCET 128
    #define MON_DROP_WCET 80
#endif
#endif

#ifdef UMC_HWDROP
#ifdef ATOMIC

#endif
#ifdef TIMING
    #define MON_WCET 124
    #define MON_DROP_WCET 2
#endif
#endif

#ifdef UMC_HWFILTER
#ifdef ATOMIC
    #define MON_WCET 26
    #define MON_DROP_WCET 1
#endif
#ifdef TIMING
    #define MON_WCET 122
    #define MON_DROP_WCET 2
#endif
#endif

#endif // __UMC_TIME_H__
