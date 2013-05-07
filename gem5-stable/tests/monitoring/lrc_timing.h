/*
 * Include file for information about monitor timing
 *
 * Author: Mohamed Ismail
 */

#ifndef __LRC_TIME_H__
#define __LRC_TIME_H__

#ifdef LRC_FULL
    #define MON_DROP_WCET 0
#endif

#ifdef LRC_SWDROP
#ifdef ATOMIC
    #define MON_WCET 19
    #define MON_DROP_WCET 12
#endif
#ifdef TIMING
    #define MON_WCET 21
    #define MON_DROP_WCET 13
#endif
#endif

#ifdef LRC_HWDROP
#ifdef ATOMIC
    #define MON_WCET 22
    #define MON_DROP_WCET 1
#endif
#ifdef TIMING
    #define MON_WCET 23
    #define MON_DROP_WCET 1
#endif
#endif

#ifdef LRC_HWFILTER
#ifdef ATOMIC
    #define MON_WCET 20
    #define MON_DROP_WCET 1
#endif
#ifdef TIMING
    #define MON_WCET 21
    #define MON_DROP_WCET 1
#endif
#endif


#endif // __LRC_TIME_H__
