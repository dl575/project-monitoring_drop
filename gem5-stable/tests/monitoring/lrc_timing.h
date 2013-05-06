/*
 * Include file for information about monitor timing
 *
 * Author: Mohamed Ismail
 */

#ifndef __LRC_TIME_H__
#define __LRC_TIME_H__

#ifdef LRC_SWDROP
#ifdef ATOMIC
    #define MON_WCET 24
    #define MON_DROP_WCET 16
#endif
#ifdef TIMING
    #define MON_WCET 40
    #define MON_DROP_WCET 26
#endif
#endif

#ifdef LRC_HWDROP
#ifdef ATOMIC

#endif
#ifdef TIMING
    #define MON_WCET 56
    #define MON_DROP_WCET 2
#endif
#endif

#ifdef LRC_HWFILTER
#ifdef ATOMIC

#endif
#ifdef TIMING
    #define MON_WCET 52
    #define MON_DROP_WCET 2
#endif
#endif


#endif // __LRC_TIME_H__
