/*
 * Include file for interface for accessing flag cache.
 *
 * Author: Mohamed Ismail
 */

#ifndef __FLAGCACHE_H__
#define __FLAGCACHE_H__

// Address of monitoring fifo (for read)
#define FC_ADDR 0x30020000

// Initialization
volatile unsigned int *flagcache;
#define INIT_FC flagcache = (int *)FC_ADDR;

// read values
#define FC_GET_ADDR            *(flagcache) 
#define FC_ARRAY_GET           *(flagcache + 1) 
#define FC_CACHE_GET           *(flagcache + 2) 

// write registers
#define FC_SET_ADDR(addr)      *(flagcache) = addr;
#define FC_ARRAY_SET           *(flagcache + 1) = 0;
#define FC_CACHE_SET           *(flagcache + 1) = 1;
#define FC_ARRAY_CLEAR         *(flagcache + 2) = 0;
#define FC_CACHE_CLEAR         *(flagcache + 2) = 1;

#endif // __FLAGCACHE_H__
