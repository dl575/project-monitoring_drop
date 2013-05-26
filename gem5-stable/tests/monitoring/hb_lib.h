/*
 * HardBound monitoring library.
 * 
 *  Custom wrappers for standard library functions such as malloc/free.
 *
 * Author: Tao Chen
 */

/* Adapted from http://www.gnu.org/software/libc/manual/html_node/Hooks-for-Malloc.html */

#ifdef HB_ENABLED

#ifndef __HB_LIB_H__
#define __HB_LIB_H__

/* Prototypes for __malloc_hook, __free_hook */
#include <malloc.h>

/* Prototypes for our hooks.  */
static void hb_init_hook (void);
static void *hb_malloc_hook (size_t, const void *);
static void *hb_realloc_hook (void *, size_t, const void *);
static void hb_free_hook (void*, const void *);

#endif // __HB_LIB_H__
#endif // HB_ENABLED