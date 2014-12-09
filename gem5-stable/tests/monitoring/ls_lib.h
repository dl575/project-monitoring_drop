/*
 * LockSet monitoring library.
 * 
 *  Custom wrappers for standard library functions such as malloc/free.
 *
 * Author: Tao Chen
 */

/* Adapted from http://www.gnu.org/software/libc/manual/html_node/Hooks-for-Malloc.html */

#ifdef LS_ENABLED

#ifndef __LS_LIB_H__
#define __LS_LIB_H__

/* Prototypes for __malloc_hook, __free_hook */
#include <malloc.h>

/* Prototypes for our hooks.  */
static void ls_init_hook (void);
static void *ls_malloc_hook (size_t, const void *);
static void *ls_realloc_hook (void *, size_t, const void *);
static void ls_free_hook (void*, const void *);

#endif // __LS_LIB_H__
#endif // LS_ENABLED