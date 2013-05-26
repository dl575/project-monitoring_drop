/*
 * BC monitoring library.
 * 
 *  Custom wrappers for standard library functions such as malloc/free.
 *
 * Author: Tao Chen
 */

/* Adapted from http://www.gnu.org/software/libc/manual/html_node/Hooks-for-Malloc.html */

#ifdef BC_ENABLED

#ifndef __BC_LIB_H__
#define __BC_LIB_H__

/* Prototypes for __malloc_hook, __free_hook */
#include <malloc.h>

/* Prototypes for our hooks.  */
static void bc_init_hook (void);
static void *bc_malloc_hook (size_t, const void *);
static void *bc_realloc_hook (void *, size_t, const void *);
static void bc_free_hook (void*, const void *);

#endif // __BC_LIB_H__
#endif // BC_ENABLED