/*
 * LockSet monitoring library.
 * 
 *  Custom wrappers for standard library functions such as malloc/free.
 *
 * Author: Tao Chen
 */

/* Adapted from http://www.gnu.org/software/libc/manual/html_node/Hooks-for-Malloc.html */

#ifdef LS_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include "ls_lib.h"
#include "monitoring.h"

/* state definitions for lockset */
#define STATE_VIRGIN    0
#define STATE_EXCLUSIVE 0
#define STATE_SHARED_RO 0
#define STATE_SHARED_RW 0

/* disable warnings about deprecated definitions*/
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/* Override initializing hook from the C library. */
void (* volatile __malloc_initialize_hook) (void) = ls_init_hook;

static void *(*old_malloc_hook)(size_t size, const void *caller);
static void *(*old_realloc_hook)(void *ptr, size_t size, const void *caller);
static void (*old_free_hook)(void *ptr, const void *caller);

static void
ls_init_hook (void)
{
  old_malloc_hook = __malloc_hook;
  old_free_hook = __free_hook;
  old_realloc_hook = __realloc_hook;
  __malloc_hook = ls_malloc_hook;
  __free_hook = ls_free_hook;
  __realloc_hook = ls_realloc_hook;
}

static void *
ls_malloc_hook (size_t size, const void *caller)
{
  void *result;
  /* Restore all old hooks */
  __malloc_hook = old_malloc_hook;
  __free_hook = old_free_hook;
  __realloc_hook = old_realloc_hook;
  /* Call recursively */
  result = malloc (size);
  /* Save underlying hooks */
  old_malloc_hook = __malloc_hook;
  old_free_hook = __free_hook;
  old_realloc_hook = __realloc_hook;
  if (result) {
    /* set tags to STATE_VIRGIN */
    set_tag((unsigned)result, size, STATE_VIRGIN);
  }
  /* Restore our own hooks */
  __malloc_hook = ls_malloc_hook;
  __free_hook = ls_free_hook;
  __realloc_hook = ls_realloc_hook;
  return result;
}

static void *
ls_realloc_hook(void *ptr, size_t size, const void *caller)
{
  void *result;
  if (!ptr) {
    return malloc(size);
  } else {
    if (size == 0) {
      free(ptr);
      return NULL;
    } else {
      /* we should not clear old tags */
      /* Restore all old hooks */
      __malloc_hook = old_malloc_hook;
      __free_hook = old_free_hook;
      __realloc_hook = old_realloc_hook;
      /* Call recursively */
      result = realloc (ptr, size);
      /* Save underlying hooks */
      old_malloc_hook = __malloc_hook;
      old_free_hook = __free_hook;
      old_realloc_hook = __realloc_hook;
      if (result) {
        /* set tags to STATE_VIRGIN */
        set_tag((unsigned)result, size, STATE_VIRGIN);
      }
      /* Restore our own hooks */
      __malloc_hook = ls_malloc_hook;
      __free_hook = ls_free_hook;
      __realloc_hook = ls_realloc_hook;
      return result;
    }
  }
}

static void
ls_free_hook (void *ptr, const void *caller)
{
  /* no need to clear tags, tags will be set to STATE_VIRGIN
     during next allocation */
  /* Restore all old hooks */
  __malloc_hook = old_malloc_hook;
  __free_hook = old_free_hook;
  __realloc_hook = old_realloc_hook;
  /* Call recursively */
  free (ptr);
  /* Save underlying hooks */
  old_malloc_hook = __malloc_hook;
  old_free_hook = __free_hook;
  old_realloc_hook = __realloc_hook;
  /* printf might call free, so protect it too. */
  // printf ("freed pointer %p\n", ptr);
  /* Restore our own hooks */
  __malloc_hook = ls_malloc_hook;
  __free_hook = ls_free_hook;
  __realloc_hook = ls_realloc_hook;
}

#endif // LS_ENABLED
