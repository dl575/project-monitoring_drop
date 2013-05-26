/*
 * BC monitoring library.
 * 
 *  Custom wrappers for standard library functions such as malloc/free.
 *
 * Author: Tao Chen
 */

/* Adapted from http://www.gnu.org/software/libc/manual/html_node/Hooks-for-Malloc.html */

#ifdef HB_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include "hb_lib.h"
#include "monitoring.h"

/* disable warnings about deprecated definitions*/
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/* Override initializing hook from the C library. */
void (* volatile __malloc_initialize_hook) (void) = hb_init_hook;

static void *(*old_malloc_hook)(size_t size, const void *caller);
static void *(*old_realloc_hook)(void *ptr, size_t size, const void *caller);
static void (*old_free_hook)(void *ptr, const void *caller);

static void
hb_init_hook (void)
{
  old_malloc_hook = __malloc_hook;
  old_free_hook = __free_hook;
  old_realloc_hook = __realloc_hook;
  __malloc_hook = hb_malloc_hook;
  __free_hook = hb_free_hook;
  __realloc_hook = hb_realloc_hook;
}

static void *
hb_malloc_hook (size_t size, const void *caller)
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
  /* printf might call malloc, so protect it too. */
  // printf ("malloc (%u) returns %p\n", (unsigned int) size, result);
  if (result) {
    set_tag_base((unsigned)&result, (unsigned)result);
    set_tag_bound((unsigned)&result, (unsigned)result + (unsigned)size);
  }
  /* Restore our own hooks */
  __malloc_hook = hb_malloc_hook;
  __free_hook = hb_free_hook;
  __realloc_hook = hb_realloc_hook;
  return result;
}

static void *
hb_realloc_hook(void *ptr, size_t size, const void *caller)
{
  void *result;
  if (!ptr) {
    return malloc(size);
  } else {
    if (size == 0) {
      free(ptr);
      return NULL;
    } else {
      /* clear tag */
      set_tag_base((unsigned)&ptr, 0);
      set_tag_bound((unsigned)&ptr, 0);
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
      /* printf might call malloc, so protect it too. */
      // printf ("realloc (%u) returns %p\n", (unsigned int) size, result);
      if (result) {
        set_tag_base((unsigned)&result, (unsigned)result);
        set_tag_bound((unsigned)&result, (unsigned)result + (unsigned)size);
      }
      /* Restore our own hooks */
      __malloc_hook = hb_malloc_hook;
      __free_hook = hb_free_hook;
      __realloc_hook = hb_realloc_hook;
      return result;
    }
  }
}

static void
hb_free_hook (void *ptr, const void *caller)
{
  /* Clear tags */
  if (ptr) {
    /* clear tag */
    set_tag_base((unsigned)&ptr, 0);
    set_tag_bound((unsigned)&ptr, 0);
  }
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
  __malloc_hook = hb_malloc_hook;
  __free_hook = hb_free_hook;
  __realloc_hook = hb_realloc_hook;
}

#endif // HB_ENABLED
