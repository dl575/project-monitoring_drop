/*
 * BC monitoring library.
 * 
 *  Custom wrappers for standard library functions such as malloc/free.
 *
 * Author: Tao Chen
 */

/* Adapted from http://www.gnu.org/software/libc/manual/html_node/Hooks-for-Malloc.html */

#ifdef BC_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include "bc_lib.h"
#include "monitoring.h"

/* disable warnings about deprecated definitions*/
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

/* Override initializing hook from the C library. */
void (* volatile __malloc_initialize_hook) (void) = bc_init_hook;

static void *(*old_malloc_hook)(size_t size, const void *caller);
static void *(*old_realloc_hook)(void *ptr, size_t size, const void *caller);
static void (*old_free_hook)(void *ptr, const void *caller);

static void
bc_init_hook (void)
{
  old_malloc_hook = __malloc_hook;
  old_free_hook = __free_hook;
  old_realloc_hook = __realloc_hook;
  __malloc_hook = bc_malloc_hook;
  __free_hook = bc_free_hook;
  __realloc_hook = bc_realloc_hook;
}

static void *
bc_malloc_hook (size_t size, const void *caller)
{
  void *result;
  unsigned int tag;
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
    /* generate a tag in the range of 1-15 */
    tag = rand() % 15 + 1;
    /* set memory tags */
    set_tag((unsigned)result, (unsigned)size, tag);
    /* set pointer tag */
    set_tag((unsigned)&result, sizeof(void*), tag << 4);
  }
  /* Restore our own hooks */
  __malloc_hook = bc_malloc_hook;
  __free_hook = bc_free_hook;
  __realloc_hook = bc_realloc_hook;
  return result;
}

static void *
bc_realloc_hook(void *ptr, size_t size, const void *caller)
{
  void *result;
  unsigned int tag;
  if (!ptr) {
    return malloc(size);
  } else {
    if (size == 0) {
      free(ptr);
      return NULL;
    } else {
      /* should we clear old tags here? */
      /* clear pointer tag */
      set_tag((unsigned)&ptr, sizeof(void*), 0);
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
        /* generate a tag in the range of 1-15 */
        tag = rand() % 15 + 1;
        /* set memory tags */
        set_tag((unsigned)result, (unsigned)size, tag);
        /* set pointer tag */
        set_tag((unsigned)&result, sizeof(void*), tag << 4);
      }
      /* Restore our own hooks */
      __malloc_hook = bc_malloc_hook;
      __free_hook = bc_free_hook;
      __realloc_hook = bc_realloc_hook;
      return result;
    }
  }
}

static void
bc_free_hook (void *ptr, const void *caller)
{
  /* Clear tags */
  if (ptr) {
    /* clear pointer tag */
    set_tag((unsigned)&ptr, sizeof(void*), 0);
    size_t size = malloc_usable_size(ptr);
    set_tag((unsigned)ptr, (unsigned)size, 0);
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
  __malloc_hook = bc_malloc_hook;
  __free_hook = bc_free_hook;
  __realloc_hook = bc_realloc_hook;
}

#endif // BC_ENABLED
