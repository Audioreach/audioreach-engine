#ifndef _SPF_MACROS_H_
#define _SPF_MACROS_H_

/**
 * \file spf_macros.h
 * \brief
 *    This file contains macros useful for containers.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*------------------------------------------------------------------------------
 *  Header Includes
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** Macros */
#ifndef MIN
#define MIN(m, n) (((m) < (n)) ? (m) : (n))
#endif

#ifndef MAX
#define MAX(m, n) (((m) > (n)) ? (m) : (n))
#endif

#define INIT_EXCEPTION_HANDLING uint32_t exception_line_number = 0;

/** try to call a function and if it fails go to exception handling */
#define TRY(exception, func)                                                                                           \
   if (AR_EOK != (exception = func))                                                                                   \
   {                                                                                                                   \
      exception_line_number = __LINE__;                                                                                \
      goto exception##bail;                                                                                            \
   }

/** throw an error and it will be caught by exception handling */
#define THROW(exception, errcode)                                                                                      \
   exception_line_number = __LINE__;                                                                                   \
   exception             = errcode;                                                                                    \
   goto exception##bail;

/** Verify that certain expression is TRUE. if not, exception handling is called.*/
#define VERIFY(exception, val)                                                                                         \
   do                                                                                                                  \
   {                                                                                                                   \
      if (0 == (val))                                                                                                  \
      {                                                                                                                \
         exception_line_number = __LINE__;                                                                             \
         exception             = exception == AR_EOK ? AR_EFAILED : exception;                                         \
         goto exception##bail;                                                                                         \
      }                                                                                                                \
   } while (0)

/** catching exceptions */
#ifdef SIM

#define CATCH(exception, msg, ...)                                                                                     \
   exception##bail : if (exception != AR_EOK)                                                                          \
   {                                                                                                                   \
      AR_MSG(DBG_ERROR_PRIO,                                                                                           \
             msg ": Exception 0x%lx in %s of %s at line %lu ",                                                         \
             ##__VA_ARGS__,                                                                                            \
             exception,                                                                                                \
             __func__,                                                                                                 \
             __FILE__,                                                                                             \
             exception_line_number);                                                                                   \
   }                                                                                                                   \
   if (exception != AR_EOK)

#else
/** catching exceptions */
#define CATCH(exception, msg, ...)                                                                                     \
   exception##bail : if (exception != AR_EOK)                                                                          \
   {                                                                                                                   \
      AR_MSG(DBG_ERROR_PRIO,                                                                                           \
             msg ":Exception 0x%lx at line number %lu",                                                                \
             ##__VA_ARGS__,                                                                                            \
             exception,                                                                                                \
             exception_line_number);                                                                                   \
   }                                                                                                                   \
   if (exception != AR_EOK)
#endif

/** Some VERIFY are required only for development time, not for production & are enabled for SIM */
#ifdef SIM
#define DBG_INIT_EXCEPTION_HANDLING INIT_EXCEPTION_HANDLING
#define DBG_VERIFY(exception, val)     VERIFY(exception, val)
#define DBG_CATCH(exception, msg, ...) CATCH(exception, msg, ##__VA_ARGS__)
#else
#define DBG_INIT_EXCEPTION_HANDLING
#define DBG_VERIFY(exception, val)
#define DBG_CATCH(exception, msg, ...)
#endif

/** Malloc and memset */
#define MALLOC_MEMSET(ptr, type, size, heap_id, result)                                                                \
   do                                                                                                                  \
   {                                                                                                                   \
      ptr = (type *)posal_memory_malloc(size, heap_id);                                                                \
      if (NULL == ptr)                                                                                                 \
      {                                                                                                                \
         AR_MSG(DBG_ERROR_PRIO, "Malloc failed. Required Size %lu", size);                                             \
         THROW(result, AR_ENOMEMORY);                                                                                  \
      }                                                                                                                \
      memset(ptr, 0, size);                                                                                            \
   } while (0)

/** check, free and assign ptr as NULL */
#define MFREE_NULLIFY(ptr)                                                                                             \
   do                                                                                                                  \
   {                                                                                                                   \
      if (ptr)                                                                                                         \
      {                                                                                                                \
         posal_memory_free(ptr);                                                                                       \
         ptr = NULL;                                                                                                   \
      }                                                                                                                \
   } while (0)

#define MFREE_REALLOC_MEMSET(ptr, type, size, heap_id, result)                                                         \
   do                                                                                                                  \
   {                                                                                                                   \
      if (ptr)                                                                                                         \
      {                                                                                                                \
         posal_memory_free(ptr);                                                                                       \
      }                                                                                                                \
      ptr = (type *)posal_memory_malloc(size, heap_id);                                                                \
      if (NULL == ptr)                                                                                                 \
      {                                                                                                                \
         AR_MSG(DBG_ERROR_PRIO, "Malloc failed. Required Size %lu", size);                                             \
         THROW(result, AR_ENOMEMORY);                                                                                  \
      }                                                                                                                \
      else                                                                                                             \
      {                                                                                                                \
         memset(ptr, 0, size);                                                                                         \
      }                                                                                                                \
   } while (0)

#define SIZE_OF_ARRAY(a) (sizeof(a) / sizeof((a)[0]))

/** Round a number to the nearest multiple of 4 towards the direction of infinity. */
#define ROUNDUP_MULTIPLE4(x) ((((x) + 3) >> 2) << 2)

#define RECURSION_ERROR_CHECK_ON_FN_ENTRY(log_id, counter_ptr, limit)                                                  \
   do                                                                                                                  \
   {                                                                                                                   \
      if (*counter_ptr < limit)                                                                                        \
      {                                                                                                                \
         if (0)                                                                                                        \
            AR_MSG(DBG_LOW_PRIO, "GK  :%08X: recursion depth %lu ", log_id, *counter_ptr);                             \
         (*counter_ptr)++;                                                                                             \
      }                                                                                                                \
      else                                                                                                             \
      {                                                                                                                \
         AR_MSG(DBG_ERROR_PRIO, "GK  :%08X: recursion max depth %lu reached", log_id, limit);                          \
         return AR_EFAILED;                                                                                            \
      }                                                                                                                \
   } while (0)

#define RECURSION_ERROR_CHECK_ON_FN_EXIT(log_id, counter_ptr)                                                          \
   do                                                                                                                  \
   {                                                                                                                   \
      if (*counter_ptr > 0)                                                                                            \
      {                                                                                                                \
         (*counter_ptr)--;                                                                                             \
      }                                                                                                                \
      else                                                                                                             \
      {                                                                                                                \
         if (0)                                                                                                        \
            AR_MSG(DBG_ERROR_PRIO, "GK  :%08X: recursion depth reached zero", log_id, *counter_ptr);                   \
      }                                                                                                                \
   } while (0)

#define ZERO_IF_NULL(ptr) ((NULL == ptr) ? 0 : *ptr)

#define SET_IF_NOT_NULL(ptr, val)                                                                                      \
   if (NULL != ptr)                                                                                                    \
   {                                                                                                                   \
      *ptr = val;                                                                                                      \
   }

#define GET_IF_NOT_NULL(ptr) ((NULL != ptr) ? *ptr : 0)

// Container functions containing the critical section should define this in the function.
#define SPF_MANAGE_CRITICAL_SECTION uint32_t lock_count = 0;

// Acquire lock for a critical section in a container.
#define SPF_CRITICAL_SECTION_START(gu_ptr)                                                                             \
   {                                                                                                                   \
      lock_count++;                                                                                                    \
      gu_critical_section_start_(gu_ptr);                                                                              \
   }

// Release lock for a critical section in a container.
#define SPF_CRITICAL_SECTION_END(gu_ptr)                                                                               \
   if (lock_count > 0)                                                                                                 \
   {                                                                                                                   \
      gu_critical_section_end_(gu_ptr);                                                                                \
      lock_count--;                                                                                                    \
   }

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _SPF_MACROS_H_
