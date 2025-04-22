#ifndef AR_OSAL_THREAD_H
#define AR_OSAL_THREAD_H

/**
 * \file ar_osal_thread.h
 * \brief
 *        Defines public APIs for thread.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/** @weakgroup weakf_osal_thread_intro
Threads must be joined to avoid memory leaks. The following thread functions
are used to create and destroy threads, and to change thread priorities.
- ar_osal_thread_create()
- ar_osal_thread_attr_init()
- ar_osal_thread_join_destroy()
- ar_osal_thread_get_priority()
- ar_osal_thread_set_priority()
- ar_osal_thread_self_get_priority()
- ar_osal_thread_self_set_priority()
*/

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/
/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "ar_osal_types.h"

/** @addtogroup osal_thread
@{ */
#include "ar_osal_types.h"
/* -----------------------------------------------------------------------
** Global definitions/forward declarations
** ----------------------------------------------------------------------- */

/** Handle to a thread. */
typedef void * ar_osal_thread_t;

/** Thread attributes. */
typedef struct ar_osal_thread_attr_t
{
    char_t    *thread_name;       /**< Pointer to the thread name */
    uint32_t  stack_size;         /**< Size of the thread stack */
    int32_t   priority;           /**< Thread priority */
}ar_osal_thread_attr_t;

/****************************************************************************
** Threads
*****************************************************************************/
/**
   osal thread start function definition.

   @param[in]   thread_param: The client supplied data pointer when creating
                                   a thread.

 */
typedef void (*ar_osal_thread_start_routine) (void *thread_param);


/**
  Thread attribute initialization API, client allocates the memory for ar_osal_thread

  @datatypes
  osal_thread_attribute_t

  @param[in]  thread_attr: Initialize thread object.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  None. @newpage
*/
int32_t ar_osal_thread_attr_init(ar_osal_thread_attr_t *thread_attr);

/**
  Creates and launches a thread.

  @datatypes
  ar_osal_thread_t

  @param[out] thread:                  Pointer to the thread.
  @param[in]  thread_attr:             Pointer to the thread attributes.
  @param[in]  thread_start:            Pointer to the entry function of the thread.
  @param[in]  thread_param:            Pointer to the arguments passed to the entry
                                       function.
                                       An argument can be to any pointer type.
  @detdesc
  The thread stack can be passed in as the osal_thread_stack argument, or
  osal_thread_stack=NULL indicates that osal allocates the stack internally.
  If the caller provides the stack, the caller is responsible for freeing
  the stack memory after joining the thread.
  @par
  Pre-allocated stacks must be freed after the dying thread is joined. The
  caller must specify the heap in which the thread stack is to be allocated.

  @return
  0 -- Success
  Nonzero -- Failure.

  @dependencies
  None. @newpage
*/
int32_t ar_osal_thread_create( ar_osal_thread_t *thread,
                                 ar_osal_thread_attr_t *attr_ptr,
                                 ar_osal_thread_start_routine thread_start,
                                 void *thread_param);

/**
  Get current thread id.


  @param[Out]  thread id: Current thread id.

  @return
  Thread id.

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
int64_t ar_osal_thread_get_id(void);

/**
  Waits for a specified thread to exit, and collects the exit status
  and clear the thread object

  @datatypes
  ar_osal_thread_t

  @param[in]  thread:          Thread to wait on.

  @return
  0 -- Success.
  Nonzero -- Failure.

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
int32_t ar_osal_thread_join_destroy(ar_osal_thread_t thread);

/**
  Queries the thread priority using a thread handle.

  @param[in]  thread: Thread handle.
  @param[out] get_priority: current thread priority

  0 -- Success.
  Nonzero -- Failure.

  @dependencies
  Before calling this function, the object must be created and initialized.
  @newpage
*/
int32_t ar_osal_thread_get_priority(ar_osal_thread_t thread, int32_t *get_priority);

/**
  Changes the thread priority using a thread handle.

  @param[in]  thread: Thread handle.
  @param[in] set_priority:  New thread priority to set

  @return
  0 -- Success.
  Nonzero -- Failure.

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
int32_t ar_osal_thread_set_priority(ar_osal_thread_t thread, int32_t set_priority);

/**
  Queries the thread priority within the thread work function.

  @param[out] get_priority: get thread priority

  @return
  0 -- Success.
  Nonzero -- Failure.

  @dependencies
  Before calling this function, the object must be created and initialized.
  @newpage
*/
int32_t ar_osal_thread_self_get_priority(int32_t *get_priority);

/**
  Changes the thread priority within the thread work function.

  @param[in] set_priority:  New priority to set

  @return
  0 -- Success.
  Nonzero -- Failure.

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
int32_t ar_osal_thread_self_set_priority(int32_t set_priority);

/** @} */ /* end_addtogroup ar_osal_thread */

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif // #ifndef AR_OSAL_THREAD_H

