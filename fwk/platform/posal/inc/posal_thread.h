/**
 * \file posal_thread.h
 * \brief
 *  	 This file contains utilities for threads. Threads must be joined to
 *  	avoid memory leaks. This file provides functions to create and destroy threads,
 *  	and to change thread priorities.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_THREAD_H
#define POSAL_THREAD_H

#include "posal_memory.h"
#include "ar_error_codes.h"
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

   /** @addtogroup posal_thread
@{ */

   /* -----------------------------------------------------------------------
    ** Global definitions/forward declarations
    ** ----------------------------------------------------------------------- */

   /** Handle to a thread. */
   typedef void* posal_thread_t;

   /** thread priority number */
   typedef int32_t posal_thread_prio_t;

   /****************************************************************************
    ** Threads
    *****************************************************************************/

   /**
  Creates and launches a thread.

  @datatypes
  #posal_thread_t \n
  #POSAL_HEAP_ID

  @param[out] pTid           Pointer to the thread ID.
  @param[in]  pzThreadName   Pointer to the thread name.
  @param[in]  pStack         Pointer to the location of the pre-allocated stack
                             (NULL causes a new allocation). \n
                             pStack must point to the lowest address in the
                             stack.
  @param[in]  nStackSize     Size of the thread stack.
  @param[in]  nPriority      Thread priority, where 0 is the lowest priority
                             and 255 is the highest priority.
  @param[in]  pfStartRoutine Pointer to the entry function of the thread.
  @param[in]  arg            Pointer to the arguments passed to the entry
                             function. An argument can be to any pointer type.
  @param[in]  heap_id        ID of the heap to which the thread stack is
                             allocated.

  @detdesc
  The thread stack can be passed in as the pStack argument, or pStack=NULL
  indicates that posal allocates the stack internally. If the caller
  provides the stack, the caller is responsible for freeing the stack memory
  after joining the thread.
  @par
  Pre-allocated stacks must be freed after the dying thread is joined. The
  caller must specify the heap in which the thread stack is to be allocated.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  None. @newpage
    */
   ar_result_t posal_thread_launch( posal_thread_t *pTid,
                                   char *pzThreadName,
                                   size_t nStackSize,
                                   posal_thread_prio_t nPriority,
                                   ar_result_t (*pfStartRoutine)(void *),
                                   void* arg,
                                   POSAL_HEAP_ID heap_id);

   /**
  Creates and launches a island thread with nrootStackSize stack size used for guest mode.

  @datatypes
  #posal_thread_t \n
  #POSAL_HEAP_ID

  @param[out] pTid           Pointer to the thread ID.
  @param[in]  pzThreadName   Pointer to the thread name.
  @param[in]  pStack         Pointer to the location of the pre-allocated stack
                             (NULL causes a new allocation). \n
                             pStack must point to the lowest address in the
                             stack.
  @param[in]  nStackSize     Size of the thread stack.
  @param[in]  nrootStackSize     Size of the island thread stack used for guest mode.
  @param[in]  nPriority      Thread priority, where 0 is the lowest priority
                             and 255 is the highest priority.
  @param[in]  pfStartRoutine Pointer to the entry function of the thread.
  @param[in]  arg            Pointer to the arguments passed to the entry
                             function. An argument can be to any pointer type.
  @param[in]  heap_id        ID of the heap to which the thread stack is
                             allocated.

  @detdesc
  The thread stack can be passed in as the pStack argument, or pStack=NULL
  indicates that posal allocates the stack internally. If the caller
  provides the stack, the caller is responsible for freeing the stack memory
  after joining the thread.
  @par
  Pre-allocated stacks must be freed after the dying thread is joined. The
  caller must specify the heap in which the thread stack is to be allocated.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  None. @newpage
    */
   ar_result_t posal_thread_launch2(posal_thread_t *pTid,
                                   char *pzThreadName,
                                   size_t nStackSize,
								   size_t nrootStackSize,
                                   posal_thread_prio_t nPriority,
                                   ar_result_t (*pfStartRoutine)(void *),
                                   void* arg,
                                   POSAL_HEAP_ID heap_id);

   /*
   Creates and launches a island thread with nrootStackSize stack size used for guest mode.
    pTid           : Thread ID.
    threadname     : Thread name.
    stack_ptr      : Location of a preallocated stack (NULL causes a new allocation).
                     stack_ptr must point to the lowest address in the stack.
    stack_size     : Size of the thread stack.
    root_stack_size: Size of the island thread stack in guestmode.
    nPriority      : Thread priority, where 0 is the lowest and 255 is the highest.
    pfStartRoutine : Entry function of the thread.
    arg            : This parameter is passed to the entry
                     function and can be a case to any pointer type.
    heap_id        : Thread stack will be allocated in the heap
                     specified by this parameter.
    scheduling policy: depending on the platform. SCHED_FIFO, SCHED_RR, and
                       SCHED_OTHER. Posix: https://man7.org/linux/man-pages/man3/pthread_attr_setschedpolicy.3.html
                       0xFFFFFFFF is ignored.
    affinity        : affinity mask. value of 0 is ignored.
    return
    An indication of success or failure.
    */
   ar_result_t posal_thread_launch3(posal_thread_t     *posal_obj_ptr,
                                    char               *threadname,
                                    size_t              stack_size,
                                    size_t              root_stack_size,
                                    posal_thread_prio_t nPriority,
                                    ar_result_t         (*pfStartRoutine)(void *),
                                    void               *arg,
                                    POSAL_HEAP_ID       heap_id,
                                    uint32_t            sched_policy,
                                    uint32_t            affinity);

   /**
  Waits for a specified thread to exit, and collects the exit status.

  @datatypes
  #posal_thread_t

  @param[in]  nTid     Thread ID to wait on.
  @param[out] nStatus  Pointer to the value returned by pfStartRoutine
                       called in posal_thread_launch().

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
    */
   void posal_thread_join(posal_thread_t nTid, ar_result_t* nStatus);

   /**
  Queries the thread id of the given thread object.

  @return
  The thread id which is integer value.

  @dependencies

  @newpage
 */
int32_t posal_thread_get_tid(posal_thread_t thread_obj);

int64_t posal_thread_get_tid_v2(posal_thread_t thread_obj);


/**
  Queries the thread id of the caller.

  @return
  The thread id which is integer value.

  @dependencies

  @newpage
    */
int32_t posal_thread_get_curr_tid(void);
int64_t posal_thread_get_curr_tid_v2(void);

/**
  Get the thread name.

  @return
  None.

  @dependencies
  None.

  @newpage
*/
void posal_thread_get_name(char *name, unsigned char max_len);

/**
  Queries the thread priority of the caller.

  @return
  The thread priority of the caller, where 0 is the lowest priority and 255 is
  the highest priority.

  @dependencies
  Before calling this function, the object must be created and initialized.

  @newpage
*/
posal_thread_prio_t posal_thread_prio_get(void);

/**
  Queries the thread priority of the specified thread.

  @param[in] tObj   Thread Object

  @return
  The thread priority of the specified thread, where 0 is the lowest priority and 255 is
  the highest priority.

  @dependencies
  Before calling this function, the object must be created and initialized.

  @newpage
*/
posal_thread_prio_t posal_thread_prio_get2(posal_thread_t tObj);

/**
  Changes the thread priority of the caller.

  @param[in] nPrio  New priority, where 0 is the lowest priority and 255 is the
                    highest priority.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
void posal_thread_set_prio(posal_thread_prio_t nPrio);

/**
  Changes the thread priority of the specified thread.
  @param[in] tObj   Thread Object
  @param[in] nPrio  New priority, where 0 is the lowest priority and 255 is the
                    highest priority.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
void posal_thread_set_prio2(posal_thread_t tObj, posal_thread_prio_t nPrio);

/**


  @param[in]  pStackpointer  Pointer to the stack pointer
  @param[in]  nStackSize     Size of the thread stack.
  @param[in]  heap_id     Heap ID


  @return
  Indication of success (0) or failure (nonzero).

*/
ar_result_t posal_thread_allocate_stack(uint32_t nStackSize, char **pStackpointer, POSAL_HEAP_ID heap_id);

/** @} */ /* end_addtogroup posal_thread */

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_THREAD_H
