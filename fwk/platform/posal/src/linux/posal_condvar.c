/**
 * \file posal_condvar.c
 * \brief
 *      This file contains a utility to form a channel of a combination of up to
 *  32 signals/queues/timers. Client can wait on any combination thereof and
 *  be woken when any desired element is active
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal_condvar.h"
#include <pthread.h>
/**
  Initializes a pthread condition variable object.

  @datatypes
  #posal_condvar_t

  @param[in] condition_var_ptr  Pointer to the pthread condition variable.

  @return
  None.

  @dependencies
  None. @newpage
 */
ar_result_t posal_condvar_create(posal_condvar_t *p_cndvar, POSAL_HEAP_ID heap_id)
{
   int32_t rc = 0;
   if (NULL == p_cndvar)
   {
      AR_MSG(DBG_FATAL_PRIO, "Invalid input argument");
      return AR_EFAILED;
   }

   void *temp = (pthread_cond_t  *)posal_memory_malloc(sizeof(pthread_cond_t ), heap_id);
   if (NULL == temp)
   {
      AR_MSG(DBG_FATAL_PRIO, "Failed to allocate memory for mutex.");
      *p_cndvar = NULL;
      rc =  AR_ENOMEMORY;
      goto exit;
   }

   rc = pthread_cond_init((pthread_cond_t *)temp, NULL);
   if (rc) {
      AR_MSG(DBG_ERROR_PRIO,"Failed to initialize conditional variable");
      *p_cndvar = NULL;
      rc = AR_EFAILED;
      goto fail;
   }

   *p_cndvar = (posal_condvar_t) temp;
   temp = NULL;

fail:
   posal_memory_free((void *)temp);

exit:
   return rc;
}


/**
  Signals a condition variable object. This utility is used to awaken a single
  waiting thread.

  @datatypes
  #posal_condvar_t

  @param[in] condition_var_ptr  Pointer to the pthread condition variable.

  @return
  None.

  @dependencies
  The object must have been created and initialized before calling this
  function.
 */
void posal_condvar_signal(posal_condvar_t p_cndvar)
{
   int32_t rc = 0;

   rc = pthread_cond_signal((pthread_cond_t *)p_cndvar);
   if (rc) {
      AR_MSG(DBG_ERROR_PRIO,"Failed to signal conditional variable with rc: %d", rc);
   }
}


/**
  Broadcasts a condition variable object. This utility is used to awaken
  multiple threads waiting for a condition variable.

  @datatypes
  #posal_condvar_t

  @param[in] condition_var_ptr  Pointer to the pthread condition variable.

  @return
  None.

  @dependencies
  The object must have been created and initialized before calling this
  function. @newpage
 */
void posal_condvar_broadcast(posal_condvar_t p_cndvar)
{
   int32_t rc = 0;

   rc = pthread_cond_broadcast((pthread_cond_t *)p_cndvar);
   if (rc) {
      AR_MSG(DBG_ERROR_PRIO,"Failed to broadcast conditional variable with rc: %d", rc);
   }
}

/**
  Waits for a condition variable object. This utility suspends the current
  thread until the specified condition is true.

  @datatypes
  #posal_condvar_t \n
  #posal_nmutex_t

  @param[in] condition_var_ptr  Pointer to the pthread condition variable.
  @param[in] nmutex             Normal Mutex associated with the condition
                                variable.

  @return
  None.

  @dependencies
  The object must have been created and initialized before calling this
  function.
 */
void posal_condvar_wait(posal_condvar_t p_cndvar, posal_nmutex_t p_nmutex)
{
   int32_t rc = 0;
   rc = pthread_cond_wait((pthread_cond_t *)p_cndvar, (pthread_mutex_t*)p_nmutex);
   if (rc) {
      AR_MSG(DBG_ERROR_PRIO,"Failed to wait on conditional variable with rc: %d", rc);
   }
}

/**
  Destroys a condition variable object.

  @datatypes
  #posal_condvar_t

  @param[in] condition_var_ptr  Pointer to the pthread condition variable.

  @return
  None.

  @dependencies
  The object must have been created and initialized before calling this
  function. @newpage
*/
void posal_condvar_destroy(posal_condvar_t *pp_cndvar)
{
   int32_t rc = 0;
   rc = pthread_cond_destroy((pthread_cond_t  *)*pp_cndvar);
   if (rc) {
      AR_MSG(DBG_ERROR_PRIO,"Failed to destroy conditional variable with rc: %d", rc);
   }
   posal_memory_free((void *)*pp_cndvar);
   *pp_cndvar = NULL;
}
