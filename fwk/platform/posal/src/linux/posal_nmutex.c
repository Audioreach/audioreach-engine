/**
 * \file posal_nmutex.c
 * \brief
 *      This file contains normal mutex utilities like initialization mutex, lock,unlock, try lock and destroy mutex.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "posal.h"
#include <pthread.h>
#include <errno.h>

/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/
/**
  Initializes a normal mutex.

  @datatypes
  #posal_nmutex_t

  @param[in] pposal_nmutex   Normal mutex to initialize.

  @return
  None.

  @dependencies
  None. @newpage
*/

ar_result_t posal_nmutex_create(posal_nmutex_t *pposal_nmutex, POSAL_HEAP_ID heap_id)
{
    pthread_mutex_t* mutex = NULL;
    ar_result_t rc = AR_EOK;

#ifdef DEBUG_POSAL_MUTEX
   AR_MSG(DBG_HIGH_PRIO, "posal_nmutex_create");
#endif

#ifdef SAFE_MODE
    if (NULL == pposal_nmutex)
    {
        AR_MSG(DBG_FATAL_PRIO, "Invalid input argument");
        return AR_EBADPARAM;
    }
#endif

    mutex = ((pthread_mutex_t *) malloc(sizeof(pthread_mutex_t)));
    if (NULL == mutex) {
        AR_MSG(DBG_ERROR_PRIO,"%s: failed to allocate memory for mutex\n", __func__);
        rc = AR_ENOMEMORY;
        goto exit;
    }

    rc = pthread_mutex_init(mutex, NULL);
    if (rc) {
        rc = AR_EFAILED;
        AR_MSG(DBG_ERROR_PRIO,"%s: failed to initialize mutex\n", __func__);
        goto fail;
    }

    *pposal_nmutex = mutex;

#ifdef DEBUG_POSAL_MUTEX
    AR_MSG(DBG_MED_PRIO, "mutex created (0x%p)|status(0x%x)", *pposal_nmutex, AR_EOK);
#endif
    mutex = NULL;
fail:
    free(mutex);
    mutex = NULL;
exit:
    return rc;
}

/**
  Locks a normal mutex.

  @datatypes
  #posal_nmutex_t

  @param[in] pposal_nmutex   Normal mutex to lock.

  @return
  None.

  @dependencies
  The object must have been created and initialized before calling this
  function.
*/
void posal_nmutex_lock(posal_nmutex_t posal_nmutex)
{
    int32_t rc;
    pthread_mutex_t* mutex = posal_nmutex;

#ifdef SAFE_MODE
    if (NULL == mutex) {
        AR_MSG(DBG_FATAL_PRIO, "Invalid input argument");
        return;
    }
#endif

    rc = pthread_mutex_lock(mutex);
#ifdef DEBUG_POSAL_MUTEX
    if (rc) {
        AR_MSG(DBG_ERROR_PRIO,"%s: Failed to lock mutex\n", __func__);
    }
#endif
    return;
}

/**
  Attempts to lock a normal mutex. If the lock is already held, a failure is returned.

  @datatypes
  #posal_nmutex_t

  @param[in] pposal_nmutex   Normal mutex to try locking.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  The object must have been created and initialized before calling this
  function. @newpage
*/
int posal_nmutex_try_lock(posal_nmutex_t posal_nmutex)
{
    ar_result_t status = AR_EOK;
    int32_t rc;
    pthread_mutex_t* mutex = posal_nmutex;

#ifdef SAFE_MODE
    if (NULL == mutex) {
        AR_MSG(DBG_FATAL_PRIO, "Invalid input argument");
        return AR_EHANDLE;
    }
#endif

    rc = pthread_mutex_trylock(mutex);
    if (rc) {
        AR_MSG(DBG_ERROR_PRIO,"%s: Failed to lock mutex\n", __func__);
        status = AR_EFAILED;
    }
    return status;
}

/**
  Unlocks a normal mutex.

  @datatypes
  #posal_nmutex_t

  @param[in] pposal_nmutex   Normal mutex to unlock.

  @return
  None.

  @dependencies
  The object must have been created and initialized before calling this
  function.
*/
void posal_nmutex_unlock(posal_nmutex_t posal_nmutex)
{
    int32_t rc;
    pthread_mutex_t* mutex = posal_nmutex;

#ifdef SAFE_MODE
    if (NULL == mutex) {
        AR_MSG(DBG_FATAL_PRIO, "Invalid input argument");
        return;
    }
#endif

    rc = pthread_mutex_unlock(mutex);
#ifdef DEBUG_POSAL_MUTEX
    if (rc) {
        AR_MSG(DBG_ERROR_PRIO,"%s: Failed to release mutex\n", __func__);
    }
#endif
    return;
}

/**
  Destroys a normal mutex. This function must be called for each corresponding
  posal_nmutex_init() function to clean up all resources.

  @datatypes
  #posal_nmutex_t

  @param[in] pposal_nmutex   Normal mutex to destroy.

  @return
  None.

  @dependencies
  The object must have been created and initialized before calling this
  function.
*/
void posal_nmutex_destroy(posal_nmutex_t *pp_posal_nmutex)
{
    pthread_mutex_t* mutex;
    int32_t rc;

#ifdef SAFE_MODE
    if (NULL == pp_posal_nmutex)
    {
        AR_MSG(DBG_FATAL_PRIO, "Invalid input argument");
        return;
    }
#endif

    mutex = *pp_posal_nmutex;
    rc = pthread_mutex_destroy(mutex);
    if (rc) {
        AR_MSG(DBG_ERROR_PRIO,"%s: Failed to destroy mutex\n", __func__);
        goto exit;
    }
    free(mutex);
    *pp_posal_nmutex = NULL;

exit:
    return;
}

/** @} */ /* end_addtogroup posal_nmutex */
