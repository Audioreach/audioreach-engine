/**
 *   \file posal_mutex.c
 * \brief
 *  	This file contains utilities for using mutex functionalities.
 *
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "posal.h"
#include <pthread.h>
#include <errno.h>


/* ----------------------------------------------------------------------------
 * Global Declarations/Definitions
 * ------------------------------------------------------------------------- */
//#define DEBUG_POSAL_MUTEX

/* -------------------------------------------------------------------------
 * Function Definitions
 * ------------------------------------------------------------------------- */
/**
 Create and initializes a mutex. Recursive mutexes are always used.

 @datatypes
 #posal_mutex_t

 @param[in] pposal_mutex   Pointer to the mutex to initialize.

 @return
 Error code.

 @dependencies
 None. @newpage
 */
ar_result_t posal_mutex_create(posal_mutex_t *pposal_mutex, POSAL_HEAP_ID heap_id)
{
    pthread_mutex_t* mutex = NULL;
    pthread_mutexattr_t attr;
    ar_result_t rc = AR_EOK;

#ifdef DEBUG_POSAL_MUTEX
   AR_MSG(DBG_HIGH_PRIO, "posal_mutex_create");
#endif

#ifdef SAFE_MODE
    if (NULL == pposal_mutex)
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

    rc = pthread_mutexattr_init(&attr);
    if (rc) {
        rc = AR_EFAILED;
        AR_MSG(DBG_ERROR_PRIO,"%s: failed to initialize mutex attribute\n", __func__);
        goto fail;
    }

#if defined (ARSPF_PLATFORM_QNX)
    rc = pthread_mutexattr_setrecursive(&attr, PTHREAD_RECURSIVE_ENABLE );
#else
    rc = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
#endif
    if (rc) {
        rc = AR_EFAILED;
        AR_MSG(DBG_ERROR_PRIO,"%s: failed to set mutex attribute\n", __func__);
        goto fail2;
    }

    rc = pthread_mutex_init(mutex, &attr);
    if (rc) {
        rc = AR_EFAILED;
        AR_MSG(DBG_ERROR_PRIO,"%s: failed to initialize mutex\n", __func__);
        goto fail2;
    }

    rc = pthread_mutexattr_destroy(&attr);

	*pposal_mutex = mutex;

#ifdef DEBUG_POSAL_MUTEX
	AR_MSG(DBG_MED_PRIO, "mutex created (0x%p)|status(0x%x)", *pposal_mutex, AR_EOK);
#endif

    return AR_EOK;

fail2:
    pthread_mutexattr_destroy(&attr);

fail:
    free(mutex);

exit:
    return rc;
}

/**
 Detroys a mutex. This function must be called for each corresponding
 posal_mutex_init() function to clean up all resources.

 @datatypes
 #posal_mutex_t

 @param[in] pposal_mutex   Pointer to the mutex to destroy.

 @return
 None.

 @dependencies
 The object must have been created and initialized before calling this
 function.
 */
void posal_mutex_destroy(posal_mutex_t *pposal_mutex)
{
    pthread_mutex_t* mutex;
    int32_t rc;

#ifdef SAFE_MODE
    if (NULL == pposal_mutex)
    {
        AR_MSG(DBG_FATAL_PRIO, "Invalid input argument");
        return;
    }
#endif

    mutex = *pposal_mutex;
    rc = pthread_mutex_destroy(mutex);
    if (rc) {
        AR_MSG(DBG_ERROR_PRIO,"%s: Failed to destroy mutex\n", __func__);
        goto exit;
    }
    free(mutex);
    *pposal_mutex = NULL;

exit:
    return;
}

/**
  Attempts to lock a mutex. If the lock is already held, a failure is returned.

  @datatypes
  #posal_mutex_t

  @param[in] pposal_mutex   Pointer to the mutex to try locking.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  The object must have been created and initialized before calling this
  function. @newpage
 */
ar_result_t posal_mutex_try_lock(posal_mutex_t posal_mutex)
{
	ar_result_t status = AR_EOK;
    int32_t rc;
    pthread_mutex_t* mutex = posal_mutex;

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
  Locks a mutex. Recursive mutexes are always used.

  @datatypes
  #posal_mutex_t

  @param[in] pposal_mutex   Pointer to the mutex to lock.

  @return
  None.

  @dependencies
  The object must have been created and initialized before calling this
  function.
 */
void posal_mutex_lock(posal_mutex_t posal_mutex)
{
    int32_t rc;
    pthread_mutex_t* mutex = posal_mutex;

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
  Unlocks a mutex. Recursive mutexes are always used.

  @datatypes
  #posal_mutex_t

  @param[in] pposal_mutex   Pointer to the mutex to unlock.

  @return
  None.

  @dependencies
  The object must have been created and initialized before calling this
  function.
 */

void posal_mutex_unlock(posal_mutex_t posal_mutex)
{
    int32_t rc;
    pthread_mutex_t* mutex = posal_mutex;

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

/** @} */ /* end_addtogroup posal_mutex */
