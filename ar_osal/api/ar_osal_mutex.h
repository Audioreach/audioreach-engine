#ifndef AR_OSAL_MUTEX_H
#define AR_OSAL_MUTEX_H

/**
 * \file ar_osal_mutex.h
 * \brief
 *     This file contains mutex APIs. Recursive mutexes are always used
*     for thread-safe programming.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/** @weakgroup weakf_ar_osal_mutex_intro
This section describes the following priority inheritance mutex functions.
Recursive mutex are always used for thread-safe programming.
 - ar_osal_mutex_init()
 - ar_osal_mutex_deinit()
 - ar_osal_mutex_get_size()
 - ar_osal_mutex_create()
 - ar_osal_mutex_destroy()
 - ar_osal_mutex_lock()
 - ar_osal_mutex_try_lock()
 - ar_osal_mutex_unlock()
*/

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "ar_osal_types.h"

/** @addtogroup mutex
@{ */

/* -----------------------------------------------------------------------
** Global definitions/forward declarations
** ----------------------------------------------------------------------- */

/** ar osal mutex type object.
*/
typedef void *ar_osal_mutex_t;

/****************************************************************************
** Mutex
*****************************************************************************/

/**
  Initializes a mutex. Memory for mutex object is created by client and
  this API is used to initialize the mutex.
  Recursive mutexes are always used.

  @datatypes
  ar_osal_mutex_t

  @param[in] mutex: Pointer to the mutex object handle.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  None. @newpage
*/
int32_t ar_osal_mutex_init(ar_osal_mutex_t mutex);

/**
  De-initializes a mutex. Mutex memory was provided by client and this
  API is used to de-initialize the mutex before freeing the object.

  @datatypes
  ar_osal_mutex_t

  @param[in] mutex: Pointer to the mutex object handle.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  None. @newpage
*/
int32_t ar_osal_mutex_deinit(ar_osal_mutex_t mutex);

/**
  Get size of mutex object. This API is used to get the mutex object
  size for client to allocate memory for the mutex object.

  @datatypes
  ar_osal_mutex_t

  @return
  Size of mutex object

  @dependencies
  None. @newpage
*/
size_t ar_osal_mutex_get_size(void);


/**
  Creates and initialize a mutex. Recursive mutex are always used.
  Mutex memory allocation and initialization done by this API
  osal_mutex_create() or osal_mutex_init() would be used, but not both.

  @datatypes
  ar_osal_mutex_t

  @param[Out] mutex: Pointer to the mutex object handle.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  None. @newpage
*/
int32_t ar_osal_mutex_create(ar_osal_mutex_t *mutex);

/**
  Delete/free a mutex object. This function must be called for
  each corresponding osal_mutex_create function to clean up all
  resources.

  @datatypes
  ar_osal_mutex_t

  @param[in] mutex: Pointer to the mutex.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  Before calling this function, the object must have been created.
  @newpage
*/
int32_t ar_osal_mutex_destroy(ar_osal_mutex_t mutex);

/**
  Locks a mutex. Recursive mutex are always used.

  @datatypes
  ar_osal_mutex_t

  @param[in] mutex: Pointer to the mutex.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  Before calling this function, the object must be created.
  @newpage

*/
int32_t ar_osal_mutex_lock(ar_osal_mutex_t mutex);

/**
  Attempts to lock a mutex. If the mutex is already locked and unavailable,
  a failure is returned.

  @datatypes
  ar_osal_mutex_t

  @param[in] mutex: Pointer to the mutex.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  Before calling this function, the object must be created.
  @newpage
*/
int32_t ar_osal_mutex_try_lock(ar_osal_mutex_t mutex);

/**
  Unlocks a mutex. Recursive mutex are always used.

  @datatypes
  ar_osal_mutex_t

  @param[in] mutex: Pointer to the mutex.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  Before calling this function, the object must be created.
  @newpage
 */
int32_t ar_osal_mutex_unlock(ar_osal_mutex_t mutex);


#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // #ifndef AR_OSAL_MUTEX_H
