/**
 * \file posal_nmutex.h
 * \brief 
 *  	 This file contains normal mutex utilities.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_NMUTEX_H
#define POSAL_NMUTEX_H

#include "ar_error_codes.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** @addtogroup posal_nmutex
@{ */

/* -----------------------------------------------------------------------
** Global definitions/forward declarations
** ----------------------------------------------------------------------- */
/** posal normal mutex type structure. */
typedef void  *posal_nmutex_t;

/****************************************************************************
** Normal Mutex APIs
*****************************************************************************/
/**
  Initializes a normal mutex.

  @datatypes
  #posal_nmutex_t

  @param[in] pposal_nmutex   Pointer to the normal mutex.

  @return
  None.

  @dependencies
  None. @newpage
*/
ar_result_t posal_nmutex_create(posal_nmutex_t *pposal_nmutex, POSAL_HEAP_ID heap_id);

/**
  Locks a normal mutex.

  @datatypes
  #posal_nmutex_t

  @param[in] pposal_nmutex   Pointer to the normal mutex.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
void posal_nmutex_lock(posal_nmutex_t posal_nmutex);

/**
  Attempts to lock a normal mutex. If the lock is already locked and
  unavailable, a failure is returned.

  @datatypes
  #posal_nmutex_t

  @param[in] posal_nmutex   Normal mutex.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  Before calling this function, the object must be created and initialized.
  @newpage
*/
int posal_nmutex_try_lock(posal_nmutex_t posal_nmutex);

/**
  Unlocks a normal mutex.

  @datatypes
  #posal_nmutex_t

  @param[in] posal_nmutex   Normal mutex.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
void posal_nmutex_unlock(posal_nmutex_t posal_nmutex);

/**
  Destroys a normal mutex. This function must be called for each corresponding
  posal_nmutex_init() function to clean up all resources.

  @datatypes
  #posal_nmutex_t

  @param[in] posal_nmutex  Normal mutex.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
void posal_nmutex_destroy(posal_nmutex_t *pp_posal_nmutex);

/** @} */ /* end_addtogroup posal_nmutex */

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_NMUTEX_H

