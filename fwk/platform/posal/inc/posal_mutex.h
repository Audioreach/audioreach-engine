/**
 * \file posal_mutex.h
 * \brief
 *  	 This file contains mutex utilites. Recursive mutexes are always used for thread-safe programming.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_MUTEX_H
#define POSAL_MUTEX_H

#include "posal_types.h"
#include "posal_memory.h"
#include "ar_error_codes.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

   /** @addtogroup posal_mutex
@{ */

   /* -----------------------------------------------------------------------
    ** Global definitions/forward declarations
    ** ----------------------------------------------------------------------- */

   /** posal mutex data type*/
   typedef void  *posal_mutex_t;

   /****************************************************************************
    ** Mutexes
    *****************************************************************************/

   /**
  Creates and initializes a mutex. Recursive mutexes are always used.

  @datatypes
  posal_mutex_t

  @param[in] posal_mutex pointer to the mutex object handle.

  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  None. @newpage
    */
   ar_result_t posal_mutex_create(posal_mutex_t *pposal_mutex, POSAL_HEAP_ID heap_id);

   /**
  Deletes a mutex. This function must be called for each corresponding
  posal_mutex_create() function to clean up all resources.

  @datatypes
  posal_mutex_t

  @param[in] posal_mutex mutex object handle.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created.
  @newpage
    */
   void posal_mutex_destroy(posal_mutex_t *posal_mutex);

   /** @} */ /* end_addtogroup posal_mutex2 */

   /**
  Locks a mutex. Recursive mutexes are always used.

  @datatypes
  #posal_mutex_t

  @param[in] posal_mutex mutex object handle.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
    */
   void posal_mutex_lock(posal_mutex_t posal_mutex);

   /**
  Attempts to lock a mutex. If the mutex is already locked and unavailable,
  a failure is returned.

  @datatypes
  #posal_mutex_t

  @param[in] posal_mutex mutex object handle.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  Before calling this function, the object must be created and initialized.
  @newpage
    */
   ar_result_t posal_mutex_try_lock(posal_mutex_t posal_mutex);

   /**
  Unlocks a mutex. Recursive mutexes are always used.

  @datatypes
  #posal_mutex_t

  @param[in]  posal_mutex mutex object handle.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
    */
   void posal_mutex_unlock(posal_mutex_t posal_mutex);

   /** @} */ /* end_addtogroup posal_mutex */

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_MUTEX_H
