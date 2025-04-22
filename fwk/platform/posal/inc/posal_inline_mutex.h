/**
 * \file posal_mutex_inline.h
 * \brief
 *  	 An alterntitive mutex api for statically linked libraries to have lower memory overhead
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_MUTEX_INLINE_H
#define POSAL_MUTEX_INLINE_H

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "ar_defs.h"
#include "qurt.h"
#include "ar_error_codes.h"
#include "qurt_pimutex.h"
#include "posal_internal_inline.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus


typedef qurt_mutex_t posal_inline_mutex_t;

/**
 Initializes a mutex. Recursive mutexes are always used.

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
static inline ar_result_t posal_inline_mutex_init(posal_inline_mutex_t *pposal_mutex)
{
    qurt_pimutex_init((qurt_mutex_t *)pposal_mutex);
    return AR_EOK;
}

/**
 Deinits a mutex. This function must be called for each corresponding
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
static inline void posal_inline_mutex_deinit(posal_inline_mutex_t *pposal_mutex)
{
   qurt_pimutex_destroy((qurt_mutex_t *)pposal_mutex);
}


#ifdef __cplusplus
}
#endif //__cplusplus

#endif //POSAL_MUTEX_INLINE_H