/**
 * \file posal_condvar.h
 * \brief 
 *  	 This file contains the ConditionVariables utilities.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_CONDITIONVARIABLE_H
#define POSAL_CONDITIONVARIABLE_H

#include "posal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** @addtogroup posal_condvar
@{ */

/****************************************************************************
** ConditionVariables
*****************************************************************************/
typedef void * posal_condvar_t;

/**
  Creates a condition variable object.

  @datatypes
  #posal_condvar_t

  @param[in] condition_var_ptr  Pointer to the condition variable.

  @return
  0 -- Success
  @par
  Nonzero -- Failure 
  
  @dependencies
  None. @newpage
 */
ar_result_t posal_condvar_create(posal_condvar_t *p_cndvar, POSAL_HEAP_ID heap_id);


/**
  Signals a condition variable object. This utility is used to awaken a single
  waiting thread.

  @datatypes
  #posal_condvar_t

  @param[in] condition_var_ptr  Pointer to the condition variable.

  @return
  None.
  
  @dependencies
  Before calling this function, the object must be created.
 */
void posal_condvar_signal(posal_condvar_t p_cndvar);

/**
  Broadcasts a condition variable object. This utility is used to awaken
  multiple threads waiting for a condition variable.

  @datatypes
  #posal_condvar_t

  @param[in] condition_var_ptr  Pointer to the condition variable.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
  @newpage
 */
void posal_condvar_broadcast(posal_condvar_t p_cndvar);

/**
  Waits for a condition variable object. This utility suspends the current
  thread until the specified condition is true.

  @datatypes
  #posal_condvar_t \n
  #posal_nmutex_t

  @param[in] condition_var_ptr  Pointer to the condition variable.
  @param[in] nmutex             Normal mutex associated with the condition
                                variable.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
 */
void posal_condvar_wait(posal_condvar_t p_cndvar, posal_nmutex_t nmutex);

/**
  Destroys a condition variable object.

  @datatypes
  #posal_condvar_t

  @param[in] condition_var_ptr  Pointer to the condition variable.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
  @newpage
*/
void posal_condvar_destroy(posal_condvar_t *pp_cndvar);

/** @} */ /* end_addtogroup posal_condvar */
#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_CONDITIONVARIABLE_H
