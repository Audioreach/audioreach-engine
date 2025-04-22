/**
 * \file posal_atomic.h
 * \brief
 *  	 This file is for atomic operations. This file provides functions for thread-safe operations on atomic variables.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_ATOMIC_H
#define POSAL_ATOMIC_H

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */

#include "ar_defs.h"
#include "posal_memory.h"
#include "ar_msg.h"
#include "atomic_ops.h"
#include "ar_error_codes.h"
#include "ar_osal_error.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** @addtogroup posal_atomic
@{ */

/* -----------------------------------------------------------------------
** Global definitions/forward declarations
** ----------------------------------------------------------------------- */
/** Atomic variable type. @newpage */
typedef atomic_word_t posal_atomic_word_internal_t;
typedef void* posal_atomic_word_t;

/****************************************************************************
** Atomic Ops
*****************************************************************************/

/**
  Sets a value atomically.

  @datatypes
  #posal_atomic_word_t

  @param[in] pWord  Pointer to the atomic variable.
  @param[in] val    Value to which to set the atomic variable.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  None.
*/
static inline void posal_atomic_set(posal_atomic_word_t pWord, int val)
{
   (void) atomic_set(pWord, (atomic_plain_word_t)val);
}

/**
  Gets the value of an atomic variable.

  @datatypes
  #posal_atomic_word_t

  @param[in] pWord  Pointer to the atomic variable.

  @return
  The new value of the atomic variable.

  @dependencies
  None. @newpage
*/
static inline int posal_atomic_get(posal_atomic_word_t pWord)
{
   return (int)(((atomic_word_t*)pWord)->value);
}

/**
   Creates and initializes an atomic variable.

   @datatypes
   posal_atomic_word_t

   @param[in] ppatomic_word pointer to the atomic word object handle.

   @return
   0 -- Success
   @par
   Nonzero -- Failure

   @dependencies
   None. @newpage
  */
 static inline ar_result_t posal_atomic_word_create(posal_atomic_word_t *ppatomic_word, POSAL_HEAP_ID heap_id)
 {
    if (NULL == ppatomic_word)
    {
       return AR_EFAILED;
    }
    *ppatomic_word = (posal_atomic_word_t) posal_memory_malloc(sizeof(posal_atomic_word_internal_t), heap_id);
    posal_atomic_set(*ppatomic_word, 0);
    return AR_EOK;
 }

 /**
   Deletes the atmoic word. This function must be called for each corresponding
   posal_atomic_word_create() function to clean up all resources.

   @datatypes
   posal_atomic_word_t

   @param[in] patomic_word object handle.

   @return
   None.

   @dependencies
   Before calling this function, the object must be created.
   @newpage
  */
 static inline void posal_atomic_word_destroy(posal_atomic_word_t patomic_word)
 {
    if (patomic_word)
    {
       posal_memory_free(patomic_word);
    }
    return;
 }


/**
  Increments an atomic variable.

  @datatypes
  #posal_atomic_word_t

  @param[in] pWord  Pointer to the atomic variable.

  @return
  The new value of the atomic variable.

  @dependencies
  None.
*/
static inline int posal_atomic_increment(posal_atomic_word_t pWord)
{
   return (int)atomic_inc_return(pWord);
}

/**
  Adds to an atomic variable.

  @datatypes
  #posal_atomic_word_t

  @param[in] pWord  Pointer to the atomic variable.
  @param[in] unVal  Number to add to the atomic variable.

  @return
  The new value of the atomic variable.

  @dependencies
  None. @newpage
*/
static inline void posal_atomic_add(posal_atomic_word_t pWord, uint32_t unVal)
{
   atomic_add(pWord, unVal);
}

/**
  Decrements an atomic variable.

  @datatypes
  #posal_atomic_word_t

  @param[in] pWord  Pointer to the atomic variable.

  @return
  The new value of the atomic variable.

  @dependencies
  None.
*/
static inline int posal_atomic_decrement(posal_atomic_word_t pWord)
{
   return (int)atomic_dec_return(pWord);
}

/**
  Subtracts from an atomic variable.

  @datatypes
  #posal_atomic_word_t

  @param[in] pWord  Pointer to the atomic variable.
  @param[in] unVal  Number to subtract from the atomic variable.

  @return
  The new value of the atomic variable.

  @dependencies
  None. @newpage
*/
static inline void posal_atomic_subtract(posal_atomic_word_t pWord, uint32_t unVal)
{
   atomic_sub(pWord, unVal);
}

/**
  Bitwise ORs an atomic variable.

  @datatypes
  #posal_atomic_word_t

  @param[in] word_ptr  Pointer to the atomic variable.
  @param[in] val       OR bitfield.

  @return
  The new value of the atomic variable.

  @dependencies
  None.
*/
static inline void posal_atomic_or(posal_atomic_word_t word_ptr, uint32_t val)
{
   atomic_or(word_ptr, val);
}

/**
  Bitwise ANDs an atomic variable.

  @datatypes
  #posal_atomic_word_t

  @param[in] word_ptr  Pointer to the atomic variable.
  @param[in] val       AND bitfield.

  @return
  The new value of the atomic variable.

  @dependencies
  None. @newpage
*/
static inline void posal_atomic_and(posal_atomic_word_t word_ptr, uint32_t val)
{
   atomic_and(word_ptr, val);
}

/** @} */ /* end_addtogroup posal_atomic */

#ifdef __cplusplus
}
#endif //__cplusplus

#endif //POSAL_ATOMIC_H
