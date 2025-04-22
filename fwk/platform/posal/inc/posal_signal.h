/**
 * \file posal_signal.h
 * \brief
 *  	 This file contains signal utilities.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_SIGNAL_H
#define POSAL_SIGNAL_H

#include "posal_types.h"
#include "posal_memory.h"
#include "ar_error_codes.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** @addtogroup posal_signal
@{ */

/****************************************************************************
** Signals
*****************************************************************************/
typedef void *posal_signal_t;
typedef void *posal_channel_t;

/**
  Creates a signal.

  @datatypes
  posal_signal_t

  @param[out] ppSignal  Double pointer to the signal.

  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  None.
*/
ar_result_t posal_signal_create(posal_signal_t* pp_sigobj, POSAL_HEAP_ID heap_id);

/**
  Destroys a signal

  @datatypes
  posal_signal_t

  @param[in] pSignal  Pointer to the signal.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
  @newpage
*/
void posal_signal_destroy(posal_signal_t *pp_sigobj);

/**
  Queries a signal for its channel.

  @datatypes
  posal_signal_t

  @param[in] pSignal  Pointer to the signal.

  @return
  A handle to the channel containing the signal.

  @dependencies
  Before calling this function, the signal group must be created and
  initialized.
*/
posal_channel_t posal_signal_get_channel(posal_signal_t p_sigobj);

/**
  Queries a signal for its channel bitfield.

  @datatypes
  posal_signal_t

  @param[in] pSignal  Pointer to the signal to query.

  @detdesc
  Use this function to find the values to be ORed together to form an enable
  bitfield for a combination of signals in a channel.

  @return
  A 32-bit value with a single bit=1.

  @dependencies
  Before calling this function, the signal group must be created and
  initialized. @newpage
*/
uint32_t posal_signal_get_channel_bit(posal_signal_t p_sigobj);

/**
  Sends a signal.

  @datatypes
  posal_signal_t

  @param[in] pSignal  Pointer to the signal.

  @return
  None.

  @dependencies
  Before calling this function, the signal group must be created and
  initialized.
*/
void posal_signal_send(posal_signal_t p_sigobj);

/**
  Clears a signal that is active in a channel.

  @datatypes
  posal_signal_t

  @param[in] pSignal   Pointer to the signal.

  @return
  None.

  @dependencies
  Before calling this function, the signal group must be created and
  initialized.
*/
void posal_signal_clear(posal_signal_t p_sigobj);

/**
  returns true if the signal is set

  @datatypes
  posal_signal_t

  @param[in] pSignal   Pointer to the signal.

  @return
  None.

  @dependencies
  Before calling this function, the signal group must be created and
  initialized.
*/
bool_t posal_signal_is_set(posal_signal_t p_sigobj);

/** @} */ /* end_addtogroup posal_signal */

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_SIGNAL_H
