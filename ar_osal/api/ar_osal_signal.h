#ifndef AR_OSAL_SIGNAL_H
#define AR_OSAL_SIGNAL_H

/**
 * \file ar_osal_signal.h
 * \brief
 *     Defines public APIs for signal.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/** @weakgroup weakf_osal_signal_intro
This section describes the following signal functions:
- ar_osal_signal_init()
- ar_osal_signal_deinit()
- ar_osal_signal_get_size()
- ar_osal_signal_create()
- ar_osal_signal_destroy()
- ar_osal_signal_set()
- ar_osal_signal_clear()
- ar_osal_signal_wait()
- ar_osal_signal_timedwait()
*/

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/
/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "ar_osal_types.h"

/** @addtogroup ar osal signal
@{ */

/* -----------------------------------------------------------------------
** Global definitions/forward declarations
** ----------------------------------------------------------------------- */

/****************************************************************************
** Signals
*****************************************************************************/

/** Signal to be triggered by events, or used to trigger events.

*/
typedef void *ar_osal_signal_t;

/**
  initialize a signal object. The client must allocate the memory for the signal and
  pass the pointer to this function.
  The signal object is initially cleared.

  @datatypes
  ar_osal_signal_t

  @param[in] signal: Pointer to the signal object.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  None. @newpage
*/
int32_t ar_osal_signal_init(ar_osal_signal_t signal);

/**
  De-initialize a signal object.

  @datatypes
  ar_osal_signal_t

  @param[in] signal: Pointer to the signal.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  None. @newpage
*/
int32_t ar_osal_signal_deinit(ar_osal_signal_t signal);


/**
  Get signal object size for client to allocate its memory.

  @datatypes
  ar_osal_signal_t

  @return
  Size of signal object

  @dependencies
  None. @newpage
*/
size_t ar_osal_signal_get_size(void);
/**
  Create a signal and initialize. Signal object memory is allocated by this API and
  returned to the client.
  The signal object is initially cleared.
  ar_osal_signal_create() or ar_osal_signal_init() would be used, but not both.

  @datatypes
  ar_osal_signal_t

  @param[out] signal: Pointer to the signal.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  None. @newpage
*/
int32_t ar_osal_signal_create(ar_osal_signal_t *signal);

/**
  Destroy a signal. Frees the memory allocated by ar_osal_signal_create
  API. Signal objects must not be destroyed while its still in use.

  @datatypes
  ar_osal_signal_t

  @param[in] signal: Pointer to the signal.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  Before calling this function, the signal group must be created and
  initialized. @newpage
*/
int32_t ar_osal_signal_destroy(ar_osal_signal_t signal);

/**
  Sets signal in the specified signal object.

  @datatypes
  ar_osal_signal_t

  @param[in] signal: Pointer to the signal.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  Before calling this function, the signal group must be created and
  initialized.
*/
int32_t ar_osal_signal_set(ar_osal_signal_t signal);

/**
   Clear signal in the specified signal object.
   Signal must be explicitly cleared by a thread when it is awakened,
   the wait operations do not automatically clear it.

   Clearing a signal that is already cleared has no effect.

  @datatypes
  ar_osal_signal_t

  @param[in] signal: Pointer to the signal.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  Before calling this function, the signal group must be created and
  initialized.
*/
int32_t ar_osal_signal_clear(ar_osal_signal_t signal);


/**
  Wait on a signal, wait infinitely for the signal to be set.

  If a thread is waiting on a signal object for signal to be set,
  and the signal object is set, then the thread is awakened.

  The signal can be cleared once the signal is set.

  The signal must be manually cleared using the ar_osal_signal_clear
  function. Any number of waiting threads, or threads that subsequently
  begin wait operations for the specified signal object, will be released
  while the signal object's state is signaled.

  If a signal is set before a thread is waiting, then it will be returned
  immediately and the signal must be cleared to avoid unintended behavior

  @datatypes
  ar_osal_signal_t

  @param[in] signal: Pointer to the signal.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  Before calling this function, the signal group must be created and
  initialized.
*/
int32_t ar_osal_signal_wait(ar_osal_signal_t signal);

/**
   Wait on signal till timeout happens.

   The signal can be cleared once the signal is set or timeout occurs.

   The signal must be manually cleared using the ar_osal_signal_clear
   function. Any number of waiting threads, or threads that subsequently
   begin wait operations for the specified signal object, will be released
   while the signal object's state is signaled.

   If a signal is set before a thread is waiting, then it will be returned
   immediately and the signal must be cleared to avoid unintended behavior

  @datatypes
  ar_osal_signal_t

  @param[in] signal: Pointer to the signal.
  @param[in] timeout_in_nsec: Time to wait in nano seconds

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  Before calling this function, the signal group must be created and
  initialized.
*/
int32_t ar_osal_signal_timedwait(ar_osal_signal_t signal, int64_t timeout_in_nsec);

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif // #ifndef AR_OSAL_SIGNAL_H

