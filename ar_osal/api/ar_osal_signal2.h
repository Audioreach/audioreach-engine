#ifndef AR_OSAL_SIGNAL2_H
#define AR_OSAL_SIGNAL2_H

/**
 * \file ar_osal_signal2.h
 * \brief
 *     Defines public APIs for signal2 used by DSP.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/** @weakgroup weakf_osal_signal2_intro
This section describes the following signal2 functions:
- ar_osal_signal2_init()
- ar_osal_signal2_deinit()
- ar_osal_signal2_get_size()
- ar_osal_signal2_create()
- ar_osal_signal2_destroy()
- ar_osal_signal2_set()
- ar_osal_signal2_get()
- ar_osal_signal2_clear()
- ar_osal_signal2_wait_any()
- ar_osal_signal2_wait_all()
*/

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/
/*=====================================================================
 Typedefs
 ======================================================================*/
#include "ar_osal_types.h"

/** @addtogroup signals_types
@{ */
/** ar_osal_signal2 type
 */
typedef void *ar_osal_signal2_t;
/** @} */ /* end_addtogroup signals_types */

/*=====================================================================
 Functions
======================================================================*/

/*======================================================================*/
/**@ingroup func_osal_signal2_init
  Initializes a signal2 object.
  Signal returns the initialized object.
  The signal object is initially cleared.
  signal object memory is allocated by the client.

  @datatypes
  #ar_osal_signal2_t

  @param[in] signal2: Pointer to the initialized object.

  @return
  0 -- Success
  Nonzero -- Failure


  @dependencies

 */
/* ======================================================================*/
int32_t ar_osal_signal2_init(ar_osal_signal2_t signal2);

/*======================================================================*/
/**@ingroup func_osal_signal2_deinit
  De-initializes signal2 object.

  @datatypes
  ar_osal_signal2_t

  @param[in] signal2: Pointer to the initialized object.

  @return
  0 -- Success
  Nonzero -- Failure


  @dependencies
 */

/* ======================================================================*/
int32_t ar_osal_signal2_deinit(ar_osal_signal2_t signal2);

/*======================================================================*/
/**@ingroup func_osal_signal2_get_size
   Get signal object size for object memory allocation by client.

  @datatypes
  ar_osal_signal2_t

  @return
  Size of signal object


  @dependencies
 */

/* ======================================================================*/
size_t ar_osal_signal2_get_size(void);

/*======================================================================*/
/**@ingroup func_osal_signal2_create
   Signal object memory is allocated and initialized by this API.
   osal_signal2_create() or osal_signal2_init() would be used, but not both.

   @datatypes
   ar_osal_signal2_t

   @param[Out] signal2: Pointer to the initialized object.

   @return
   0 -- Success
   Nonzero -- Failure


   @dependencies

 */
/* ======================================================================*/
int32_t ar_osal_signal2_create(ar_osal_signal2_t *signal2);


/*======================================================================*/
/**@ingroup func_osal_signal2_destroy
  Destroys the specified signal object.

  @note1cont Signal objects must not be destroyed while they are still in use. If this
  happens the behavior is undefined.
  @note1cont In general, application code should "destroy" a signal2 object prior to
             deallocating it; calling osal_signal2_destroy() before deallocating it ensures
             that all osal_signal2_set() calls have completed.

  @datatypes
  #ar_osal_signal2_t

  @param[in] signal2: Pointer to the signal object to destroy.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  None.
 */
/* ======================================================================*/
int32_t ar_osal_signal2_destroy(ar_osal_signal2_t signal2);

/*======================================================================*/
/**@ingroup func_osal_signal2_wait_any
  Suspends the current thread until any of the specified signals are set.

  Signals are represented as bits 0-31 in the 32-bit mask value. A mask bit value of 1 indicates
  that a signal is to be waited on, and 0 that it is not to be waited on.

  If a thread is waiting on a signal object for any of the specified set of signals to be set,
  and one or more of those signals is set in the signal object, then the thread is awakened.

  At most one thread can wait on a signal object at any given time.

  Signals must be explicitly cleared by a thread when it is awakened
  the wait operations do not automatically clear them.

  @datatypes
  #ar_osal_signal2_t

  @param[in] signal2:             Pointer to the signal object to wait on.
  @param[in] signal2_mask:        Mask value identifying the individual signals
                                       in the signal object to be waited on.

  @return
  A 32 bit word with current signals.

  @dependencies
  None.
*/
/* ======================================================================*/
uint32_t ar_osal_signal2_wait_any(ar_osal_signal2_t signal2, uint32_t signal2_mask);

/*======================================================================*/
/**@ingroup func_osal_signal2_wait_all
  Suspends the current thread until all of the specified signals are set.

  Signals are represented as bits 0-31 in the 32-bit mask value. A mask bit value of 1 indicates
  that a signal is to be waited on, and 0 that it is not to be waited on.

  If a thread is waiting on a signal object for all of the specified set of signals to be set,
  and all of those signals are set in the signal object, then the thread is awakened.

  At most one thread can wait on a signal object at any given time.

  Signals must be explicitly cleared by a thread when it is awakened
   the wait operations do not automatically clear them.

  @datatypes
  #ar_osal_signal2_t

  @param[in] signal2:             Pointer to the signal object to wait on.
  @param[in] signal2_mask:        Mask value which identifies the individual signals
                                       in the signal object to be waited on.

  @return
  A 32 bit word with current signals.

  @dependencies
  None.
*/
/* ======================================================================*/
uint32_t ar_osal_signal2_wait_all(ar_osal_signal2_t signal2, uint32_t signal2_mask);

/*======================================================================*/
/**@ingroup func_osal_signal2_set
  Sets signals in the specified signal object.

  Signals are represented as bits 0-31 in the 32-bit mask value. A mask bit value of 1 indicates
  that a signal is to be set, and 0 that it is not to be set.

  @datatypes
  ar_osal_signal2_t

  @param[in] signal2:  Pointer to the signal object to be modified.
  @param[in] signal2_mask:  Mask value identifying the individual
               signals to be set in the signal object.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  None.
*/
/* ======================================================================*/
int32_t ar_osal_signal2_set(ar_osal_signal2_t signal2, uint32_t signal2_mask);

/*======================================================================*/
/**@ingroup func_osal_signal2_get
   Gets a signal from a signal object.

   Returns the current signal values of the specified signal object.

  @datatypes
  ar_osal_signal2_t

  @param[in] signal2: Pointer to the signal object to access.

  @return
  A 32 bit word with current signals

  @dependencies
  None.
*/
/* ======================================================================*/
uint32_t ar_osal_signal2_get(ar_osal_signal2_t signal2);

/*======================================================================*/
/**@ingroup func_osal_signal2_clear
  Clear signals in the specified signal object.

  Signals are represented as bits 0-31 in the 32-bit signal2_mask value.
  A signal2_mask bit value of 1 indicates that a signal is to be cleared,
  and 0 that it is not to be cleared.

  Signals must be explicitly cleared by a thread when it is awakened
   the wait operations do not automatically clear them.

  @datatypes
  ar_osal_signal2_t

  @param[in] signal:   Pointer to the signal object to modify.
  @param[in] signal2_mask:     signal2_mask value identifying the
                          individual signals to be cleared in the signal object.

  @return
  0 -- Success
  Nonzero -- Failure

  @dependencies
  None.
 */
/* ======================================================================*/
int32_t ar_osal_signal2_clear(ar_osal_signal2_t signal2, uint32_t signal2_mask);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* AR_OSAL_SIGNAL2_H */
