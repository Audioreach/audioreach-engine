/**
 * \file posal_interrupt.h
 * \brief
 *  	 This file contains utilities for registering with interrupts.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_INTERRUPT_H
#define POSAL_INTERRUPT_H

#include "posal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus


/** @addtogroup posal_interrupt
@{ */

/*forward declaration*/
typedef void *posal_interrupt_ist_t;

/**
  Registers an interrupt. The client must allocate the memory for the
  posal_interrupt_ist_t structure.

  @datatypes
  posal_interrupt_ist_t

  @param[in] ist_ptr          Pointer to the IST.
  @param[in] intr_id          Interrupt number to register.
  @param[in] callback_ptr     Pointer to the callback function when an
                              interrupt occurs.
  @param[in] arg_ptr          Pointer to the arguments sent by the client.
  @param[in] thread_name      Pointer to the IST thread name.
  @param[in] stack_size       Size of the IST stack.

  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  None. @newpage
*/
ar_result_t posal_interrupt_register(posal_interrupt_ist_t *ist_ptr, uint16_t intr_id,
                              void (*callback_ptr)(void *),void *arg_ptr,char_t *thread_name, uint32_t stack_size, POSAL_HEAP_ID heap_id);

/**
  Deregisters an interrupt.

  @datatypes
  posal_interrupt_ist_t

  @param[in] ist_ptr  Pointer to the IST.

  @return
  @par
  Nonzero -- Failure
  @dependencies
  Before calling this function, the interrupt object must be registered.
*/
ar_result_t posal_interrupt_deregister(posal_interrupt_ist_t *ist_ptr);

/** @} */ /* end_addtogroup posal_interrupt */

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_INTERRUPT_H
