/**
 * \file posal_linux_signal.h
 *
 * \brief
 *  	  This file contains a utilities to work with linux signal
 *
 * \copyright
 *      Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *      SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef POSAL_LINUX_SIGNAL_H
#define POSAL_LINUX_SIGNAL_H

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include <pthread.h>
#include <errno.h>

/* ----------------------------------------------------------------------------
 * Global Declarations/Definitions
 * ------------------------------------------------------------------------- */
typedef struct {
    pthread_cond_t created_signal;
    uint32_t signalled;
    pthread_mutex_t mutex_handle;
}posal_linux_signal_internal_t;

typedef void *posal_linux_signal_t;

/* -------------------------------------------------------------------------
 * Function Declarations
 * ------------------------------------------------------------------------- */
/**
  Creates and initializes a signal.

  @datatypes
  posal_linux_signal_t

  @param[in] p_signal  Double pointer to the signal.

  @param[in] heap_id   Heap ID used for malloc.

  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  None.
*/
ar_result_t posal_linux_signal_create(posal_linux_signal_t *p_signal);

/**
  Destroys the signal created using posal_linux_signal_create().

  @datatypes
  posal_linux_signal_t

  @param[in] p_signal  Double pointer to the signal.

  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  None.
*/
ar_result_t posal_linux_signal_destroy(posal_linux_signal_t *p_signal);

/**
  Reset the signal(s) specified by the signal bit mask.

  @datatypes
  posal_linux_signal_t

  @param[in] p_signal         Double pointer to the signal.

  @param[in] signal_bitmask   Signal bit mask.

  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  None.
*/
ar_result_t posal_linux_signal_clear(posal_linux_signal_t *p_signal, uint32_t signal_bitmask);

/**
  Wait for the signal(s) specified by the signal bit mask.

  @datatypes
  posal_linux_signal_t

  @param[in] p_signal         Double pointer to the signal.

  @param[in] signal_bitmask   Signal bit mask.

  @return
  Signal mask indicating the signals that are satisfied the wait request

  @dependencies
  None.
*/
uint32_t posal_linux_signal_wait(posal_linux_signal_t *p_signal, uint32_t signal_bitmask);


/**
  Set the signal(s) specified by the signal bit mask.

  @datatypes
  posal_linux_signal_t

  @param[in] p_signal         Double pointer to the signal.

  @param[in] signal_bitmask   Signal bit mask.

  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  None.
*/
ar_result_t posal_linux_signal_set(posal_linux_signal_t *p_signal, uint32_t signal_bitmask);

/**
  Get the currently activated signal(s)

  @datatypes
  posal_linux_signal_t

  @param[in] p_signal         Double pointer to the signal.

  @param[in] signal_bitmask   Signal bit mask.

  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  None.
*/
ar_result_t posal_linux_signal_get(posal_linux_signal_t *p_signal);

#endif //#ifndef POSAL_LINUX_SIGNAL_H