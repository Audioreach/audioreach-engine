/**
 * \file posal_target_i.h
 *
 * \brief
 *  	This file contains inline functions to support APIs for QuRT mutex/signal
 *    implementation.
 *
 * \copyright
 *      Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *      SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#ifndef POSAL_TARGET_I_H
#define POSAL_TARGET_I_H
/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "posal.h"
#include "posal_queue_i.h"
#include "posal_linux_signal.h"

/* ----------------------------------------------------------------------------
 * Global Declarations/Definitions
 * ------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
 * Function Declarations/Definitions
 * ------------------------------------------------------------------------- */
static inline void posal_signal_create_target_inline(posal_linux_signal_t *p_signal)
{
   posal_linux_signal_create(p_signal);
}

static inline void posal_signal_destroy_target_inline(posal_linux_signal_t *p_signal)
{
   posal_linux_signal_destroy(p_signal);
}

static inline void posal_signal_clear_with_bitmask_target_inline(posal_linux_signal_t *p_signal, uint32_t unBitMask)
{
   posal_linux_signal_clear(p_signal, unBitMask);
}

static inline void posal_signal_set_target_inline(posal_linux_signal_t *p_signal, uint32_t unBitMask)
{
   posal_linux_signal_set(p_signal, unBitMask);
}

static inline uint32_t posal_signal_get_target_inline(posal_linux_signal_t *p_signal)
{
   return (posal_linux_signal_get(p_signal));
}

static void posal_queue_mutex_lock(posal_queue_internal_t *queue_ptr)
{
   posal_mutex_lock_inline(queue_ptr->queue_mutex);
}

static void posal_queue_mutex_unlock(posal_queue_internal_t *queue_ptr)
{
   posal_mutex_unlock_inline(queue_ptr->queue_mutex);
}
#endif //#ifndef POSAL_TARGET_I_H