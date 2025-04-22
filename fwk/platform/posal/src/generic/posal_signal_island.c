/**
 * \file posal_signal_island.c
 * \brief
 *  	 This file contains signals utilities.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal.h"
#include "posal_internal.h"
#include "posal_target_i.h"

/* =======================================================================
**                          Function Definitions
** ======================================================================= */

/****************************************************************************
** Signals
*****************************************************************************/
void posal_signal_send(posal_signal_t p_signal)
{
   posal_signal_internal_t *sig_ptr = (posal_signal_internal_t *)p_signal;
   (void) posal_signal_set_target_inline(&sig_ptr->pChannel->anysig, sig_ptr->unMyChannelBit);
}

void posal_signal_clear(posal_signal_t p_signal)
{
   posal_signal_clear_inline(p_signal);
}

bool_t posal_signal_is_set(posal_signal_t p_signal)
{
   return posal_signal_is_set_inline(p_signal);
}

uint32_t posal_signal_get_channel_bit(posal_signal_t p_signal)
{
   return posal_signal_get_channel_bit_inline(p_signal);
}
