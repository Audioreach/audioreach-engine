/**
 * \file posal_signal.c
 * \brief
 *  	This file contains signals utilities.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal.h"
#include "posal_internal.h"

/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/

/* -----------------------------------------------------------------------
** Constant / Define Declarations
** ----------------------------------------------------------------------- */

/* =======================================================================
**                          Function Definitions
** ======================================================================= */

/****************************************************************************
** Signals
*****************************************************************************/
ar_result_t posal_signal_create(posal_signal_t* pp_signal, POSAL_HEAP_ID heap_id)
{
   posal_signal_t temp_sig_ptr = NULL;

   // allocate space
   if (NULL == (temp_sig_ptr = (posal_signal_t*) posal_memory_malloc(sizeof(posal_signal_internal_t), heap_id)))
   {
      *pp_signal = NULL;
      return AR_ENOMEMORY;
   }

   memset(temp_sig_ptr, 0, sizeof(posal_signal_internal_t));

   *pp_signal = temp_sig_ptr;

#ifdef DEBUG_POSAL_SIGNAL
   AR_MSG(DBG_LOW_PRIO, "SIGNAL CREATE: SIG=0x%x \n", *pp_signal);
#endif //DEBUG_POSAL_SIGNAL
   // return.
   return AR_EOK;
}

void posal_signal_destroy(posal_signal_t *pp_sig)
{
   posal_signal_internal_t *sig_ptr = (posal_signal_internal_t *)*pp_sig;

   //release the channel bit
   if (sig_ptr->pChannel) sig_ptr->pChannel->unBitsUsedMask ^= sig_ptr->unMyChannelBit;

   posal_memory_free(sig_ptr);
   *pp_sig = NULL;
}

posal_channel_t posal_signal_get_channel(posal_signal_t p_signal)
{
   return posal_signal_get_channel_inline(p_signal);
}
