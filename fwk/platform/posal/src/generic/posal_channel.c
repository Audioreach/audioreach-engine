/**
 * \file posal_channel.c
 * \brief
 *  This file contains a utility to form a channel of a combination of up to
 *  32 signals/queues/timers. Client can wait on any combination thereof and
 *  be woken when any desired element is active
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */

#include "posal.h"
#include "posal_internal.h"
#include "posal_queue_i.h"
#include "posal_target_i.h"

/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/


/* =======================================================================
**                          Function Definitions
** ======================================================================= */

/****************************************************************************
** Channels
*****************************************************************************/
ar_result_t posal_channel_create(posal_channel_t *ppChannel, POSAL_HEAP_ID heap_id)
{
   posal_channel_internal_t *ch_ptr = NULL;

   ch_ptr = (posal_channel_internal_t *)posal_memory_malloc(sizeof(posal_channel_internal_t), heap_id);
   if (NULL == ch_ptr)
   {
      *ppChannel = NULL;
      return AR_ENOMEMORY;
   }

   ch_ptr->unBitsUsedMask = 0;
   posal_signal_create_target_inline(&ch_ptr->anysig);
   //AR_MSG(DBG_MED_PRIO, "posal_channel_create: ch_ptr=0x%p, signal=0x%p", ch_ptr, ch_ptr->anysig);

   *ppChannel = (posal_channel_t)ch_ptr;

   return AR_EOK;
}

void posal_channel_destroy(posal_channel_t *pp_chan)
{
   if (!pp_chan)
   {
      return;
   }

   posal_channel_internal_t *ch_ptr = (posal_channel_internal_t *)*pp_chan;
   if (!ch_ptr)
   {
      return;
   }

   posal_signal_destroy_target_inline(&ch_ptr->anysig);
   ch_ptr->unBitsUsedMask = 0;
   posal_memory_free(ch_ptr);
   *pp_chan = NULL;
}

static ar_result_t posal_channel_check_bit_mask(posal_channel_t pChannel, uint32_t *pUnBitMask)
{
   uint32_t unBitMask = *pUnBitMask;
   posal_channel_internal_t *ch_ptr = (posal_channel_internal_t *) pChannel;

   // check if all available bits were taken
   if (POSAL_CHANNEL_ALL_BITS == ch_ptr->unBitsUsedMask)
   {
      return AR_ENEEDMORE;
   }

   // check if a single bit is requested, i.e., unBitMask must be power of 2
   // check if the requested bit is available on channel
   if ((0 != unBitMask) && ((0 != (unBitMask & (unBitMask - 1))) || (0 != (unBitMask & ch_ptr->unBitsUsedMask))))
   {
      AR_MSG(DBG_ERROR_PRIO, "Incorrect BitMask of queue/signal!");
      return AR_EBADPARAM;
   }

   if (0 == unBitMask)
   { // user do not care about the position of channel bit,
      // obtain the available bit from LSB, pChannel->unBitsUsedMask is the
      // bookkeeper of available bits, 1-used, 0-available
      // unBitMask = 1;
      // while ( pChannel->unBitsUsedMask & unBitMask) unBitMask <<= 1;
      unBitMask = 1 << (NUM_MASK_BITS - s32_cl1_s32(ch_ptr->unBitsUsedMask));
   }

   *pUnBitMask = unBitMask;

   return AR_EOK;
}

ar_result_t posal_channel_addq(posal_channel_t pChannel, posal_queue_t* pQ, uint32_t unBitMask)
{
   ar_result_t result;
   posal_channel_internal_t *ch_ptr          = (posal_channel_internal_t *)pChannel;
   posal_queue_internal_t   *q_ptr           = (posal_queue_internal_t *)pQ;
   bool_t                    is_q_signal_set = FALSE;

   if (AR_DID_FAIL(result = posal_channel_check_bit_mask(pChannel, &unBitMask)))
   {
      AR_MSG(DBG_ERROR_PRIO, "Cannot add queue!");
      return result;
   }

   // clear any residual signal
   posal_signal_clear_with_bitmask_target_inline(&ch_ptr->anysig, unBitMask);

   posal_queue_mutex_lock(q_ptr);

   // if there is any existing associated channel then first remove it
   if (q_ptr->channel_ptr)
   {
      posal_channel_internal_t *old_ch_ptr = (posal_channel_internal_t *)q_ptr->channel_ptr;

      AR_MSG(DBG_LOW_PRIO, "Removing old channel from the queue");

      // check if signal on old channel is set or not.
      is_q_signal_set = posal_channel_poll_inline(old_ch_ptr, q_ptr->channel_bit);

      // clear the signal from the old channel if set
      posal_signal_clear_with_bitmask_target_inline(&old_ch_ptr->anysig, q_ptr->channel_bit);

      // release the channel bit
      old_ch_ptr->unBitsUsedMask ^= q_ptr->channel_bit;

      q_ptr->channel_ptr = NULL;
   }

   q_ptr->channel_ptr = pChannel;
   q_ptr->channel_bit = unBitMask;

#ifdef DEBUG_POSAL_CHANNEL
   AR_MSG(DBG_LOW_PRIO, "ADDQ: Q=0x%x Channelptr=0x%x Bitfield=0x%x", pQ, ch_ptr, pQ->myChannelBit);
#endif // DEBUG_POSAL_CHANNEL

   // bookkeeping available channel bits: 1-used, 0-available
   ch_ptr->unBitsUsedMask |= unBitMask;

   // if signal was set on old channel then set it on new channel now
   if (is_q_signal_set)
   {
      posal_signal_set_target_inline(&ch_ptr->anysig, unBitMask);
   }

   posal_queue_mutex_unlock(q_ptr);

   return AR_EOK;
}

ar_result_t posal_channel_add_signal(posal_channel_t pChannel, posal_signal_t pSignal, uint32_t unBitMask)
{
   ar_result_t               result;
   posal_channel_internal_t *ch_ptr        = (posal_channel_internal_t *)pChannel;
   posal_signal_internal_t  *sig_ptr       = (posal_signal_internal_t *)pSignal;
   bool_t                    is_signal_set = FALSE;

   if (AR_DID_FAIL(result = posal_channel_check_bit_mask(pChannel, &unBitMask)))
   {
      AR_MSG(DBG_ERROR_PRIO, "Cannot add signal!");
      return result;
   }

   // clear any residual signal
   posal_signal_clear_with_bitmask_target_inline(&ch_ptr->anysig, unBitMask);

   // if there is any existing associated channel then first remove it
   if (sig_ptr->pChannel)
   {
      posal_channel_internal_t *old_ch_ptr = sig_ptr->pChannel;

      AR_MSG(DBG_LOW_PRIO, "Removing old channel from the signal");

      // check if signal on old channel is set or not.
      is_signal_set = posal_channel_poll_inline(old_ch_ptr, sig_ptr->unMyChannelBit);

      // clear the signal from the old channel if set
      posal_signal_clear_with_bitmask_target_inline(&old_ch_ptr->anysig, sig_ptr->unMyChannelBit);

      // release the channel bit
      old_ch_ptr->unBitsUsedMask ^= sig_ptr->unMyChannelBit;

      sig_ptr->pChannel = NULL;
   }

   sig_ptr->pChannel       = pChannel;
   sig_ptr->unMyChannelBit = unBitMask;
#ifdef DEBUG_POSAL_CHANNEL
   AR_MSG(DBG_LOW_PRIO,
          "ADDSIG: Channelptr=0x%x Signalptr=0x%x ChannelBit=%d",
          ch_ptr,
          sig_ptr,
          sig_ptr->unMyChannelBit);
#endif // DEBUG_POSAL_CHANNEL

   // bookkeeping available channel bits: 1-used, 0-available
   ch_ptr->unBitsUsedMask |= unBitMask;

   // if signal was set on old channel then set it on new channel now
   if (is_signal_set)
   {
      posal_signal_set_target_inline(&ch_ptr->anysig, unBitMask);
   }

   return AR_EOK;
}
