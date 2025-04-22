/**
 *  \file posal_internal_inline.h
 * \brief
 *      Internal definitions. Helps optimize by making inline calls.
 *      Must not be used by shared libraries due to backward compatibility concerns.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_INTERNAL_INLINE_H
#define POSAL_INTERNAL_INLINE_H
/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "posal.h"
#include "qurt.h"
#include "qurt_signal2.h"
#include "qurt_pimutex.h"

typedef struct {

   qurt_signal2_t    anysig;
   /**< Any 32-bit signal channel. */

   uint32_t               unBitsUsedMask;
   /**< Mask bookkeeping for used bits.
        @values
         - 1 -- Used
         - 0 -- Available @tablebulletend @newpagetable */
} posal_channel_internal_t;

/** Signal to be triggered by events, or used to trigger events.
  The signal coalesces on a channel bit. The only way to receive a signal is
  through its associated channel.
 */
typedef struct {

   posal_channel_internal_t         *pChannel;
   /**< Pointer to the associated channel. */

   uint32_t                    unMyChannelBit;
   /**< Channel bitfield of this signal. @newpagetable */
} posal_signal_internal_t;

/*============== posal_channel inline functions ==============*/

static inline uint32_t posal_channel_wait_inline(posal_channel_t pChannel, uint32_t unEnableBitfield)
{
    posal_channel_internal_t *ch_ptr = (posal_channel_internal_t *) pChannel;

   // MSG_2(MSG_SSID_QDSP6, DBG_LOW_PRIO, "WAIT: Channelptr=0x%x, ENABLE=0x%x", pChannel, unEnableBitfield);
   uint32_t status = (uint32_t)qurt_signal2_wait_any(&ch_ptr->anysig, unEnableBitfield);
   // MSG_2(MSG_SSID_QDSP6, DBG_LOW_PRIO, "GOT: Channelptr=0x%x, STATUS=0x%x", pChannel,
   // qurt_signal2_get(&pChannel->anysig));
   return (status & unEnableBitfield);
}

static inline uint32_t posal_channel_poll_inline(posal_channel_t pChannel, uint32_t unEnableBitfield)
{
   posal_channel_internal_t *ch_ptr = (posal_channel_internal_t *) pChannel;

   return (unEnableBitfield & qurt_signal2_get(&ch_ptr->anysig));
}

/*============== posal_island inline functions ==============*/

static inline bool_t posal_island_get_island_status_inline(void)
{
/*For island enabled targets invoke qurt call else return normal mode value. */
#ifdef USES_AUDIO_IN_ISLAND
   return qurt_island_get_status();
#else
   return 0; //normal mode
#endif //USES_AUDIO_IN_ISLAND
}

/*============== posal_mutex inline functions ==============*/

static inline void posal_mutex_unlock_inline(posal_mutex_t posal_mutex)
{
   qurt_pimutex_unlock((qurt_mutex_t*) posal_mutex);
}

static inline void posal_mutex_lock_inline(posal_mutex_t posal_mutex)
{
   qurt_pimutex_lock((qurt_mutex_t*) posal_mutex);
}


/*============== posal_signal inline functions ==============*/

static inline posal_channel_t posal_signal_get_channel_inline(posal_signal_t p_signal)
{
   posal_signal_internal_t *sig_ptr = (posal_signal_internal_t *)p_signal;
   return (posal_channel_t *)sig_ptr->pChannel;
}

static inline uint32_t posal_signal_get_channel_bit_inline(posal_signal_t p_signal)
{
   posal_signal_internal_t *sig_ptr = (posal_signal_internal_t *)p_signal;
   return sig_ptr->unMyChannelBit;
}

static inline void posal_signal_clear_inline(posal_signal_t p_signal)
{
   posal_signal_internal_t *sig_ptr = (posal_signal_internal_t *)p_signal;
   (void) qurt_signal2_clear(&sig_ptr->pChannel->anysig, sig_ptr->unMyChannelBit);
}

static inline bool_t posal_signal_is_set_inline(posal_signal_t p_signal)
{
   posal_signal_internal_t *sig_ptr = (posal_signal_internal_t *)p_signal;
   return ((qurt_signal2_get(&sig_ptr->pChannel->anysig) & sig_ptr->unMyChannelBit) > 0);
}

#endif // POSAL_INTERNAL_INLINE_H
