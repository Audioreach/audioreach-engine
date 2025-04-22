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
#include "posal_linux_signal.h"

typedef struct {
   posal_linux_signal_t    anysig;
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

    uint32_t status = posal_linux_signal_wait(&ch_ptr->anysig, unEnableBitfield);
    return (status & unEnableBitfield);
}

static inline uint32_t posal_channel_poll_inline(posal_channel_t pChannel, uint32_t unEnableBitfield)
{
   posal_channel_internal_t *ch_ptr = (posal_channel_internal_t *) pChannel;

   return (unEnableBitfield & posal_linux_signal_get(&ch_ptr->anysig));
}

/*============== posal_island inline functions ==============*/

static inline bool_t posal_island_get_island_status_inline(void)
{
/*For island enabled targets invoke qurt call else return normal mode value. */
   return FALSE; //normal mode
}

/*============== posal_mutex inline functions ==============*/

static inline void posal_mutex_unlock_inline(posal_mutex_t posal_mutex)
{
    pthread_mutex_unlock((pthread_mutex_t*) posal_mutex);
}

static inline void posal_mutex_lock_inline(posal_mutex_t posal_mutex)
{
    pthread_mutex_lock((pthread_mutex_t*) posal_mutex);
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
    posal_linux_signal_clear(&sig_ptr->pChannel->anysig, sig_ptr->unMyChannelBit);
}

static inline bool_t posal_signal_is_set_inline(posal_signal_t p_signal)
{
    posal_signal_internal_t *sig_ptr = (posal_signal_internal_t *)p_signal;
    return ((posal_linux_signal_get(&sig_ptr->pChannel->anysig) & sig_ptr->unMyChannelBit) > 0);
}

#endif // POSAL_INTERNAL_INLINE_H
