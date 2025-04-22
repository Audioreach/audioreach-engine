/**
 * \file posal_channel.h
 * \brief
 *  	 This file contains a utility to form a channel of a combination
 *  	of up to 32 signals and queues. The client can wait on any combination of
 *  	signals and queues, be awakened when any desired element is active.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_CHANNEL_H
#define POSAL_CHANNEL_H

#include "posal_types.h"
#include "posal_memory.h"
#include "posal_queue.h"

// The numbber of bits available for masks
#define NUM_MASK_BITS 31

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** @addtogroup posal_channel
@{ */

/* -----------------------------------------------------------------------
** Global definitions/forward declarations
** ----------------------------------------------------------------------- */
typedef void *posal_channel_t;
typedef void *posal_signal_t;

/** Mask of all channel bits. */
#define POSAL_CHANNEL_ALL_BITS       0xFFFFFFFFL

/** Assign the next available bit in channel mask */
#define POSAL_CHANNEL_MASK_DONT_CARE 0x0UL

/**
  Creates and initializes a channel.

  @datatypes
  posal_channel_t

  @param[in] pChannel  Double pointer to the channel.

  @param[in] heap_id   Heap ID used for malloc.

  @detdesc
  There can be only one receiver thread in this channel implementation. A
  channel is a combination of up to 32 signals and queues.
  @par
  Any mask of the associated elements can be set up to block until one of the
  nonmasked queues receives an element. Typically, the receiver blocks on a
  channel, checks which queues have data, and pops the data off at least one
  queue.
  @par
  @note1hang Channel operations are not thread-protected. Only the
             owner-receiver thread is to touch them. @newpage
  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  None.
*/
ar_result_t posal_channel_create(posal_channel_t* pChannel, POSAL_HEAP_ID heap_id);

/**
  Destroys a channel. This function must be called to clean up the
  resources from a corresponding call to posal_channel_create().

  @datatypes
  posal_channel_t

  @param[in] pChannel  Pointer to the channel.

  @return
  None.

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
void posal_channel_destroy(posal_channel_t *pp_chan);

/**
  Adds a queue to a channel. Each queue can belong to only one channel.

  @datatypes
  posal_channel_t \n
  posal_queue_t

  @param[in] pChannel   Pointer to the channel.
  @param[in] pQ         Pointer to the queue.
  @param[in] unBitMask  Indicates the bit used to add the queue.

  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  Before calling this function, the object must be created and initialized.
  @newpage
*/
ar_result_t posal_channel_addq(posal_channel_t pChannel, posal_queue_t* pQ, uint32_t unBitMask);

/**
  Adds a signal to a channel.

  @datatypes
  posal_channel_t \n
  posal_signal_t

  @param[in] pChannel   Pointer to the channel.
  @param[in] pSignal    Pointer to the signal.
  @param[in] unBitMask  Indicates the bit used to add the signal.

  @return
  0 -- Success
  @par
  Nonzero -- Failure

  @dependencies
  Before calling this function, the object must be created and initialized.
*/
ar_result_t posal_channel_add_signal(posal_channel_t pChannel, posal_signal_t pSignal, uint32_t unBitMask);

/**
  Waits for a combination of queues and signals in a channel to
  receive an item.

  @note1hang Only one thread can wait on a channel at a time.

  @datatypes
  posal_channel_t

  @param[in] pChannel          Pointer to the channel.
  @param[in] unEnableBitfield  Mask that indicates the queues and signals.

  @return
  A bitfield indicating which queues have items.

  @dependencies
  Before calling this function, the object must be created and initialized. @newpage
*/
uint32_t posal_channel_wait(posal_channel_t pChannel, uint32_t unEnableBitfield);


/**
  Checks which channel queues and signals received active triggers.

  @datatypes
  posal_channel_t

  @param[in] pChannel          Pointer to the channel.
  @param[in] unEnableBitfield  Mask that indicates the queues and signals.

  @detdesc
  A bit setting of 1 indicates that there is something to receive. This
  operation is like a poll for a specific set of bits in cases where
  blocking from posal_channel_wait() is not wanted.

  @return
  A 32-bit field indicating which of the queues and signals received
  active triggers.

  @dependencies
  Before calling this function, the object must be created and initialized.
  @newpage
*/
uint32_t posal_channel_poll(posal_channel_t pChannel, uint32_t unEnableBitfield);


/** @} */ /* end_addtogroup posal_channel */

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_CHANNEL_H
