/**
 * \file posal_queue_i.h
 * \brief
 *     This file contains the definitins internal to the posal queue structure. These types have to be kept internal
 *     to the implementation of the queue so that we can take advantage of optimizations by eliminating the create
 * function where possible
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _POSAL_QUEUE_I_H
#define _POSAL_QUEUE_I_H

#include "posal_types.h"
#include "posal_std.h"
#include "posal_channel.h"
#include "posal_mutex.h"
#include "posal_signal.h"
#include "posal_memory.h"
#include "posal_inline_mutex.h"
#include "posal_internal.h"

#ifndef ALIGN_8_BYTES
#define ALIGN_8_BYTES(a) ((a + 7) & (0xFFFFFFF8))
#endif

// #define QUEUE_DISABLE_NEEDED

typedef struct posal_queue_element_t
{
   void *q_payload_ptr;
   union
   {
      uint32_t opcode;
      void    *q_sec_ptr;
   };
} posal_queue_element_t;

typedef struct posal_queue_element_list_t posal_queue_element_list_t;

/* This is a base structure which defines a node in the queue.
 * posal_priority_queue_element_list_t is an extension of this base structure.
 */
struct posal_queue_element_list_t
{
   posal_queue_element_t       elem;
   posal_queue_element_list_t *next_ptr;
   posal_queue_element_list_t *prev_ptr;
};

typedef struct posal_priority_queue_element_list_t posal_priority_queue_element_list_t;

/* Extension of posal_queue_element_list_t.
 * All the elements of posal_queue_element_list_t should be at the beginning of this structure.
 * This to ensure that this structure can be type-casted to the posal_queue_element_list_t safely.
 */
struct posal_priority_queue_element_list_t
{
   posal_queue_element_t                elem;
   posal_priority_queue_element_list_t *next_ptr;
   posal_priority_queue_element_list_t *prev_ptr;

   uint32_t priority;
};

typedef struct posal_queue_internal_t
{
   posal_inline_mutex_t queue_mutex;
   /**< Mutex for thread-safe access to the queue. */

   posal_channel_t channel_ptr;
   /**< Pointer to the associated channel. */

   posal_queue_element_list_t *active_head_ptr;

   posal_queue_element_list_t *active_tail_ptr;

   POSAL_HEAP_ID heap_id;

   uint32_t channel_bit;
   /**< Channel bitfield of this queue. */

   int16_t active_nodes;
   /**< Number of nodes currently in use */

   int16_t num_nodes;
   /**< Number of queue nodes. */

   int16_t max_nodes;
   /**< Number of queue nodes. */

   uint16_t is_priority_queue : 1;
   /**< Flag to mark this queue as priority queue. */

   uint16_t disable_signaling : 1;
   /**< Specifies whether the signaling from the queue is disabled.*/

#ifdef QUEUE_DISABLE_NEEDED
   bool_t disable_flag;
   /**< Specifies whether the queue is disabled.
        @values
        - TRUE -- Disabled
        - FALSE -- Enabled @tablebulletend */
#endif

#if DEBUG_WITH_QUEUE_NAME
   char name[POSAL_DEFAULT_NAME_LEN];
/**< Name of the queue. */
#endif
} posal_queue_internal_t;

static inline posal_queue_element_list_t *posal_queue_create_node(posal_queue_internal_t *queue_ptr)
{
   if (!queue_ptr->is_priority_queue)
   {
      uint32_t q_handle = posal_queuepool_get_handle_from_heap_id(queue_ptr->heap_id);
      return (posal_queue_element_list_t *)posal_bufpool_get_node(q_handle);
   }
   else
   {
      return (posal_queue_element_list_t *)posal_memory_malloc(sizeof(posal_priority_queue_element_list_t),
                                                               queue_ptr->heap_id);
   }
}

static inline void posal_queue_release_node(posal_queue_internal_t *queue_ptr, posal_queue_element_list_t *node_ptr)
{
   if (!queue_ptr->is_priority_queue)
   {
      posal_bufpool_return_node(node_ptr);
   }
   else
   {
      posal_memory_free(node_ptr);
   }
}
#endif //_POSAL_QUEUE_I_H
