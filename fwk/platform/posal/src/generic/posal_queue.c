/**
 * \file posal_queue.c
 * \brief
 *     This file contains an efficient queue of 8-uint8_t nodes.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "posal.h"
#include "posal_queue_i.h"
#include "posal_internal.h"
#include "posal_globalstate.h"
#include "posal_target_i.h"

// #define DEBUG_POSAL_QUEUE_POOL
// #define DEBUG_QUEUE_LOW
// #define QUEUE_NODE_UTILIZATION_DEBUG 1

/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/

/* -----------------------------------------------------------------------
** Constant / Define Declarations
** ----------------------------------------------------------------------- */

/* =======================================================================
**                          Function Definitions
** ======================================================================= */
static void        posal_queue_free_all_nodes(posal_queue_internal_t *);
static ar_result_t posal_queue_create_prealloc_nodes(posal_queue_t *q_ptr, posal_queue_init_attr_t *attr_ptr);

/****************************************************************************
** Queues
*****************************************************************************/

ar_result_t posal_queue_pool_setup(POSAL_HEAP_ID heap_id, uint32_t num_arrays, uint16_t nodes_per_arr)
{
   uint32_t handle = posal_bufpool_pool_create(sizeof(posal_queue_element_list_t),
                                               heap_id,
                                               num_arrays,
                                               FOUR_BYTE_ALIGN,
                                               nodes_per_arr);
   if (POSAL_BUFPOOL_INVALID_HANDLE == handle)
   {
      AR_MSG(DBG_FATAL_PRIO, "Queue error:queue failed to create bufpool!");
      return AR_EFAILED;
   }
   posal_queuepool_set_handle_from_heap_id(heap_id, handle);
   /* Commenting FATAL_PRIO as Slate_SIM doesn't have memorymap and usecase will halt*/
#ifndef AUDIOSSMODE
   AR_MSG(DBG_MED_PRIO, "Created Q Pool - heap_id = %lu, Handle 0x%lx", heap_id, handle);
#endif
   return AR_EOK;
}

void posal_queue_pool_reset(POSAL_HEAP_ID heap_id)
{
   // remove all lists other than the ones that have any nodes
   // with any luck, just bufmgr lists will have nodes
   posal_bufpool_pool_free_unused_lists(posal_queuepool_get_handle_from_heap_id(heap_id));
}

void posal_queue_pool_destroy(POSAL_HEAP_ID heap_id)
{
   posal_bufpool_pool_destroy(posal_queuepool_get_handle_from_heap_id(heap_id));
}

void posal_queue_destroy(posal_queue_t *q_ptr)
{
   posal_queue_internal_t *queue_ptr = (posal_queue_internal_t *)q_ptr;
   // check for NULL
   posal_queue_deinit(q_ptr);
   posal_memory_free(queue_ptr);
}

ar_result_t posal_queue_peek_forward(posal_queue_t *q_ptr, posal_queue_element_t **payload_ptr, void **iterator)
{
   posal_queue_internal_t *queue_ptr = (posal_queue_internal_t *)q_ptr;

   if (NULL == iterator || NULL == payload_ptr)
   {
      return AR_EBADPARAM;
   }

   *payload_ptr = NULL;

   posal_queue_element_list_t **it_list_pptr = (posal_queue_element_list_t **)iterator;

   // if queue is empty or iterator already reached the end of list then return.
   if (0 == queue_ptr->active_nodes || (queue_ptr->active_tail_ptr == *it_list_pptr))
   {
      *it_list_pptr = NULL;
      return AR_ENEEDMORE;
   }

   if (NULL == *it_list_pptr)
   {
      // initialize the iterator to the head.
      *it_list_pptr = queue_ptr->active_head_ptr;
   }
   else
   {
      // move the iterator to the next.
      *it_list_pptr = (*it_list_pptr)->next_ptr;
   }

   if (*it_list_pptr)
   {
      *payload_ptr = &((*it_list_pptr)->elem);
   }

   return AR_EOK;
}

ar_result_t posal_queue_peek_front(posal_queue_t *q_ptr, posal_queue_element_t **payload_ptr)
{
   posal_queue_internal_t *queue_ptr = (posal_queue_internal_t *)q_ptr;

   if (NULL == queue_ptr->channel_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Q SEND: channel not initialized on Q");
      return AR_EBADPARAM;
   }

   // acquire the mutex
   posal_queue_mutex_lock(queue_ptr);

   // make sure not empty (this is non-blocking mq).
   if (0 == queue_ptr->active_nodes)
   {
      posal_queue_mutex_unlock(queue_ptr);
      return AR_ENEEDMORE;
   }

   *payload_ptr = &(queue_ptr->active_head_ptr->elem);

   // release the mutex
   posal_queue_mutex_unlock(queue_ptr);

   return AR_EOK;
}

ar_result_t posal_queue_pop_back(posal_queue_t *q_ptr, posal_queue_element_t *payload_ptr)
{
   posal_queue_internal_t *queue_ptr = (posal_queue_internal_t *)q_ptr;
   if (NULL == queue_ptr->channel_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Q SEND: channel not initialized on Q");
      return AR_EBADPARAM;
   }

   // grab the mutex
   posal_queue_mutex_lock(queue_ptr);

#ifdef DEBUG_QUEUE_LOW
   AR_MSG(DBG_HIGH_PRIO,
          "Queue pop back 0x%x, active %lu, allocated %lu, max %lu",
          queue_ptr,
          queue_ptr->active_nodes,
          queue_ptr->num_nodes,
          queue_ptr->max_nodes);
#endif

   // make sure not empty (this is non-blocking mq).
   if (0 == queue_ptr->active_nodes)
   {
      posal_queue_mutex_unlock(queue_ptr);
      return AR_ENEEDMORE;
   }

   *payload_ptr = queue_ptr->active_tail_ptr->elem;
   queue_ptr->active_nodes--;

#ifdef DEBUG_POSAL_QUEUE
   uint32_t unOpcode = (uint32_t)(*payload_ptr >> 32);
   // filter out POSAL_DATA_BUFFER to avoid the flood.
   if (posal_globalstate.bEnableQLogging && 0x0000000AL != unOpcode)
   {
      AR_MSG(DBG_HIGH_PRIO,
             "Q PopBk: Thrd=0x%x, Q=0x%x, Msg=0x%x 0x%x",
             qurt_thread_get_id(),
             queue_ptr,
             unOpcode,
             (uint32_t)(*payload_ptr));
   }

#endif // DEBUG_POSAL_QUEUE

   // if mq is empty, clear signal.
   if (0 == queue_ptr->active_nodes)
   {
      posal_channel_internal_t *ch_ptr = (posal_channel_internal_t *)queue_ptr->channel_ptr;
      posal_signal_clear_with_bitmask_target_inline(&ch_ptr->anysig, queue_ptr->channel_bit);
   }
   else
   {
      // only advance the active tail if there is still a node for it to point to. otherwise leave it where it is.
      queue_ptr->active_tail_ptr = queue_ptr->active_tail_ptr->prev_ptr;
   }

   // release the mutex
   posal_queue_mutex_unlock(queue_ptr);

   return AR_EOK;
}

ar_result_t posal_queue_push_back_with_priority(posal_queue_t *q_ptr, posal_queue_element_t *payload_ptr, uint32_t priority)
{
   posal_queue_internal_t *queue_ptr = (posal_queue_internal_t *)q_ptr;

   ar_result_t result = posal_queue_push_back(q_ptr, payload_ptr);

   if (AR_SUCCEEDED(result) && queue_ptr->active_nodes > 1 && queue_ptr->is_priority_queue)
   {
      ((posal_priority_queue_element_list_t *)queue_ptr->active_tail_ptr)->priority = priority;

      if (priority > 0)
      {
         posal_priority_queue_element_list_t *src_node_ptr =
            (posal_priority_queue_element_list_t *)queue_ptr->active_tail_ptr;

         posal_priority_queue_element_list_t *dst_node_ptr = src_node_ptr->prev_ptr;

         for (uint32_t num_elements = 0; num_elements < (queue_ptr->active_nodes - 1); num_elements++)
         {
            if (priority <= dst_node_ptr->priority)
            {
               break;
            }

            dst_node_ptr = dst_node_ptr->prev_ptr;
         }

         if (dst_node_ptr->next_ptr == (posal_priority_queue_element_list_t *)queue_ptr->active_head_ptr)
         {
            // case when node is the highest priority and going at the top of the queue.
            queue_ptr->active_head_ptr = (posal_queue_element_list_t *)src_node_ptr;
         }

         if (dst_node_ptr->next_ptr == src_node_ptr)
         {
            return AR_EOK; // Already in the right place.
         }
         else
         {
            queue_ptr->active_tail_ptr = (posal_queue_element_list_t *)src_node_ptr->prev_ptr;

            if (dst_node_ptr != src_node_ptr)
            {
               // move to the src node  next to dest node. maintain circular connection
               src_node_ptr->prev_ptr->next_ptr = src_node_ptr->next_ptr;
               src_node_ptr->next_ptr->prev_ptr = src_node_ptr->prev_ptr;

               dst_node_ptr->next_ptr->prev_ptr = src_node_ptr;
               src_node_ptr->next_ptr           = dst_node_ptr->next_ptr;
               dst_node_ptr->next_ptr           = src_node_ptr;
               src_node_ptr->prev_ptr           = dst_node_ptr;
            }
         }
      }
   }

   return result;
}

static ar_result_t posal_queue_create_prealloc_nodes(posal_queue_t *q_ptr, posal_queue_init_attr_t *attr_ptr)
{
   ar_result_t result = AR_EOK;

   posal_queue_internal_t *queue_ptr = (posal_queue_internal_t *)q_ptr;
   // if no pre allocated nodes, then return EOK
   if (0 == attr_ptr->prealloc_nodes)
   {
      return result;
   }

   if (0 > attr_ptr->prealloc_nodes)
   {
      return AR_EBADPARAM;
   }

   uint32_t                    num_nodes_to_alloc = attr_ptr->prealloc_nodes;
   posal_queue_element_list_t *node_ptr;
   while (num_nodes_to_alloc)
   {
      // allocate node from the bufpool
      node_ptr = posal_queue_create_node(queue_ptr);
      if (NULL == node_ptr)
      {
         AR_MSG(DBG_FATAL_PRIO, "Queue error: unable to get node");
         posal_queue_free_all_nodes(queue_ptr);
         return AR_ENOMEMORY;
      }

#ifdef DEBUG_POSAL_QUEUE_POOL
      AR_MSG(DBG_MED_PRIO,
             "Queue pool: Allocated new node %p, num nodes to alloc %lu curr num nodes %lu",
             node_ptr,
             attr_ptr->prealloc_nodes,
             queue_ptr->num_nodes);
#endif

      if (0 == queue_ptr->num_nodes)
      {
         // assign all the list pointers to this node
         queue_ptr->active_head_ptr = node_ptr;
         queue_ptr->active_tail_ptr = node_ptr;

         node_ptr->prev_ptr = node_ptr;
         node_ptr->next_ptr = node_ptr;
      }
      else // add new node to the list
      {
         node_ptr->prev_ptr = queue_ptr->active_tail_ptr;
         node_ptr->next_ptr = queue_ptr->active_tail_ptr->next_ptr;

         queue_ptr->active_tail_ptr->next_ptr->prev_ptr = node_ptr;
         queue_ptr->active_tail_ptr->next_ptr           = node_ptr;
      }
      queue_ptr->num_nodes++;
      num_nodes_to_alloc--;
   }

   return result;
}

/*
 * Takes in preallocated memory rather than allocating for queue instance along with
 * specific attributes to be used.
 */
ar_result_t posal_queue_init(posal_queue_t *q_ptr, posal_queue_init_attr_t *attr_ptr)
{
   ar_result_t result = AR_EOK;

   posal_queue_internal_t *queue_ptr = (posal_queue_internal_t *)q_ptr;
   // verify incoming parameters
   if ((NULL == queue_ptr) || (NULL == attr_ptr))
   {
      return AR_EBADPARAM;
   }

   if (0 >= attr_ptr->max_nodes)
   {
      return AR_EBADPARAM;
   }

   // if number of prealloc nodes is greater than max nodes, return failure
   if (attr_ptr->prealloc_nodes > attr_ptr->max_nodes)
   {
      return AR_EBADPARAM;
   }

   // memset takes care of active_nodes, num_nodes, etc. all being set to zero
   POSAL_ZEROAT(queue_ptr);

   queue_ptr->max_nodes         = attr_ptr->max_nodes;
   queue_ptr->heap_id           = attr_ptr->heap_id;
   queue_ptr->is_priority_queue = attr_ptr->is_priority_queue ? 1 : 0;

   result = posal_queue_create_prealloc_nodes(q_ptr, attr_ptr);
   if (AR_DID_FAIL(result))
   {
      return result;
   }

   // initialize Mutex
   posal_inline_mutex_init(&(queue_ptr->queue_mutex));

#ifdef DEBUG_WITH_QUEUE_NAME
   // save name
   posal_strlcpy(queue_ptr->name, attr_ptr->name, POSAL_DEFAULT_NAME_LEN);
#endif

#if defined(DEBUG_POSAL_QUEUE) || defined(QUEUE_NODE_UTILIZATION_DEBUG)
   AR_MSG(DBG_LOW_PRIO,
          "Q CREATE: Q=0x%lx, PreAllocatedNodes=%lu, MaxNumNodes=%lu, ",
          (uint32_t)queue_ptr,
          attr_ptr->prealloc_nodes,
          attr_ptr->max_nodes);
#endif
   // return.
   return result;
}

ar_result_t posal_queue_create_v1(posal_queue_t **queue_pptr, posal_queue_init_attr_t *attr_ptr)
{
   if ((NULL == attr_ptr) || (NULL == queue_pptr))
   {
      return AR_EBADPARAM;
   }

   // allocate space
   if (NULL == (*queue_pptr = (posal_queue_t *)posal_memory_malloc(sizeof(posal_queue_internal_t), attr_ptr->heap_id)))
   {
      return AR_ENOMEMORY;
   }

   return posal_queue_init(*queue_pptr, attr_ptr);
}

void posal_queue_deinit(posal_queue_t *q_ptr)
{
   posal_queue_internal_t *queue_ptr = (posal_queue_internal_t *)q_ptr;
   // check for NULL
   if (!queue_ptr)
      return;

   posal_channel_internal_t *channel_ptr = (posal_channel_internal_t *)queue_ptr->channel_ptr;

   // release the channel bit if Q is added to a channel
   if (channel_ptr)
   {
      channel_ptr->unBitsUsedMask ^= queue_ptr->channel_bit;
   }
   // deinit channel ptr -- it is possible to double-deinit
   queue_ptr->channel_ptr = NULL;

   // return nodes, if any. print warning.
   if (queue_ptr->active_nodes != 0)
   {
      AR_MSG(DBG_HIGH_PRIO, "Warning: Queue was destroyed while %ld nodes present", queue_ptr->active_nodes);
   }

#if defined(DEBUG_POSAL_QUEUE) || defined(QUEUE_NODE_UTILIZATION_DEBUG)
   AR_MSG(DBG_LOW_PRIO,
          "Q DESTROY: Q=0x%lx, ActiveNodes=%lu, MaxNumNodes=%lu",
          (uint32_t)queue_ptr,
          queue_ptr->active_nodes,
          queue_ptr->max_nodes);
#endif

   posal_queue_free_all_nodes(queue_ptr);
   posal_inline_mutex_deinit(&queue_ptr->queue_mutex);
}

ar_result_t posal_queue_disable(posal_queue_t *posal_queue)
{
   posal_queue_internal_t *queue_ptr = (posal_queue_internal_t *)posal_queue;
   POSAL_ASSERT(queue_ptr != NULL);
#ifdef QUEUE_DISABLE_NEEDED
   posal_queue_mutex_lock(queue_ptr);
   queue_ptr->disable_flag = TRUE;
   posal_queue_mutex_unlock(queue_ptr);
   AR_MSG(DBG_HIGH_PRIO, "Queue 0x%p disabled", queue_ptr);
#else
   AR_MSG(DBG_ERROR_PRIO, "Queue disable not implemeneted");
#endif
   return AR_EOK;
}

ar_result_t posal_queue_enable_disable_signaling(posal_queue_t *q_ptr, bool_t is_enable)
{
   posal_queue_internal_t *queue_ptr = (posal_queue_internal_t *)q_ptr;

   POSAL_ASSERT(queue_ptr != NULL);

   // grab the mutex
   posal_queue_mutex_lock(queue_ptr);

   posal_channel_internal_t *ch_ptr = (posal_channel_internal_t *)queue_ptr->channel_ptr;
   queue_ptr->disable_signaling     = is_enable ? FALSE : TRUE;

   if (queue_ptr->disable_signaling)
   {
      // if signaling is disabled then clear the signal from the channel.
      posal_signal_clear_with_bitmask_target_inline(&ch_ptr->anysig, queue_ptr->channel_bit);
   }
   else if (queue_ptr->active_nodes > 0)
   {
      // if signaling is enabled and there are some elements in the queue then set the signal
      posal_signal_set_target_inline(&ch_ptr->anysig, queue_ptr->channel_bit);
   }
   // release the mutex
   posal_queue_mutex_unlock(queue_ptr);
   return AR_EOK;
}

static void posal_queue_free_all_nodes(posal_queue_internal_t *queue_ptr)
{

#ifdef DEBUG_POSAL_QUEUE_POOL
   AR_MSG(DBG_MED_PRIO,
          "Queue pool: Freeing %lu nodes, %lu active nodes",
          queue_ptr->num_nodes,
          queue_ptr->active_nodes);
#endif
   while (queue_ptr->num_nodes > 0)
   {
      posal_queue_element_list_t *node_ptr = queue_ptr->active_head_ptr->next_ptr;
      posal_queue_release_node(queue_ptr, queue_ptr->active_head_ptr);
      queue_ptr->active_head_ptr = node_ptr;
      queue_ptr->num_nodes--;
   }
   queue_ptr->active_nodes    = 0;
   queue_ptr->active_head_ptr = NULL;
   queue_ptr->active_tail_ptr = NULL;
   return;
}

ar_result_t posal_queue_set_attributes(posal_queue_init_attr_t *q_attr_ptr,
                                       POSAL_HEAP_ID            heap_id,
                                       uint32_t                 num_max_q_elem,
                                       uint32_t                 num_max_prealloc_q_elem,
                                       char_t                  *q_name_ptr)
{
   if (!q_attr_ptr || !q_name_ptr || !num_max_q_elem)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "posal_queue_set_attributes(): Invalid args, q_attr_ptr[0x%lX], num_max_ele[%lu], q_name_ptr[0x%lX]",
             q_attr_ptr,
             num_max_q_elem,
             q_name_ptr);

      return AR_EFAILED;
   }

   posal_queue_attr_init(q_attr_ptr);
   posal_queue_attr_set_heap_id(q_attr_ptr, heap_id);
   posal_queue_attr_set_max_nodes(q_attr_ptr, num_max_q_elem);
   posal_queue_attr_set_prealloc_nodes(q_attr_ptr, num_max_prealloc_q_elem);
   posal_queue_attr_set_name(q_attr_ptr, q_name_ptr);

   return AR_EOK;
}
