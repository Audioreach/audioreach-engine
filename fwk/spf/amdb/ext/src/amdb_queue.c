/**
 * \file amdb_queue.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amdb_queue.h"
#include "amdb_static.h"

/*----------------------------------------------------------------------------
 * Function Declarations
 * -------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
static bool_t amdb_queue_is_empty(const amdb_queue_t *queue_ptr)
{
   return (0 == queue_ptr->num_elements);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
static bool_t amdb_queue_is_full(const amdb_queue_t *queue_ptr)
{
   return (queue_ptr->max_elements == queue_ptr->num_elements);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
amdb_queue_t *amdb_queue_create(uint32_t max_items, uint32_t size_of_element, char name[], POSAL_HEAP_ID heap_id)
{
   uint32_t size_needed = 0;

   size_needed += sizeof(amdb_queue_t);
   size_needed += size_of_element * max_items;

   uint8_t *mem_ptr = (uint8_t *)posal_memory_malloc(size_needed, heap_id);
   if (NULL == mem_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "amdb: Failed to allocate memory for queue object %s.", name);
      return NULL;
   }

   amdb_queue_t *queue_ptr = (amdb_queue_t *)(mem_ptr);
   mem_ptr += sizeof(amdb_queue_t);
   queue_ptr->data_ptr        = mem_ptr;
   queue_ptr->push_location   = 0;
   queue_ptr->num_elements    = 0;
   queue_ptr->max_elements    = max_items;
   queue_ptr->size_of_element = size_of_element;
   strlcpy(queue_ptr->name, name, sizeof(queue_ptr->name) / sizeof(queue_ptr->name[0]));

   posal_nmutex_create(&queue_ptr->queue_nmutex, heap_id);
   posal_condvar_create(&queue_ptr->queue_push_condition, heap_id);
   posal_condvar_create(&queue_ptr->queue_pop_condition, heap_id);

   return queue_ptr;
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_queue_push(amdb_queue_t *queue_ptr, const void *in_data_ptr)
{
   posal_nmutex_lock(queue_ptr->queue_nmutex);
   while (amdb_queue_is_full(queue_ptr))
   {
      posal_condvar_wait(queue_ptr->queue_push_condition, queue_ptr->queue_nmutex);
   }

   uint8_t *push_location_ptr = queue_ptr->data_ptr + queue_ptr->push_location * queue_ptr->size_of_element;
   memscpy(push_location_ptr, queue_ptr->size_of_element, in_data_ptr, queue_ptr->size_of_element);
   queue_ptr->num_elements++;
   queue_ptr->push_location++;
   if (queue_ptr->push_location >= queue_ptr->max_elements)
   {
      queue_ptr->push_location = 0;
   }

   posal_condvar_signal(queue_ptr->queue_pop_condition); // In case others are waiting to pop.
   if (!amdb_queue_is_full(queue_ptr))
   {
      posal_condvar_signal(queue_ptr->queue_push_condition); // In case others are waiting to push.
   }
   posal_nmutex_unlock(queue_ptr->queue_nmutex);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_queue_pop(amdb_queue_t *queue_ptr, void *out_data_ptr)
{
   posal_nmutex_lock(queue_ptr->queue_nmutex);
   while (amdb_queue_is_empty(queue_ptr))
   {
      posal_condvar_wait(queue_ptr->queue_pop_condition, queue_ptr->queue_nmutex);
   }

   uint32_t pop_location = queue_ptr->push_location + queue_ptr->max_elements - queue_ptr->num_elements;
   if (pop_location >= queue_ptr->max_elements)
   {
      pop_location -= queue_ptr->max_elements;
   }
   uint8_t *pop_location_ptr = queue_ptr->data_ptr + pop_location * queue_ptr->size_of_element;
   memscpy(out_data_ptr, queue_ptr->size_of_element, pop_location_ptr, queue_ptr->size_of_element);
   queue_ptr->num_elements--;

   posal_condvar_signal(queue_ptr->queue_push_condition); // In case others are waiting to push.
   if (!amdb_queue_is_empty(queue_ptr))
   {
      posal_condvar_signal(queue_ptr->queue_pop_condition); // In case others are waiting to pop.
   }
   posal_nmutex_unlock(queue_ptr->queue_nmutex);
}

/*---------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : None

  --------------------------------------------------------------------------------------------------------------------*/
void amdb_queue_destroy(amdb_queue_t *queue_ptr)
{
   posal_nmutex_destroy(&queue_ptr->queue_nmutex);
   posal_condvar_destroy(&queue_ptr->queue_push_condition);
   posal_condvar_destroy(&queue_ptr->queue_pop_condition);
   posal_memory_free(queue_ptr);
}
