/**
 * \file spf_ref_counter.c
 * \brief
 *     This file contains the implementation for spf message utility functions.
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spf_ref_counter.h"

/*-------------------------------------------------------------------------
Type Declarations
-------------------------------------------------------------------------*/

/** Ref count structure. This is allocated internally whenever the user calls
    spf_ref_counter_create_ref().
 */
typedef struct spf_ref_counter_t
{
   // Ref counts for this entry.
   int32_t ref_count;

   // Mutex to ensure atomicity of operations.
   posal_mutex_t mutex;

   // To validate entries. Check against a hard-coded magic constant to ensure
   // user passes in memory allocated by the reference counter.
   uint32_t magic;

   // User's data. 64-bit for 8-byte alignment.
   uint64_t data_start;
} spf_ref_counter_t;

/* -----------------------------------------------------------------------
** Constant / Define Declarations
** ----------------------------------------------------------------------- */
#define SPF_REF_COUNTER_MAGIC_ALLOCED 0x2DC463F2

// Set magic to this before freeing to invalidate the entry.
#define SPF_REF_COUNTER_MAGIC_FREED 0x5DE1F794

/* =======================================================================
**                          Function Definitions
** ======================================================================= */
// Gets a ref count entry from the data pointer. Relies on the fact that the
// user memory is located a fixed distance from the start of the entry.
static ar_result_t spf_ref_counter_get_entry(void *data_ptr, spf_ref_counter_t **entry_pptr)
{
   ar_result_t        result = AR_EOK;
   spf_ref_counter_t *entry_ptr =
      (spf_ref_counter_t *)(((uint8_t *)data_ptr) - (sizeof(spf_ref_counter_t) - sizeof(uint64_t)));

   if (SPF_REF_COUNTER_MAGIC_ALLOCED != entry_ptr->magic)
   {
      AR_MSG(DBG_ERROR_PRIO, "spf_ref_counter: error: address 0x%p not owned by ref counter", data_ptr);

      return AR_EFAILED;
   }

   *entry_pptr = entry_ptr;
   return result;
}

ar_result_t spf_ref_counter_create_ref(uint32_t data_size_bytes, POSAL_HEAP_ID heap_id, void **data_pptr)
{
   ar_result_t result = AR_EOK;

   uint32_t entry_bytes = data_size_bytes + sizeof(spf_ref_counter_t) - sizeof(uint64_t); // data_start accounted for by
                                                                                          // data_size_bytes.

   // Alloc the entry.
   spf_ref_counter_t *entry_ptr = (spf_ref_counter_t *)posal_memory_malloc(entry_bytes, heap_id);
   if (NULL == entry_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "spf_ref_counter: create_ref() error allocating memory");
      return AR_ENOMEMORY;
   }

   result = posal_mutex_create(&(entry_ptr->mutex), heap_id);
   if (AR_DID_FAIL(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "spf_ref_counter: create_ref() error creating mutex");
      posal_memory_free(entry_ptr);
      return result;
   }

   // Init the entry.
   entry_ptr->ref_count = 1;
   entry_ptr->magic     = SPF_REF_COUNTER_MAGIC_ALLOCED;

   // Return the data pointer.
   *data_pptr = &(entry_ptr->data_start);
   return result;
}

ar_result_t spf_ref_counter_add_ref(void *data_ptr)
{
   ar_result_t        result    = AR_EOK;
   spf_ref_counter_t *entry_ptr = NULL;

   if (!data_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "spf_ref_counter: add_ref() received null core ptr; erroring out");
      return AR_EFAILED;
   }

   // Get entry and increment the ref count.
   result = spf_ref_counter_get_entry(data_ptr, &entry_ptr);
   if (AR_DID_FAIL(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "spf_ref_counter: add_ref() error getting entry");
      return result;
   }

   posal_mutex_lock(entry_ptr->mutex);
   entry_ptr->ref_count++;
   posal_mutex_unlock(entry_ptr->mutex);

   return result;
}

ar_result_t spf_ref_counter_remove_ref(void *                          data_ptr,
                                       spf_ref_counter_destroy_cb_func destroy_cb_func,
                                       void *                          cb_context_ptr)
{
   ar_result_t        result    = AR_EOK;
   spf_ref_counter_t *entry_ptr = NULL;

   if (!data_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "spf_ref_counter: remove_ref() received null core ptr; erroring out");
      return AR_EFAILED;
   }
   // Get entry and decrement the ref count.
   result = spf_ref_counter_get_entry(data_ptr, &entry_ptr);
   if (AR_DID_FAIL(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "spf_ref_counter: remove_ref() error getting entry");
      return result;
   }

   posal_mutex_lock(entry_ptr->mutex);

   if (0 >= entry_ptr->ref_count)
   {
      AR_MSG(DBG_ERROR_PRIO, "spf_ref_counter: remove_ref() error, ref count is below zero");
      posal_mutex_unlock(entry_ptr->mutex);
      return AR_EFAILED;
   }

   entry_ptr->ref_count--;

   // Call callback.
   if (NULL != destroy_cb_func)
   {
      destroy_cb_func(cb_context_ptr, entry_ptr->ref_count);
   }

   if (0 == entry_ptr->ref_count)
   {
      // Invalidate entry.
      entry_ptr->magic = SPF_REF_COUNTER_MAGIC_FREED;

      // Unlock.
      posal_mutex_unlock(entry_ptr->mutex);
      posal_mutex_destroy(&(entry_ptr->mutex));

      // Free memory.
      posal_memory_free(entry_ptr);
   }
   else
   {
      posal_mutex_unlock(entry_ptr->mutex);
   }

   return result;
}
