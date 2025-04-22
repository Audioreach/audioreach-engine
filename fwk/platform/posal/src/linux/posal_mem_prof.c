/**
 * \file posal_mem_prof.c
 * \brief
 *  	   This file contains a utility for memory profile.
 *
 * \copyright
 *       Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *       SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "posal_mem_prof.h"
#include "posal_internal.h"
#include "posal_memory_i.h"

#include "posal.h"

/* ----------------------------------------------------------------------------
 * Global Declarations/Definitions
 * ------------------------------------------------------------------------- */
#define POSAL_MEM_PROF_HASH_TABLE_SIZE 16
#define POSAL_MEM_PROF_HASH_TABLE_RESIZE_FACTOR 2
#define POSAL_MEM_PROF_MAGIC_NUMBER 0xCAFEC0DE

posal_mem_prof_t          g_posal_mem_prof;
posal_mem_prof_t *        g_posal_mem_prof_ptr = NULL;
extern posal_heap_table_t posal_heap_table[POSAL_HEAP_MGR_MAX_NUM_HEAPS];

#ifndef ALIGN_4_BYTES
#define ALIGN_4_BYTES(a) ((a + 3) & (0xFFFFFFFC))
#endif


/* -------------------------------------------------------------------------
 * Function Definitions
 * ------------------------------------------------------------------------- */
static void posal_mem_prof_mem_free(void *free_context_ptr, spf_hash_node_t *node_ptr)
{
   if (NULL != node_ptr)
   {
      posal_mem_prof_node_t *mem_prof_node_ptr = (posal_mem_prof_node_t *)(node_ptr);
      posal_memory_free(mem_prof_node_ptr);
   }
}

ar_result_t posal_mem_prof_init(POSAL_HEAP_ID heap_id)
{
   ar_result_t result = AR_EOK;

   AR_MSG(DBG_HIGH_PRIO, "POSAL MEM PROF: Init called.");
   if (NULL != g_posal_mem_prof_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "POSAL MEM PROF:g_posal_mem_prof_ptr is not null during init.");
      return AR_EFAILED;
   }

   /** Create mutex required for memory profiling */
   result = posal_mutex_create(&g_posal_mem_prof.prof_mutex, heap_id);
   if (result != AR_EOK)
   {
      AR_MSG(DBG_ERROR_PRIO, "POSAL MEM PROF:Failed to create mutex for posal mem prof.");
      return result;
   }

   /** Initialize mem prof variables */
   g_posal_mem_prof.heap_id         = heap_id;
   g_posal_mem_prof.mem_prof_status = POSAL_MEM_PROF_STOPPED;
   g_posal_mem_prof_ptr             = &g_posal_mem_prof;
   AR_MSG(DBG_HIGH_PRIO, "POSAL MEM PROF: Init Done.");
   return result;
}

ar_result_t posal_mem_prof_start()
{
   AR_MSG(DBG_HIGH_PRIO, "POSAL MEM PROF: Start called.");
   ar_result_t result = AR_EOK;
   if (NULL == g_posal_mem_prof_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "POSAL MEM PROF:g_posal_mem_prof_ptr is not null during start.");
      return AR_EFAILED;
   }

   posal_mutex_lock(g_posal_mem_prof_ptr->prof_mutex);
   if (POSAL_MEM_PROF_STARTED != g_posal_mem_prof.mem_prof_status)
   {
      result = spf_hashtable_init(&g_posal_mem_prof.mem_ht,
                                  g_posal_mem_prof_ptr->heap_id,
                                  POSAL_MEM_PROF_HASH_TABLE_SIZE,
                                  POSAL_MEM_PROF_HASH_TABLE_RESIZE_FACTOR,
                                  posal_mem_prof_mem_free,
                                  (void *)g_posal_mem_prof_ptr);
      if (AR_EOK != result)
      {
         posal_mutex_unlock(g_posal_mem_prof_ptr->prof_mutex);
         AR_MSG(DBG_ERROR_PRIO, "POSAL MEM PROF: Failed to init hashtable during posal mem prof start.");
         return result;
      }
   }

   g_posal_mem_prof.mem_prof_status = POSAL_MEM_PROF_STARTED;
   posal_mutex_unlock(g_posal_mem_prof_ptr->prof_mutex);
   AR_MSG(DBG_HIGH_PRIO, "POSAL MEM PROF: Start done.");
   return result;
}

ar_result_t posal_mem_prof_stop()
{
   AR_MSG(DBG_HIGH_PRIO, "POSAL MEM PROF: Stop Called.");
   ar_result_t result = AR_EOK;
   if (NULL == g_posal_mem_prof_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "POSAL MEM PROF:g_posal_mem_prof_ptr is not null during stop.");
      return AR_EFAILED;
   }

   posal_mutex_lock(g_posal_mem_prof_ptr->prof_mutex);
   if (POSAL_MEM_PROF_STARTED == g_posal_mem_prof.mem_prof_status)
   {
      spf_hashtable_deinit(&g_posal_mem_prof.mem_ht);
      g_posal_mem_prof.mem_prof_status = POSAL_MEM_PROF_STOPPED;
   }
   posal_mutex_unlock(g_posal_mem_prof_ptr->prof_mutex);

   AR_MSG(DBG_HIGH_PRIO, "POSAL MEM PROF: Stop done.");
   return result;
}

void posal_mem_prof_deinit()
{
   AR_MSG(DBG_HIGH_PRIO, "POSAL MEM PROF: Deinit Called.");
   if (NULL == g_posal_mem_prof_ptr)
   {
      return;
   }

   if (0 != g_posal_mem_prof_ptr->mem_ht.num_nodes)
   {
      spf_hashtable_deinit(&g_posal_mem_prof_ptr->mem_ht);
   }
   posal_mutex_destroy(&g_posal_mem_prof_ptr->prof_mutex);
   memset(&g_posal_mem_prof, 0, sizeof(posal_mem_prof_t));
   g_posal_mem_prof_ptr = NULL;
   AR_MSG(DBG_HIGH_PRIO, "POSAL MEM PROF: Deinit done.");
   return;
}

static void posal_mem_prof_update_stats(void *ptr, POSAL_HEAP_ID update_heap_id, bool_t is_malloc, uint32_t mem_size)
{
   posal_mem_prof_node_t *mem_prof_node_ptr = NULL;
   uint32_t               actual_size       = (mem_size - sizeof(posal_mem_prof_marker_t));

#ifdef DEBUG_POSAL_MEM_PROF
   AR_MSG(DBG_HIGH_PRIO,
          "POSAL MEM PROF: is_malloc = %lu, ptr = 0x%X, update_heap_id = 0x%X, mem_size = %lu",
          is_malloc,
          ptr,
          update_heap_id,
          mem_size);
#endif

   /** Acquire the hashnode for given heap id */
   mem_prof_node_ptr = (posal_mem_prof_node_t *)spf_hashtable_find(&g_posal_mem_prof_ptr->mem_ht,
                                                                   &update_heap_id,
                                                                   sizeof(POSAL_HEAP_ID));

   if (is_malloc)
   {
      /** If No entry is  present for the given heap id, create one */
      if (NULL == mem_prof_node_ptr)
      {
         posal_mem_prof_node_t *new_mem_prof_node_ptr =
            posal_memory_malloc(sizeof(posal_mem_prof_node_t), g_posal_mem_prof_ptr->heap_id);
         if (NULL == new_mem_prof_node_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, "POSAL MEM PROF: Failed to allocate memory for hash node.");
            return;
         }
         memset(new_mem_prof_node_ptr, 0, sizeof(posal_mem_prof_node_t));
         new_mem_prof_node_ptr->hash_node.key_ptr  = &new_mem_prof_node_ptr->heap_id;
         new_mem_prof_node_ptr->hash_node.key_size = sizeof(POSAL_HEAP_ID);
         new_mem_prof_node_ptr->hash_node.next_ptr = NULL;
         new_mem_prof_node_ptr->heap_id            = update_heap_id;
         new_mem_prof_node_ptr->mem_count          = actual_size;
         spf_hashtable_insert(&g_posal_mem_prof_ptr->mem_ht, &new_mem_prof_node_ptr->hash_node);
      }
      else /** Update the mem count if a hash node is cound for the given heap id */
      {
         mem_prof_node_ptr->mem_count += (actual_size);
      }
   }
   else
   {
      if (NULL == mem_prof_node_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "POSAL MEM PROF: WARNING Freeing from node which doesn't exist. Ignoring");
      }
      else
      {
         if (mem_prof_node_ptr->mem_count < actual_size)
         {
            AR_MSG(DBG_ERROR_PRIO, "POSAL MEM PROF: WARNING mem count less than amount being freed");
            mem_prof_node_ptr->mem_count = 0;
         }
         else
         {
            mem_prof_node_ptr->mem_count -= actual_size;
         }

         /** Remove the hash node if the mem count reaches 0 */
         if (0 == mem_prof_node_ptr->mem_count)
         {
            spf_hashtable_remove(&g_posal_mem_prof_ptr->mem_ht,
                                 &mem_prof_node_ptr->heap_id,
                                 sizeof(POSAL_HEAP_ID),
                                 &mem_prof_node_ptr->hash_node);
         }
      }
   }
}

uint32_t posal_mem_prof_get_mem_size(void *ptr, POSAL_HEAP_ID orig_heap_id)
{
   return 0;
}

inline void posal_mem_prof_pre_process_malloc(POSAL_HEAP_ID  orig_heap_id,
                                              POSAL_HEAP_ID *heap_id_ptr,
                                              uint32_t *     bytes_ptr)
{
   (*heap_id_ptr) = GET_ACTUAL_HEAP_ID(orig_heap_id);

   if ((NULL == g_posal_mem_prof_ptr) || (orig_heap_id < POSAL_HEAP_MGR_MAX_NUM_HEAPS))
   {
      return;
   }

   /** Modify the bytes required only if profiling is enabled */
   posal_mutex_lock(g_posal_mem_prof_ptr->prof_mutex);
   if (POSAL_MEM_PROF_STARTED == g_posal_mem_prof_ptr->mem_prof_status)
   {
#ifdef DEBUG_POSAL_MEM_PROF
      AR_MSG(DBG_HIGH_PRIO, "POSAL MEM PROF: posal_mem_prof_pre_process_malloc. bytes = %lu", *bytes_ptr);
#endif
      (*bytes_ptr) = ALIGN_4_BYTES((*bytes_ptr)) + sizeof(posal_mem_prof_marker_t);
   }
   posal_mutex_unlock(g_posal_mem_prof_ptr->prof_mutex);
   return;
}

inline void posal_mem_prof_post_process_malloc(void *ptr, POSAL_HEAP_ID orig_heap_id, bool_t is_mem_tracked)
{
   bool_t                   IS_MALLOC_TRUE = TRUE;
   uint32_t                 mem_size       = 0;
   posal_mem_prof_marker_t *marker_ptr     = NULL;
   /** Do not track unmodified heap id since they don't belong to any cntrs or modules */
   if ((NULL == g_posal_mem_prof_ptr) || (orig_heap_id < POSAL_HEAP_MGR_MAX_NUM_HEAPS) || (!is_mem_tracked))
   {
      return;
   }

   posal_mutex_lock(g_posal_mem_prof_ptr->prof_mutex);
   if ((NULL == ptr) || (POSAL_MEM_PROF_STOPPED == g_posal_mem_prof_ptr->mem_prof_status))
   {
      posal_mutex_unlock(g_posal_mem_prof_ptr->prof_mutex);
      return;
   }

   mem_size = posal_mem_prof_get_mem_size(ptr, orig_heap_id);
#ifdef DEBUG_POSAL_MEM_PROF
   AR_MSG(DBG_HIGH_PRIO,
          "POSAL MEM PROF: posal_mem_prof_post_process_malloc. ptr = 0x%X, mem_size = %lu",
          ptr,
          mem_size);
#endif
   if (8 >= mem_size)
   {
      posal_mutex_unlock(g_posal_mem_prof_ptr->prof_mutex);
      return;
   }

   /** Update the tail of the memory allocated with heap id associated with it */
   marker_ptr          = (posal_mem_prof_marker_t *)(((uint8_t *)ptr) + mem_size - sizeof(posal_mem_prof_marker_t));
   marker_ptr->heap_id = orig_heap_id;

   /** Update the tail of the memory allocated with with a magic number as well */
   marker_ptr->magic_number = POSAL_MEM_PROF_MAGIC_NUMBER;

   /** Update stats */
   posal_mem_prof_update_stats(ptr, orig_heap_id, IS_MALLOC_TRUE, mem_size);
   posal_mutex_unlock(g_posal_mem_prof_ptr->prof_mutex);
   return;
}

inline void posal_mem_prof_process_free(void *ptr)
{
   bool_t                   IS_MALLOC_FALSE = FALSE;
   uint32_t                 mem_size        = 0;
   POSAL_HEAP_ID            heap_id         = POSAL_HEAP_INVALID;
   posal_mem_prof_marker_t *marker_ptr      = NULL;

   if ((NULL == ptr) || (NULL == g_posal_mem_prof_ptr))
   {
      return;
   }

   posal_mutex_lock(g_posal_mem_prof_ptr->prof_mutex);
   if (POSAL_MEM_PROF_STARTED == g_posal_mem_prof_ptr->mem_prof_status)
   {
      mem_size = posal_mem_prof_get_mem_size(ptr, heap_id);
#ifdef DEBUG_POSAL_MEM_PROF
      AR_MSG(DBG_HIGH_PRIO,
             "POSAL MEM PROF: posal_mem_prof_pre_process_malloc. ptr = 0x%X, mem_size = %lu",
             ptr,
             mem_size);
#endif
      if ((8 >= mem_size) || (ALIGN_4_BYTES(mem_size) != mem_size))
      {
         posal_mutex_unlock(g_posal_mem_prof_ptr->prof_mutex);
         return;
      }

      /** Get heap id from tail */
      marker_ptr = (posal_mem_prof_marker_t *)(((uint8_t *)ptr) + mem_size - sizeof(posal_mem_prof_marker_t));

      /** Get and check for the magic number from tail and reset it and update stats if valid */
      if (POSAL_MEM_PROF_MAGIC_NUMBER == marker_ptr->magic_number)
      {
         marker_ptr->magic_number = 0; // Reset the magic number so it wont come by accident later
         posal_mem_prof_update_stats(ptr, marker_ptr->heap_id, IS_MALLOC_FALSE, mem_size);
      }
   }
   posal_mutex_unlock(g_posal_mem_prof_ptr->prof_mutex);
   return;
}

void posal_mem_prof_query(POSAL_HEAP_ID heap_id, uint32_t *mem_usage_ptr)
{
   posal_mem_prof_node_t *mem_prof_node_ptr = NULL;

   if ((NULL == g_posal_mem_prof_ptr) || (NULL == mem_usage_ptr))
   {
      return;
   }

   /** Initialize */
   *mem_usage_ptr = 0;

   posal_mutex_lock(g_posal_mem_prof_ptr->prof_mutex);
   if (POSAL_MEM_PROF_STARTED == g_posal_mem_prof_ptr->mem_prof_status)
   {
      /**  Find the hash node and update mem usage if valid entry is found */
      mem_prof_node_ptr =
         (posal_mem_prof_node_t *)spf_hashtable_find(&g_posal_mem_prof_ptr->mem_ht, &heap_id, sizeof(POSAL_HEAP_ID));

      if (NULL != mem_prof_node_ptr)
      {
         *mem_usage_ptr = mem_prof_node_ptr->mem_count;
      }
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "POSAL MEM PROF: WARNING mem prof query when profiling is stopped");
   }
   posal_mutex_unlock(g_posal_mem_prof_ptr->prof_mutex);
}
