/**
 * \file amdb.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amdb_internal.h"

amdb_t  g_amdb;
amdb_t *g_amdb_ptr = &g_amdb;

/*----------------------------------------------------------------------------------------------------------------------
   AMDB stub and capi vtables
  --------------------------------------------------------------------------------------------------------------------*/
const amdb_node_vtbl_t amdb_node_vtbl[2] =
   { { stub_resolve_symbols, stub_store_static_functions, stub_clear_symbols, stub_end },
     { capi_resolve_symbols, capi_store_static_functions, capi_clear_symbols, capi_end } };

/*----------------------------------------------------------------------------------------------------------------------
 * DESCRIPTION:
 *
 *--------------------------------------------------------------------------------------------------------------------*/
ar_result_t amdb_init(POSAL_HEAP_ID heap_id, bool_t init_cmd_thread)
{
   ar_result_t result = AR_EOK;
   amdb_t *    me     = g_amdb_ptr;
   AR_MSG(DBG_HIGH_PRIO, "AMDB: Init called");

   me->heap_id = heap_id;
   result      = spf_hashtable_init(&me->ht,
                               heap_id,
                               AMDB_NODES_HASH_TABLE_SIZE,
                               AMDB_HASH_TABLE_RESIZE_FACTOR,
                               amdb_node_free,
                               me);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "amdb: hashtable init failed %d", result);
      return result;
   }

   result = spf_hashtable_init(&me->load_ht,
                               heap_id,
                               AMDB_NODES_HASH_TABLE_SIZE,
                               AMDB_HASH_TABLE_RESIZE_FACTOR,
                               amdb_load_handle_holder_free,
                               me);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "amdb: load hashtable init failed %d", result);
      return result;
   }

   posal_mutex_create(&me->mutex_node, me->heap_id);

   result = amdb_init_dynamic(me->heap_id);

   amdb_reg_built_in_modules();

   if (init_cmd_thread)
   {
      amdb_thread_init(heap_id);
   }
   return result;
}

/*----------------------------------------------------------------------------------------------------------------------
 * DESCRIPTION:
 *
 *--------------------------------------------------------------------------------------------------------------------*/
void amdb_deinit(bool_t deinit_cmd_thread)
{
   amdb_t *me = g_amdb_ptr;

   if (deinit_cmd_thread)
   {
      amdb_thread_deinit();
   }

   // Deinit dynamic also destroys the parallel loader threads
   // THis is use to implicitly wait for thread to finish any loading
   amdb_deinit_dynamic();
   posal_mutex_lock(me->mutex_node);
   spf_hashtable_deinit(&me->ht);
   spf_hashtable_deinit(&me->load_ht);
   posal_mutex_unlock(me->mutex_node);
   posal_mutex_destroy(&me->mutex_node);
}

/*----------------------------------------------------------------------------------------------------------------------
 * DESCRIPTION:
 *
 *--------------------------------------------------------------------------------------------------------------------*/
ar_result_t amdb_register(uint32_t    module_type,
                          uint32_t    module_id,
                          void *      f1,
                          void *      f2,
                          uint32_t    filename_len,
                          const char *filename_str,
                          uint32_t    tag_len,
                          const char *tag_str,
                          bool_t      is_built_in)
{
   ar_result_t  result   = AR_EOK;
   amdb_t *     amdb_ptr = g_amdb_ptr;
   amdb_node_t *node_ptr = NULL;
   amdb_capi_t *capi_ptr = NULL;
   amdb_stub_t *stub_ptr = NULL;
   bool_t       is_stub  = (((NULL == f1) || (NULL == f2)) && ((NULL == filename_str) || (NULL == tag_str)) &&
                     (AMDB_MODULE_TYPE_FRAMEWORK != module_type));
   bool_t   is_static  = ((is_stub) || ((0 == filename_len) && (NULL == filename_str)));
   uint32_t alloc_size = 0;

   bool_t print_msg = TRUE;
#ifdef SIM
   // do not print messages on SIM, to reduce logging. print on target & for non-built-in registrations.
   print_msg = !is_built_in;
#endif
   if (print_msg)
   {
      AR_MSG(DBG_HIGH_PRIO,
             "AMDB_REG:module_type = %lu, module_id = 0x%lX, is_stub = %u, is_static = %u, is_built_in = %u, "
             "filename_len "
             "= %d, tag_len = %d",
             module_type,
             module_id,
             is_stub,
             is_static,
             is_built_in,
             filename_len,
             tag_len);
   }

#ifdef SIM
// some strings may not be null terminated (since length is given); but %s looks at null term only.
// AR_MSG(DBG_HIGH_PRIO, "AMDB_REG:file name %s, tag %s", filename_str, tag_str);
#endif

   posal_mutex_lock(amdb_ptr->mutex_node);
   spf_hash_node_t *phn = spf_hashtable_find(&amdb_ptr->ht, &module_id, sizeof(module_id));
   if (phn)
   {
      posal_mutex_unlock(amdb_ptr->mutex_node);
      AR_MSG(DBG_ERROR_PRIO, "amdb: AMDB Module is already registered");
      THROW(result, AR_EALREADY);
   }
   posal_mutex_unlock(amdb_ptr->mutex_node);

   if (is_stub)
   {
      alloc_size = sizeof(amdb_stub_t);
   }
   else
   {
      if (is_static)
      {
         alloc_size = sizeof(amdb_capi_t);
      }
      else
      {
         alloc_size = align_to_8_byte(sizeof(amdb_capi_t)) + sizeof(amdb_dynamic_t);
         // 2 is added to store null char at the end
         alloc_size += (is_built_in) ? 0 : (tag_len + filename_len + 2);
      }
   }
   node_ptr = (amdb_node_t *)posal_memory_malloc(alloc_size, amdb_ptr->heap_id);
   if (0 == node_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "AMDB: Failed to allocate node memory");
      THROW(result, AR_ENOMEMORY);
   }
   memset(node_ptr, 0, alloc_size);
   if (is_stub)
   {
      stub_ptr                            = (amdb_stub_t *)node_ptr;
      stub_ptr->node.flags.interface_type = AMDB_INTERFACE_TYPE_STUB;
   }
   else
   {
      capi_ptr                            = (amdb_capi_t *)node_ptr;
      capi_ptr->node.flags.interface_type = AMDB_INTERFACE_TYPE_CAPI;
   }
   result = amdb_node_init(node_ptr,
                           is_static,
                           is_built_in,
                           module_type,
                           module_id,
                           (void *)f1,
                           (void *)f2,
                           tag_len,
                           tag_str,
                           filename_len,
                           filename_str);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "AMDB: Failed to initialize the node, type = 0x%X, id = 0x%X", module_type, module_id);
      THROW(result, result);
   }
   result = amdb_insert_node(node_ptr);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "AMDB: node insertion failed %d", result);
      if (!is_static)
      {
         amdb_dynamic_t* dyn = amdb_node_get_dyn(node_ptr);
         posal_inline_mutex_deinit(&dyn->dl_mutex);
      }
      THROW(result, result);
   }

   CATCH(result)
   {
      if (NULL != node_ptr)
      {
         posal_memory_free(node_ptr);
      }
   }
   return result;
}

/*----------------------------------------------------------------------------------------------------------------------
 * DESCRIPTION:
 *
 *--------------------------------------------------------------------------------------------------------------------*/
ar_result_t amdb_deregister(uint32_t module_id)
{
   ar_result_t result = AR_EOK;

   amdb_t *         me       = g_amdb_ptr;
   amdb_node_t *    node_ptr = NULL;
   spf_hashtable_t *ht       = &me->ht;
   int              key      = module_id;

   AR_MSG(DBG_LOW_PRIO, "AMDB: DEREG called, id = 0x%X", module_id);
   posal_mutex_lock(me->mutex_node);
   node_ptr = amdb_node_hashtable_find(ht, &key, sizeof(key));

   if ((node_ptr) && (FALSE == node_ptr->flags.is_built_in))
   {
      AR_MSG(DBG_LOW_PRIO, "AMDB: DEREG : De-registering module (0x%lX)", module_id);
      result = spf_hashtable_remove(ht, &node_ptr->key, sizeof(node_ptr->key), &node_ptr->hn);

      if (AR_EOK == result)
      {
         posal_mutex_unlock(me->mutex_node);
         THROW(result, AR_EOK);
      }
   }
   posal_mutex_unlock(me->mutex_node);
   THROW(result, AR_EFAILED);

   CATCH(result)
   {
      AR_MSG(DBG_ERROR_PRIO, "AMDB: DEREG : De-register failed for module (0x%lX)", module_id);
   }
   return result;
}

/*----------------------------------------------------------------------------------------------------------------------
   DESCRIPTION:

   The callback function that is called when the handles are available for all modules. If NULL, this function will be
   blocking.

   1. Populate appropriate callback functions/context based on incoming param
   2. Start  looping through all the requested modules
   3. get the module from the database.
   4. If the module is not present is the database - populate the result section and continue to the next module
   5. If the module is present and loaded, module handle info will have required info populated when we get node
   6. If the module is present and not loaded, prepare for dynamic loading
   7. If NO dynamic loading happened, call the given callback and free the default  callback mechanism
   8. If ANY dynamic loading happened, then the loader handles will not be null, releasing the loader handles
      will call the given/default callback
  --------------------------------------------------------------------------------------------------------------------*/
void amdb_request_module_handles(spf_list_node_t *           module_handle_info_list_ptr,
                                 amdb_get_modules_callback_f callback_function,
                                 void *                      callback_context)
{
   handle_get_modules_request_list(module_handle_info_list_ptr, callback_function, callback_context);
}
/*----------------------------------------------------------------------------------------------------------------------
   DESCRIPTION:
   Releases the handles. If the reference counters reach their lower limits, the module will closed and freed
  --------------------------------------------------------------------------------------------------------------------*/
void amdb_release_module_handles(spf_list_node_t *module_handle_info_list_ptr)
{
   for (; NULL != module_handle_info_list_ptr; LIST_ADVANCE(module_handle_info_list_ptr))
   {
      amdb_module_handle_info_t *h_info_ptr = (amdb_module_handle_info_t *)module_handle_info_list_ptr->obj_ptr;
      if ((AR_DID_FAIL(h_info_ptr->result)) || (NULL == h_info_ptr->handle_ptr))
      {
         h_info_ptr->handle_ptr = NULL;
         continue;
      }

      amdb_node_t *node_ptr = amdb_handle_to_node(h_info_ptr->handle_ptr);

      if (NULL != node_ptr)
      {
         AR_MSG(DBG_LOW_PRIO, "AMDB_RELEASE : Releasing module (0x%lX), handle = 0x%X", node_ptr->key, node_ptr);
         amdb_node_dec_dl_ref(node_ptr);
         amdb_node_dec_mem_ref(node_ptr);
      }
      h_info_ptr->handle_ptr = NULL;
   }
}

/*----------------------------------------------------------------------------------------------------------------------
 * DESCRIPTION:
 * Release and unload all modules, reset amdb
 *--------------------------------------------------------------------------------------------------------------------*/
ar_result_t amdb_reset(bool_t is_flush_needed, bool_t is_reset_needed)
{
   ar_result_t result = AR_EOK;
   amdb_t *    me_ptr = g_amdb_ptr;

   AR_MSG(DBG_HIGH_PRIO, "AMDB: Reset called, flush = %lu, reset = %lu", is_flush_needed, is_reset_needed);

   if (is_flush_needed)
   {
      result |= amdb_thread_reset(is_flush_needed, is_reset_needed);
   }

   if (is_reset_needed)
   {
      amdb_module_handle_info_t handle;
      handle.result = AR_EOK;

      spf_list_node_t list_node;
      list_node.next_ptr = NULL;
      list_node.obj_ptr  = &handle;

      /*  Release all module handles */
      for (uint32_t i = 0; i < me_ptr->load_ht.table_size; i++)
      {
         spf_hash_node_t *hash_node_ptr = me_ptr->load_ht.table_ptr[i];

         while (hash_node_ptr)
         {
            amdb_load_handle_t *handle_holder_ptr = STD_RECOVER_REC(amdb_load_handle_t, hn, hash_node_ptr);
            handle.handle_ptr                     = (void *)handle_holder_ptr->module_handle_ptr;

            amdb_release_module_handles(&list_node);

            hash_node_ptr = hash_node_ptr->next_ptr;
         }
      }

      /* Deinit and reinit amdb */
      bool_t        INIT_CMD_THREAD_FALSE = FALSE; // Do not destroy and recreate the amdb cmd thread
      POSAL_HEAP_ID heap_id               = me_ptr->heap_id;

      amdb_deinit(INIT_CMD_THREAD_FALSE);

      if (AR_DID_FAIL(result = amdb_init(heap_id, INIT_CMD_THREAD_FALSE)))
      {
         AR_MSG(DBG_ERROR_PRIO, "AMDB: init after reset failed with result: %lu", result);
         return result;
      }
   }
   return result;
}
