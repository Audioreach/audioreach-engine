/**
 * \file amdb_utils.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amdb_internal.h"

extern amdb_t *g_amdb_ptr;

extern const amdb_static_capi_module_t amdb_static_capi_modules[];
extern const uint32_t                  amdb_num_static_capi_modules;

extern const amdb_dynamic_capi_module_t amdb_dynamic_capi_modules[];
extern const uint32_t                   amdb_num_dynamic_capi_modules;

// When SPF is compiled standalone, SPF modules are not part of base image and
// need to be registered seperately with AMDB. Define AMDB_REG_SPF_MODULES to
// register SPF modules with AMDB when SPF is compiled standalone.
#ifdef AMDB_REG_SPF_MODULES
extern const amdb_static_capi_module_t amdb_spf_static_capi_modules[];
extern const uint32_t                  amdb_spf_num_static_capi_modules;

extern const amdb_dynamic_capi_module_t amdb_spf_dynamic_capi_modules[];
extern const uint32_t                   amdb_spf_num_dynamic_capi_modules;
#endif

extern const amdb_module_id_t amdb_virtual_stub_modules[];
extern const uint32_t         amdb_num_virtual_stub_modules;

extern ar_result_t amdb_reg_built_in_private_dynamic_modules();

/*----------------------------------------------------------------------------------------------------------------------
 * DESCRIPTION:
 *
 *--------------------------------------------------------------------------------------------------------------------*/
ar_result_t amdb_register_built_in(uint32_t    module_type,
                                   uint32_t    module_id,
                                   void       *get_static_properties_f,
                                   void       *init_f,
                                   const char *filename_str,
                                   const char *tag_str)
{
   return amdb_register(module_type,
                        module_id,
                        get_static_properties_f,
                        init_f,
                        0, // the strings can be NULL, so sending 0 len, it shouldn't matter coz we just copy struct
                        filename_str,
                        0, // the strings can be NULL, so sending 0 len, it shouldn't matter coz we just copy struct
                        tag_str,
                        TRUE);
}
/*----------------------------------------------------------------------------------------------------------------------
 * DESCRIPTION:
 *
 *--------------------------------------------------------------------------------------------------------------------*/
static ar_result_t amdb_reg_built_in_dynamic_modules()
{
   ar_result_t result = AR_EOK;

   uint64_t start_time = posal_timer_get_time_in_msec();

   AR_MSG(DBG_HIGH_PRIO, "AMDB: adding dynamic built in modules to AMDB");
   // Even though module are separately listed in this file, there's no repetition from CAPI to CAPI V2 list.
   // add modules to the AMDB
   uint32_t num_capi_added      = 0;
   uint32_t num_capi_add_failed = 0;

   for (uint32_t i = 0; i < amdb_num_dynamic_capi_modules; i++)
   {
      result = amdb_register_built_in(amdb_dynamic_capi_modules[i].mtype,
                                      amdb_dynamic_capi_modules[i].mid,
                                      NULL,
                                      NULL,
                                      amdb_dynamic_capi_modules[i].filename,
                                      amdb_dynamic_capi_modules[i].tag);

      if (AR_DID_FAIL(result))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "AMDB: Failed to add dynamic CAPI V2 module %lX, 0x%lX. result %d. ",
                amdb_dynamic_capi_modules[i].mtype,
                amdb_dynamic_capi_modules[i].mid,
                result);
         num_capi_add_failed++;
      }
      else
      {
         num_capi_added++;
      }
   }

#ifdef AMDB_REG_SPF_MODULES
   for (uint32_t i = 0; i < amdb_spf_num_dynamic_capi_modules; i++)
   {
      result = amdb_register_built_in(amdb_spf_dynamic_capi_modules[i].mtype,
                                      amdb_spf_dynamic_capi_modules[i].mid,
                                      NULL,
                                      NULL,
                                      amdb_spf_dynamic_capi_modules[i].filename,
                                      amdb_spf_dynamic_capi_modules[i].tag);

      if (AR_DID_FAIL(result))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "AMDB: Failed to add SPF dynamic CAPI V2 module %lX, 0x%lX. result %d. ",
                amdb_spf_dynamic_capi_modules[i].mtype,
                amdb_spf_dynamic_capi_modules[i].mid,
                result);
         num_capi_add_failed++;
      }
      else
      {
         num_capi_added++;
      }
   }
#endif

   AR_MSG(DBG_HIGH_PRIO,
          "AMDB: Registering built-in CAPI V2 dynamic modules. Num added %lu, Num Failed %lu, Total %lu.",
          num_capi_added,
          num_capi_add_failed,
          amdb_num_dynamic_capi_modules);
   uint64_t end_time = posal_timer_get_time_in_msec();
   AR_MSG(DBG_HIGH_PRIO,
          "AMDB: amdb_reg_built_in_dynamic_modules done adding dynamic modules to AMDB. Time taken %lu ms",
          (uint32_t)(end_time - start_time));

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------
 * DESCRIPTION:
 *
 *--------------------------------------------------------------------------------------------------------------------*/
static ar_result_t amdb_reg_built_in_static_modules()
{
   ar_result_t result     = AR_EOK;
   uint64_t    start_time = posal_timer_get_time_in_msec();
   AR_MSG(DBG_HIGH_PRIO, "AMDB: Adding static modules to AMDB. ");
   // Even though module are separately listed in this file, there's no repetition from CAPI to CAPi V2 list.
   // add modules to the data base
   uint32_t num_stub_added = 0, num_capi_added = 0;
   uint32_t num_stub_add_failed = 0, num_capi_add_failed = 0;

   for (uint32_t i = 0; i < amdb_num_static_capi_modules; i++)
   {
      result = amdb_register_built_in(amdb_static_capi_modules[i].mtype,
                                      amdb_static_capi_modules[i].mid,
                                      (void *)amdb_static_capi_modules[i].get_static_prop_fn,
                                      (void *)amdb_static_capi_modules[i].init_fn,
                                      NULL,
                                      NULL);

      if (AR_DID_FAIL(result))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "AMDB: Failed to add static CAPI V2 module %lX, 0x%lX. result %d. ",
                amdb_static_capi_modules[i].mtype,
                amdb_static_capi_modules[i].mid,
                result);
         num_capi_add_failed++;
      }
      else
      {
         num_capi_added++;
      }
   }

#ifdef AMDB_REG_SPF_MODULES
   for (uint32_t i = 0; i < amdb_spf_num_static_capi_modules; i++)
   {
      result = amdb_register_built_in(amdb_spf_static_capi_modules[i].mtype,
                                      amdb_spf_static_capi_modules[i].mid,
                                      (void *)amdb_spf_static_capi_modules[i].get_static_prop_fn,
                                      (void *)amdb_spf_static_capi_modules[i].init_fn,
                                      NULL,
                                      NULL);

      if (AR_DID_FAIL(result))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "AMDB: Failed to add SPF static CAPI V2 module %lX, 0x%lX. result %d. ",
                amdb_spf_static_capi_modules[i].mtype,
                amdb_spf_static_capi_modules[i].mid,
                result);
         num_capi_add_failed++;
      }
      else
      {
         num_capi_added++;
      }
   }
#endif

   for (uint32_t i = 0; i < amdb_num_virtual_stub_modules; i++)
   {
      result = amdb_register_built_in(amdb_virtual_stub_modules[i].mtype,
                                      amdb_virtual_stub_modules[i].mid,
                                      NULL,
                                      NULL,
                                      NULL,
                                      NULL);
      if (AR_DID_FAIL(result))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "AMDB: Failed to add static virtual stub module %lX, 0x%lX. result %d. ",
                amdb_virtual_stub_modules[i].mtype,
                amdb_virtual_stub_modules[i].mid,
                result);
         num_stub_add_failed++;
      }
      else
      {
         num_stub_added++;
      }
   }

   AR_MSG(DBG_HIGH_PRIO,
          "AMDB: Registering static modules to AMDB. Num added (CAPI V2, stub)=(%lu, %lu), Num Failed (%lu, %lu)).",
          num_capi_added,
          num_stub_added,
          num_capi_add_failed,
          num_stub_add_failed);
   uint64_t end_time = posal_timer_get_time_in_msec();
   AR_MSG(DBG_HIGH_PRIO,
          "AMDB: ar_register_static_modules done adding static modules to AMDB. Time taken %lu ms",
          (uint32_t)(end_time - start_time));

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------
 * DESCRIPTION:
 *
 *--------------------------------------------------------------------------------------------------------------------*/
ar_result_t amdb_reg_built_in_modules()
{
   ar_result_t result = AR_EOK;

   uint64_t start_time = posal_timer_get_time_in_msec();

   AR_MSG(DBG_MED_PRIO, "AMDB: Registering built-in dynamic");

   result = amdb_reg_built_in_private_dynamic_modules();
   if (AR_DID_FAIL(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "AMDB: Registering built-in private dynamic modules failed %d", result);
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "AMDB: Registering built-in private dynamic modules success");
   }

   result = amdb_reg_built_in_dynamic_modules();

   if (AR_DID_FAIL(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "AMDB: Registering built-in dynamic modules failed %d", result);
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "AMDB: Registering built-in dynamic modules success");
   }

   AR_MSG(DBG_MED_PRIO, "AMDB: Registering built-in static ");

   result = amdb_reg_built_in_static_modules();

   if (AR_DID_FAIL(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "AMDB: Registering built-in static modules failed %d", result);
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "AMDB: Registering built-in static modules success");
   }

   uint64_t end_time = posal_timer_get_time_in_msec();
   AR_MSG(DBG_HIGH_PRIO, "AMDB: ar_register_all_built_in : time taken %lu ms", (uint32_t)(end_time - start_time));

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------
  IN  : amdb node ptr
  OUT : None

  Cast magic
  --------------------------------------------------------------------------------------------------------------------*/
void *amdb_node_to_handle(amdb_node_t *node_ptr)
{
   return (void *)(node_ptr);
}

/*----------------------------------------------------------------------------------------------------------------------
  IN  : handle
  OUT : None

  Cast magic
  --------------------------------------------------------------------------------------------------------------------*/
amdb_node_t *amdb_handle_to_node(void *handle_ptr)
{
   return (amdb_node_t *)(handle_ptr);
}

/*----------------------------------------------------------------------------------------------------------------------
  Atomically increments the mem ref variable
  --------------------------------------------------------------------------------------------------------------------*/
void amdb_node_inc_mem_ref(amdb_node_t *me)
{
   posal_atomic_increment(&me->mem_refs);
   return;
}

/*----------------------------------------------------------------------------------------------------------------------
  Atomically decrements the mem ref variable
  Frees the memory if the memory count is 0
  --------------------------------------------------------------------------------------------------------------------*/
void amdb_node_dec_mem_ref(amdb_node_t *me)
{
   // Free the memory if no longer needed.
   if (0 == posal_atomic_decrement(&me->mem_refs))
   {
      amdb_node_vtbl[me->flags.interface_type - 1].end(me);
      if (!me->flags.is_static)
      {
         amdb_dynamic_t *dyn = (amdb_dynamic_t *)(((amdb_capi_t *)me) + 1);
         posal_inline_mutex_deinit(&dyn->dl_mutex);
      }
      posal_memory_free(me);
      me = NULL;
   }
   return;
}

/*----------------------------------------------------------------------------------------------------------------------
  Decrements the dl refs and mem refs. Internally these functions may free the node and dlclose based on the values of
  the reference counts
  --------------------------------------------------------------------------------------------------------------------*/
void amdb_node_free(void *void_ptr, spf_hash_node_t *node)
{
   amdb_node_t *me = STD_RECOVER_REC(amdb_node_t, hn, node);
   amdb_node_dec_mem_ref(me);
}

/*----------------------------------------------------------------------------------------------------------------------
 * Initializes amdb node with incoming params.
 * Allocated memory for dynamic part of the amdb node if the node type is not static
 *--------------------------------------------------------------------------------------------------------------------*/
ar_result_t amdb_node_init(amdb_node_t *me,
                           bool_t       is_static,
                           bool_t       is_built_in,
                           uint32_t     type,
                           uint32_t     id1,
                           void        *f1,
                           void        *f2,
                           uint32_t     tag_len,
                           const char  *tag_str,
                           uint32_t     filename_len,
                           const char  *filename_str)
{
   ar_result_t result   = AR_EOK;
   amdb_t     *amdb_ptr = g_amdb_ptr;

   posal_atomic_set(&me->mem_refs, 1);

   me->key               = id1;
   me->hn.key_size       = sizeof(me->key);
   me->hn.key_ptr        = &me->key;
   me->flags.module_type = type;
   me->flags.is_built_in = is_built_in;
   if (is_static)
   {
      me->flags.is_static = TRUE;
      amdb_node_vtbl[me->flags.interface_type - 1].store_static_functions(me, f1, f2);
   }
   else
   {
      result = node_init_dynamic(me, is_built_in, tag_len, tag_str, filename_len, filename_str, amdb_ptr->heap_id);
   }

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------
  Finds the node with given key ptr and size and returns the amdb node
  --------------------------------------------------------------------------------------------------------------------*/
amdb_node_t *amdb_node_hashtable_find(spf_hashtable_t *ht, const void *key_ptr, int key_size)
{
   amdb_node_t *node_ptr = NULL;
   node_ptr              = NULL;

   spf_hash_node_t *phn = spf_hashtable_find(ht, key_ptr, key_size);
   if (!phn)
   {
      return NULL;
   }

   node_ptr = STD_RECOVER_REC(amdb_node_t, hn, phn);
   return node_ptr;
}

/*----------------------------------------------------------------------------------------------------------------------
   Inserts the hash node to the hashtable
  --------------------------------------------------------------------------------------------------------------------*/
ar_result_t amdb_insert_node(amdb_node_t *node_ptr)
{
   uint32_t         result = AR_EOK;
   amdb_t          *me     = g_amdb_ptr;
   spf_hashtable_t *ht     = &me->ht;

   posal_mutex_lock(me->mutex_node);

   result = spf_hashtable_insert(ht, &node_ptr->hn);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "AMDB: Failed to insert node to hashtable, err =  %lu", result);
   }

   posal_mutex_unlock(me->mutex_node);

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------
  Searches the hashtable for the amdb node with matching keys given
  If the node is found, and is valid and:
  If the node is of static type - module info ptr is filled with this node's details
  If the node is of dynamic type - checks if dynamic loading is needed or not.
  --------------------------------------------------------------------------------------------------------------------*/
ar_result_t amdb_get_node(amdb_module_handle_info_t *module_info_ptr, bool_t *to_be_loaded)
{
   amdb_t      *me       = g_amdb_ptr;
   amdb_node_t *node_ptr = NULL;
   int          key      = 0;
   ar_result_t  err      = AR_EOK;
   *to_be_loaded         = FALSE;
   key                   = module_info_ptr->module_id;

   posal_mutex_lock(me->mutex_node);

   node_ptr = amdb_node_hashtable_find(&me->ht, &key, sizeof(key));
   if (node_ptr)
   {
      if (!node_ptr->flags.is_static)
      {
         // Check if dynamic loading is needed.
         ar_result_t local_err = check_dynamic_loading(node_ptr, to_be_loaded);
         if (AR_EOK != local_err)
         {
            node_ptr = NULL;
            posal_mutex_unlock(me->mutex_node);
            THROW(err, AR_EFAILED);
         }
      }

      module_info_ptr->interface_type = node_ptr->flags.interface_type;
      module_info_ptr->module_type    = (uint16_t)node_ptr->flags.module_type;
      module_info_ptr->handle_ptr     = amdb_node_to_handle(node_ptr);
      amdb_node_inc_mem_ref(node_ptr);
   }
   else
   {
      posal_mutex_unlock(me->mutex_node);
      THROW(err, AR_EFAILED);
   }
   posal_mutex_unlock(me->mutex_node);

   AR_MSG(DBG_HIGH_PRIO,
          "AMDB_GET: Found Module (%u, 0x%lX), is_static: %u, to_be_loaded: %u, interface: %d",
          module_info_ptr->module_type,
          module_info_ptr->module_id,
          node_ptr->flags.is_static,
          *to_be_loaded,
          module_info_ptr->interface_type);

   CATCH(err)
   {
      AR_MSG(DBG_ERROR_PRIO, "AMDB_GET: Failed to get module (0x%lX), is not present", module_info_ptr->module_id);
   }
   return err;
}

/*----------------------------------------------------------------------------------------------------------------------
  Searches the hashtable for the amdb node with matching keys given
  If the node is found, return TRUE else FALSE
  --------------------------------------------------------------------------------------------------------------------*/
bool_t amdb_get_details_from_mid(uint32_t module_id, amdb_node_t **node_pptr)
{
   amdb_t      *me       = g_amdb_ptr;
   amdb_node_t *node_ptr = NULL;
   int          key      = 0;
   key                   = module_id;

   posal_mutex_lock(me->mutex_node);

   node_ptr = amdb_node_hashtable_find(&me->ht, &key, sizeof(key));
   if (node_ptr && AMDB_INTERFACE_TYPE_CAPI == node_ptr->flags.interface_type)
   {
      AR_MSG(DBG_HIGH_PRIO, "AMDB_GET: Found Module (0x%lX)", module_id);
      *node_pptr = node_ptr;
      posal_mutex_unlock(me->mutex_node);
      return TRUE;
   }
   else
   {
      posal_mutex_unlock(me->mutex_node);
      return FALSE;
   }
   posal_mutex_unlock(me->mutex_node);
   AR_MSG(DBG_ERROR_PRIO, "AMDB_GET: Failed to get module (0x%lX), is not present", module_id);
   return FALSE;
}

#ifdef AMDB_TEST
/*----------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : result

  Removes all nodes
  --------------------------------------------------------------------------------------------------------------------*/
static ar_result_t amdb_remove_all_nodes(void)
{
   amdb_t          *me = g_amdb_ptr;
   amdb_node_t     *capi_ptr;
   ar_result_t      err = AR_EOK;
   uint32_t         n   = 0;
   spf_hash_node_t *phn = 0;

   posal_mutex_lock(me->mutex_node);
   hashtable *ht = &me->ht;
   if (NULL == ht)
   {
      AR_MSG(DBG_ERROR_PRIO, "adsp amdb: failed to find hashtable");
      posal_mutex_unlock(me->mutex_node);
      THROW(err, AR_EFAILED);
   }

   hashtable_remove_all(ht);
   posal_mutex_unlock(me->mutex_node);

   CATCH(err)
   {
   }
   return err;
}
/*----------------------------------------------------------------------------------------------------------------------
  IN  : None
  OUT : result

  Wrapper function for removing all nodes
  --------------------------------------------------------------------------------------------------------------------*/
ar_result_t amdb_remove_all_capi(void)
{
   return amdb_remove_all_nodes();
}
#endif
