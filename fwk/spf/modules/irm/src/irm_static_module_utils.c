/**
@file irm_static_module_utils.cpp

@brief IRM to Static Modules (for profiling) interface utils.

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

#include "irm_api.h"
#include "irm_i.h"

/**
   Why is this an array instead of, say, a dynamically sized hash table?
   We can't be sure IRM will be up at initialization time for all the other services so to avoid any dependency on
   initialization order we can store them here. (this is obviously open for discussion)
*/
static irm_static_module_info_t irm_static_module_info_list[IRM_MAX_ALLOWED_STATIC_SERVICES] = {{0}};

static ar_result_t irm_insert_sm_nodes(irm_t *irm_ptr, spf_list_node_t **head_pptr);
static ar_result_t irm_fill_sm_info(irm_node_obj_t *instance_node_ptr);

/**
 * For the enable-all case insert and fill the handle for all of the static modules
 */
ar_result_t irm_insert_all_static_modules(irm_t *irm_ptr)
{
   ar_result_t      result         = AR_EOK;
   spf_list_node_t *block_list_ptr = irm_ptr->core.block_head_node_ptr;
   for (; NULL != block_list_ptr; block_list_ptr = block_list_ptr->next_ptr)
   {
      irm_node_obj_t *block_obj_ptr = block_list_ptr->obj_ptr;
      if (NULL == block_obj_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "IRM: irm_fill_static_insance_handles: block_obj_ptr is NULL");
         result = AR_EFAILED;
         break;
      }

      if (IRM_BLOCK_ID_STATIC_MODULE == block_obj_ptr->id)
      {
         result = irm_insert_sm_nodes(irm_ptr, &block_obj_ptr->head_node_ptr);
         break; // there can only be 1 entry per block
      }
   }

   return result;
}

static ar_result_t irm_insert_sm_nodes(irm_t *irm_ptr, spf_list_node_t **head_pptr)
{
   for (uint8_t i = 0; i < IRM_MAX_ALLOWED_STATIC_SERVICES; ++i)
   {
      if (0 == irm_static_module_info_list[i].static_miid)
      {
         continue;
      }
      irm_node_obj_t *sm_node = irm_check_insert_node(irm_ptr, head_pptr, irm_static_module_info_list[i].static_miid);
      if (NULL == sm_node)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "IRM: irm_insert_sm_nodes: failed to insert node for id: 0x%x",
                irm_static_module_info_list[i].static_miid);
         return AR_EFAILED;
      }
      sm_node->static_module_info_ptr = &irm_static_module_info_list[i];
      sm_node->heap_id                = irm_static_module_info_list[i].heap_id;
   }
   return AR_EOK;
}

ar_result_t irm_fill_static_instance_info(irm_t *irm_ptr)
{
   ar_result_t result = AR_EOK;

   spf_list_node_t *block_list_ptr = irm_ptr->core.block_head_node_ptr;
   for (; NULL != block_list_ptr; block_list_ptr = block_list_ptr->next_ptr)
   {
      irm_node_obj_t *block_obj_ptr = block_list_ptr->obj_ptr;
      if (NULL == block_obj_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO, "IRM: irm_fill_static_insance_handles: block_obj_ptr is NULL");
         result = AR_EFAILED;
         break;
      }

      if (IRM_BLOCK_ID_STATIC_MODULE == block_obj_ptr->id)
      {
         spf_list_node_t *static_module_list_ptr = block_obj_ptr->head_node_ptr;
         for (; NULL != static_module_list_ptr; static_module_list_ptr = static_module_list_ptr->next_ptr)
         {
            irm_node_obj_t *mod_obj_ptr = static_module_list_ptr->obj_ptr;
            if (NULL == mod_obj_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "IRM: irm_fill_static_insance_handles: mod_obj_ptr is NULL");
               result = AR_EFAILED;
               break;
            }

            result = irm_fill_sm_info(mod_obj_ptr);

            if (result != AR_EOK)
            {
               break;
            }
         }
         break; // there can only be 1 entry per block
      }
   }
   return result;
}

static ar_result_t irm_fill_sm_info(irm_node_obj_t *instance_node_ptr)
{
   for (uint8_t i = 0; i < IRM_MAX_ALLOWED_STATIC_SERVICES; i++)
   {
      if (irm_static_module_info_list[i].static_miid == instance_node_ptr->id)
      {
         instance_node_ptr->static_module_info_ptr = &irm_static_module_info_list[i];
         instance_node_ptr->heap_id                = irm_static_module_info_list[i].heap_id;
         return AR_EOK;
      }
   }
   AR_MSG(DBG_ERROR_PRIO,
          "IRM: irm_fill_sm_info: find matching registered module with id 0x%x.",
          instance_node_ptr->id);
   return AR_EFAILED;
}

ar_result_t irm_register_static_module(uint32_t mid, uint32_t heap_id, int64_t tid)
{
   for (uint8_t i = 0; i < IRM_MAX_ALLOWED_STATIC_SERVICES; i++)
   {
      if (irm_static_module_info_list[i].static_miid == 0)
      {
         irm_static_module_info_list[i].static_miid = mid;
         irm_static_module_info_list[i].heap_id     = heap_id;
         irm_static_module_info_list[i].tid         = tid;
         AR_MSG(DBG_LOW_PRIO,
                "IRM: register_static_module: static module with id 0x%x heap id: 0x%x tid: 0x%x registered with IRM.",
                mid,heap_id, tid);
         return AR_EOK;
      }
      else if (irm_static_module_info_list[i].static_miid == mid)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "IRM: register_static_module: module with id 0x%x already registered with IRM. Bailing out.",
                mid);
         return AR_EFAILED;
      }
   }
   AR_MSG(DBG_ERROR_PRIO,
          "IRM: register_static_module: failed to register module with id 0x%x. No more slots availible",
          mid);
   return AR_EFAILED;
}
