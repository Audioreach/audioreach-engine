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

//#define  AMDB_PRIV_DEBUG

extern const uint32_t amdb_num_dynamic_private_capi_modules;
extern const amdb_dynamic_capi_module_t amdb_private_dynamic_capi_modules[];

extern uint32_t amdb_num_static_private_capi_modules;
extern const amdb_static_capi_module_t amdb_private_static_capi_modules[];

extern ar_result_t amdb_register_built_in(uint32_t    module_type,
                                   uint32_t    module_id,
                                   void *      get_static_properties_f,
                                   void *      init_f,
                                   const char *filename_str,
                                   const char *tag_str);


/*----------------------------------------------------------------------------------------------------------------------
 * DESCRIPTION:
 *
 *--------------------------------------------------------------------------------------------------------------------*/
ar_result_t amdb_reg_built_in_private_dynamic_modules()
{
   ar_result_t result = AR_EOK;

   uint64_t start_time = posal_timer_get_time_in_msec();

   AR_MSG(DBG_HIGH_PRIO, "AMDB: adding dynamic built in modules to AMDB");
   // Even though module are separately listed in this file, there's no repetition from CAPI to CAPI V2 list.
   // add modules to the AMDB
   uint32_t num_capi_added      = 0;
   uint32_t num_capi_add_failed = 0;

   for (uint32_t i = 0; i < amdb_num_dynamic_private_capi_modules; i++)
   {
      result = amdb_register_built_in(amdb_private_dynamic_capi_modules[i].mtype,
                                      amdb_private_dynamic_capi_modules[i].mid,
                                      NULL,
                                      NULL,
                                      amdb_private_dynamic_capi_modules[i].filename,
                                      amdb_private_dynamic_capi_modules[i].tag);

      if (AR_DID_FAIL(result))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "AMDB: Failed to add dynamic CAPI V2 module %lX, 0x%lX. result %d. ",
                amdb_private_dynamic_capi_modules[i].mtype,
                amdb_private_dynamic_capi_modules[i].mid,
                result);
         num_capi_add_failed++;
      }
      else
      {
         num_capi_added++;
      }
   }
   AR_MSG(DBG_HIGH_PRIO,
          "AMDB: Registering built-in CAPI V2 dynamic modules. Num added %lu, Num Failed %lu, Total %lu.",
          num_capi_added,
          num_capi_add_failed,
          amdb_num_dynamic_private_capi_modules);
   uint64_t end_time = posal_timer_get_time_in_msec();
   AR_MSG(DBG_HIGH_PRIO,
          "AMDB: amdb_reg_built_in_private_dynamic_modules done adding dynamic modules to AMDB. Time taken %lu ms",
          (uint32_t)(end_time - start_time));

   return result;
}

/* Checks if a private module can be loaded with fallback static code on a dynamic loading failure.*/
ar_result_t amdb_internal_handle_dl_open_failure(amdb_module_handle_info_t *module_handle_info,
                                                 amdb_node_t *              node_ptr)
{
   if(!module_handle_info || !node_ptr)
   {
      return AR_EFAILED;
   }
   uint32_t module_id = module_handle_info->module_id;
   if (AMDB_INTERFACE_TYPE_CAPI != node_ptr->flags.interface_type)
   {
#ifdef AMDB_PRIV_DEBUG
      AR_MSG(DBG_HIGH_PRIO,
             "AMDB: internal_fallback_static_modules Module 0x%lx invalid intf type 0x%lx",
             module_id,
             node_ptr->flags.interface_type);
#endif
      return AR_EFAILED;
   }

#ifdef AMDB_PRIV_DEBUG
   AR_MSG(DBG_HIGH_PRIO,
          "AMDB: internal_fallback_static_modules Num modules %lu, searching for 0x%lx",
          amdb_num_static_private_capi_modules,
          module_id);
#endif

   uint32_t idx = 0;
   for (; idx < amdb_num_static_private_capi_modules; idx++)
   {
      if (module_id == amdb_private_static_capi_modules[idx].mid)
      {
         break;
      }
   }

   if (idx == amdb_num_static_private_capi_modules)
   {
#ifdef AMDB_PRIV_DEBUG
      AR_MSG(DBG_HIGH_PRIO, "AMDB: internal_fallback_static_modules Module 0x%lx not found", module_id);
#endif
      return AR_EFAILED;
   }

   amdb_node_vtbl[node_ptr->flags.interface_type - 1]
      .store_static_functions(node_ptr,
                              amdb_private_static_capi_modules[idx].get_static_prop_fn,
                              amdb_private_static_capi_modules[idx].init_fn);
#ifdef AMDB_PRIV_DEBUG
   AR_MSG(DBG_HIGH_PRIO,
          "AMDB: internal_fallback_static_modules Fallback to static successful for Module 0x%lx",
          module_id);
#endif
   return AR_EOK;
}
