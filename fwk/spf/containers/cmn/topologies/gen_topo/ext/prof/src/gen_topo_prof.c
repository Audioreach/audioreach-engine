/**
 * \file gen_topo_prof.c
 *
 * \brief
 *
 *     Implementation of profiling support.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "irm_cntr_if.h"
#include "irm_api.h"

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static bool_t gen_topo_is_mod_profiling_enabled(gen_topo_module_t *module_ptr)
{
   if ((NULL != module_ptr->prof_info_ptr) && ((TRUE == module_ptr->prof_info_ptr->flags.is_pcycles_enabled) ||
                                               (TRUE == module_ptr->prof_info_ptr->flags.is_pktcnt_enabled)))
   {
      return TRUE;
   }
   return FALSE;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static bool_t gen_topo_is_profiling_enabled(gen_topo_t *topo_ptr)
{
   gu_sg_list_t *sg_list_ptr = topo_ptr->gu.sg_list_ptr;
   while (sg_list_ptr)
   {
      gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr;
      while (module_list_ptr)
      {
         gen_topo_module_t *module_ptr = (gen_topo_module_t *)module_list_ptr->module_ptr;

         if (gen_topo_is_mod_profiling_enabled(module_ptr))
         {
            return TRUE;
         }
         LIST_ADVANCE(module_list_ptr);
      }
      LIST_ADVANCE(sg_list_ptr);
   }

   return FALSE;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static ar_result_t gen_topo_fill_mod_metric_payload(gen_topo_t *                        topo_ptr,
                                                    cntr_param_id_get_prof_info_t *     cntr_param_id_get_prof_info,
                                                    cntr_param_id_mod_metric_info_t *   mod_metric_info_ptr,
                                                    cntr_param_id_mod_metric_payload_t *mod_metric_payload_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   // Get Module ptr from ID
   gen_topo_module_t *module_ptr = (gen_topo_module_t *)gu_find_module(&topo_ptr->gu, mod_metric_info_ptr->instance_id);

   if (NULL == module_ptr)
   {
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_ERROR_PRIO,
               "PROF: Module with IID 0x%X not found ",
               mod_metric_info_ptr->instance_id);
      return AR_EFAILED;
   }

   // Allocate the required memory if needed first, we can free it if no metrics were enabled for some reason
   if (NULL == module_ptr->prof_info_ptr)
   {
      MALLOC_MEMSET(module_ptr->prof_info_ptr,
                    gen_topo_module_prof_info_t,
                    sizeof(gen_topo_module_prof_info_t),
                    cntr_param_id_get_prof_info->heap_id,
                    result);
   }

   // Loop through the incoming metrics, mark enable for required metrics and assign corresponding payload ptrs
   for (uint32_t metric_idx = 0; metric_idx < mod_metric_info_ptr->num_metrics; metric_idx++)
   {
      gen_topo_module_prof_info_t *gen_topo_prof_info_ptr = module_ptr->prof_info_ptr;

      switch (mod_metric_payload_ptr[metric_idx].metric_id)
      {
         case IRM_METRIC_ID_PROCESSOR_CYCLES:
         {
            gen_topo_prof_info_ptr->flags.is_pcycles_enabled = cntr_param_id_get_prof_info->is_enable;
            module_ptr->prof_info_ptr->accum_pcylces = 0;
            mod_metric_payload_ptr[metric_idx].payload_ptr   = (void *)&module_ptr->prof_info_ptr->accum_pcylces;

            break;
         }
         case IRM_METRIC_ID_PACKET_COUNT:
         {
            module_ptr->prof_info_ptr->accum_pktcnt = 0;
            gen_topo_prof_info_ptr->flags.is_pktcnt_enabled = cntr_param_id_get_prof_info->is_enable;
            mod_metric_payload_ptr[metric_idx].payload_ptr  = (void *)&module_ptr->prof_info_ptr->accum_pktcnt;

            break;
         }
         case IRM_METRIC_ID_HEAP_INFO:
         {
            uint32_t log_id = topo_ptr->gu.log_id;
            gen_topo_get_mod_heap_id_and_log_id(&log_id,
                                                &mod_metric_info_ptr->module_heap_id,
                                                module_ptr->serial_num,
                                                topo_ptr->heap_id);
            break;
         }
         default:
         {
            TOPO_MSG(topo_ptr->gu.log_id,
                     DBG_ERROR_PRIO,
                     "PROF: WARNING, unsupported metric ID 0x%X, ignoring",
                     mod_metric_payload_ptr[metric_idx].metric_id);
            break;
         }
      }
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_HIGH_PRIO,
               "PROF: module id = 0x%X, metric id = 0x%X, payload ptr = 0x%X, pcycle enable = %lu, pktcnt enable = %lu",
               mod_metric_info_ptr->instance_id,
               mod_metric_payload_ptr[metric_idx].metric_id,
               mod_metric_payload_ptr[metric_idx].payload_ptr,
               gen_topo_prof_info_ptr->flags.is_pcycles_enabled,
               gen_topo_prof_info_ptr->flags.is_pktcnt_enabled);
   }

   // Only Pcycles and packet counts are supported now, if both are disabled, free the allocated memory.
   if (!gen_topo_is_mod_profiling_enabled(module_ptr))
   {
      MFREE_NULLIFY(module_ptr->prof_info_ptr);
   }

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->gu.log_id)
   {
   }
   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
static ar_result_t gen_topo_update_prof_mutex(gen_topo_t *                   topo_ptr,
                                              cntr_param_id_get_prof_info_t *cntr_param_id_get_prof_info)
{
   ar_result_t result = AR_EOK;
   // If any module profiling is enabled
   if (TRUE == gen_topo_is_profiling_enabled(topo_ptr))
   {
      //.. and mutex is not created
      if (NULL == topo_ptr->gu.prof_mutex)
      {
         result = posal_mutex_create(&topo_ptr->gu.prof_mutex, cntr_param_id_get_prof_info->heap_id);
         if (result != AR_EOK)
         {
            TOPO_MSG(topo_ptr->gu.log_id, DBG_ERROR_PRIO, "PROF: Failed to create prof mutex");
            return result;
         }
      }
   }
   else
   {
      // If all module profiling is disabled after updating with incoming param, destroy the mutex
      if (NULL != topo_ptr->gu.prof_mutex)
      {
         posal_mutex_destroy(&topo_ptr->gu.prof_mutex);
      }
   }
   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
ar_result_t gen_topo_get_prof_info(void *vtopo_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr)
{
   ar_result_t result        = AR_EOK;
   gen_topo_t *topo_ptr      = (gen_topo_t *)vtopo_ptr;
   uint32_t    required_size = sizeof(cntr_param_id_get_prof_info_t);
   uint8_t *   payload_ptr   = (uint8_t *)param_payload_ptr;

   if (required_size > (*param_size_ptr))
   {
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_ERROR_PRIO,
               "PROF: insufficient size, given = %lu, required = %lu ",
               *param_size_ptr,
               required_size);
      return AR_EBADPARAM;
   }

   cntr_param_id_get_prof_info_t *cntr_param_id_get_prof_info = (cntr_param_id_get_prof_info_t *)payload_ptr;
   payload_ptr += sizeof(cntr_param_id_get_prof_info_t);

   TOPO_MSG(topo_ptr->gu.log_id,
            DBG_HIGH_PRIO,
            "PROF: Enable/disable = %lu, num modules = %lu",
            cntr_param_id_get_prof_info->is_enable,
            cntr_param_id_get_prof_info->num_modules);

   // Loop through all the modules in the payload and enable/disable each metrics
   for (uint32_t mod_idx = 0; mod_idx < cntr_param_id_get_prof_info->num_modules; mod_idx++)
   {
      // Sanity checks for payload sizes
      required_size += sizeof(cntr_param_id_mod_metric_info_t);
      if (required_size > (*param_size_ptr))
      {
         TOPO_MSG(topo_ptr->gu.log_id,
                  DBG_ERROR_PRIO,
                  "PROF: insufficient size, given = %lu, required = %lu ",
                  *param_size_ptr,
                  required_size);
         return AR_EBADPARAM;
      }
      cntr_param_id_mod_metric_info_t *mod_metric_info_ptr = (cntr_param_id_mod_metric_info_t *)payload_ptr;
      payload_ptr += sizeof(cntr_param_id_mod_metric_info_t);

      required_size += (mod_metric_info_ptr->num_metrics * sizeof(cntr_param_id_mod_metric_payload_t));
      if (required_size > (*param_size_ptr))
      {
         TOPO_MSG(topo_ptr->gu.log_id,
                  DBG_ERROR_PRIO,
                  "PROF: insufficient size, given = %lu, required = %lu ",
                  *param_size_ptr,
                  required_size);
         return AR_EBADPARAM;
      }

      cntr_param_id_mod_metric_payload_t *mod_metric_payload_ptr = (cntr_param_id_mod_metric_payload_t *)payload_ptr;
      payload_ptr += (mod_metric_info_ptr->num_metrics * sizeof(cntr_param_id_mod_metric_payload_t));

      TOPO_MSG(topo_ptr->gu.log_id, DBG_HIGH_PRIO, "PROF: num_metrics = %lu", mod_metric_info_ptr->num_metrics);

      // Create the prof info memory for the module, update the flags, and update payload with metric ptrs
      result |= gen_topo_fill_mod_metric_payload(topo_ptr,
                                                 cntr_param_id_get_prof_info,
                                                 mod_metric_info_ptr,
                                                 mod_metric_payload_ptr);
   }

   cntr_param_id_get_prof_info->cntr_heap_id = topo_ptr->heap_id;

   // Check if any profiling is enabled in the topo, create mutex if needed, free if not needed
   result |= gen_topo_update_prof_mutex(topo_ptr, cntr_param_id_get_prof_info);
   cntr_param_id_get_prof_info->cntr_mutex_ptr = &topo_ptr->gu.prof_mutex;

   TOPO_MSG(topo_ptr->gu.log_id,
            DBG_HIGH_PRIO,
            "PROF: cntr_mutex_ptr 0x%X",
            cntr_param_id_get_prof_info->cntr_mutex_ptr);

   return result;
}

/*----------------------------------------------------------------------------------------------------------------------

----------------------------------------------------------------------------------------------------------------------*/
void gen_topo_prof_handle_deinit(gen_topo_t *topo_ptr, gen_topo_module_t *module_ptr)
{
   if (NULL == topo_ptr || NULL == module_ptr)
   {
      return;
   }
   if (NULL != module_ptr->prof_info_ptr)
   {
      posal_memory_free(module_ptr->prof_info_ptr);
      module_ptr->prof_info_ptr = NULL;
   }

   // Destroy the mutex when last module with prof enabled is destroyed
   if (FALSE == gen_topo_is_profiling_enabled(topo_ptr))
   {
      if (NULL != topo_ptr->gu.prof_mutex)
      {
         posal_mutex_destroy(&topo_ptr->gu.prof_mutex);
      }
   }
}
