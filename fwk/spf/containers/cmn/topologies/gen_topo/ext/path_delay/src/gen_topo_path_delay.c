/**
 * \file gen_topo_path_delay.c
 *
 * \brief
 *
 *     Implementation of path delay aspects of topology interface functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "gen_topo_capi.h"

typedef enum gen_topo_path_delay_op_t
{
   PATH_DELAY_OP_REMOVE = 1,
   PATH_DELAY_OP_UPDATE = 2,
} gen_topo_path_delay_op_t;

ar_result_t gen_topo_add_path_delay_info(void *   v_topo_ptr,
                                         void *   v_module_ptr,
                                         uint32_t port_id,
                                         void *   v_gu_cmn_port_ptr,
                                         uint32_t path_id)
{
   ar_result_t             result       = AR_EOK;
   gen_topo_t *            topo_ptr     = (gen_topo_t *)v_topo_ptr;
   gen_topo_common_port_t *cmn_port_ptr = NULL;

   if (gu_is_output_port_id(port_id))
   {
      gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)v_gu_cmn_port_ptr;
      cmn_port_ptr                         = &out_port_ptr->common;
   }
   else
   {
      gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)v_gu_cmn_port_ptr;
      cmn_port_ptr                       = &in_port_ptr->common;
   }

   gen_topo_module_t *module_ptr = (gen_topo_module_t *)v_module_ptr;
   INIT_EXCEPTION_HANDLING

   gen_topo_delay_info_t *delay_info_ptr = NULL;
   MALLOC_MEMSET(delay_info_ptr, gen_topo_delay_info_t, sizeof(gen_topo_delay_info_t), topo_ptr->heap_id, result);

   delay_info_ptr->path_id = path_id;
   spf_list_insert_tail(&cmn_port_ptr->delay_list_ptr, delay_info_ptr, topo_ptr->heap_id, TRUE /* use pool*/);

#ifdef PATH_DELAY_DEBUGGING
   TOPO_MSG(topo_ptr->gu.log_id,
            DBG_LOW_PRIO,
            "PATH_DELAY: path-id 0x%lX, Created delay info for module 0x%08lX, port-id %lX ",
            path_id,
            module_ptr->gu.module_instance_id,
            port_id);
#endif

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->gu.log_id)
   {
      TOPO_MSG(topo_ptr->gu.log_id, DBG_ERROR_PRIO, "PATH_DELAY: Error! creating topo delay info");
   }

   return result;
}

static spf_list_node_t *gen_topo_find_path_delay_info_of_port(gen_topo_t *            topo_ptr,
                                                              uint32_t                path_id,
                                                              gen_topo_common_port_t *cmn_port_ptr)
{
   spf_list_node_t *node_ptr = NULL;
   for (node_ptr = cmn_port_ptr->delay_list_ptr; (NULL != node_ptr); LIST_ADVANCE(node_ptr))
   {
      gen_topo_delay_info_t *delay_info_ptr = (gen_topo_delay_info_t *)node_ptr->obj_ptr;
      if (path_id == delay_info_ptr->path_id)
      {
         break;
      }
   }
   return node_ptr;
}

ar_result_t gen_topo_destroy_all_delay_path_per_port(gen_topo_t *            topo_ptr,
                                                     gen_topo_module_t *     module_ptr,
                                                     gen_topo_common_port_t *cmn_port_ptr)
{
   if (cmn_port_ptr->delay_list_ptr)
   {
#ifdef PATH_DELAY_DEBUGGING
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_LOW_PRIO,
               "PATH_DELAY: Destroying all delay info for module 0x%08lX",
               module_ptr->gu.module_instance_id);
#endif
      spf_list_delete_list_and_free_objs(&cmn_port_ptr->delay_list_ptr, TRUE /* pool-used*/);
   }

   return AR_EOK;
}

/**
 * ext_out_port_pptr = the ext out port which is on the path-id
 *
 * topo_ptr, module_ptr, in_port_ptr mandatory
 * path_id can be zero if all paths have to be operated on.
 * PATH_DELAY_OP_REMOVE - out_port_ptr not mandatory.
 * PATH_DELAY_OP_UPDATE - aggregated_delay_ptr, ext_out_port_pptr mandatory
 */
static ar_result_t gen_topo_operate_on_delay_path_per_port(gen_topo_t *             topo_ptr,
                                                           gen_topo_module_t *      module_ptr,
                                                           uint32_t                 path_id,
                                                           gen_topo_input_port_t *  in_port_ptr,
                                                           gen_topo_output_port_t * out_port_ptr,
                                                           gen_topo_path_delay_op_t op,
                                                           uint32_t *               aggregated_algo_delay_ptr,
                                                           uint32_t *               aggregated_ext_in_delay_ptr,
                                                           uint32_t *               aggregated_ext_out_delay_ptr)
{
   ar_result_t result = AR_EOK;
   switch (op)
   {
      case PATH_DELAY_OP_REMOVE:
      {
         if (0 == path_id)
         {
            if (in_port_ptr)
            {
               gen_topo_destroy_all_delay_path_per_port(topo_ptr,
                                                        (gen_topo_module_t *)in_port_ptr->gu.cmn.module_ptr,
                                                        &in_port_ptr->common);
            }
            if (out_port_ptr)
            {
               gen_topo_destroy_all_delay_path_per_port(topo_ptr,
                                                        (gen_topo_module_t *)out_port_ptr->gu.cmn.module_ptr,
                                                        &out_port_ptr->common);
            }
         }
         else
         {
            if (in_port_ptr)
            {
               spf_list_node_t *node_ptr =
                  gen_topo_find_path_delay_info_of_port(topo_ptr, path_id, &in_port_ptr->common);
               if (node_ptr)
               {
#ifdef PATH_DELAY_DEBUGGING
                  TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_LOW_PRIO,
                           "PATH_DELAY: path-id 0x%lX, Destroying delay info for module 0x%08lX, port-id %lX",
                           path_id,
                           module_ptr->gu.module_instance_id,
                           in_port_ptr->gu.cmn.id);
#endif
                  spf_list_delete_node_and_free_obj(&node_ptr,
                                                    &in_port_ptr->common.delay_list_ptr,
                                                    TRUE /* pool-used*/);
               }
            }

            if (out_port_ptr)
            {
               spf_list_node_t *node_ptr =
                  gen_topo_find_path_delay_info_of_port(topo_ptr, path_id, &out_port_ptr->common);

               if (node_ptr)
               {
#ifdef PATH_DELAY_DEBUGGING
                  TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_LOW_PRIO,
                           "PATH_DELAY: path-id 0x%lX, Destroying delay info for module 0x%08lX, port-id %lX",
                           path_id,
                           module_ptr->gu.module_instance_id,
                           out_port_ptr->gu.cmn.id);
#endif
                  spf_list_delete_node_and_free_obj(&node_ptr,
                                                    &out_port_ptr->common.delay_list_ptr,
                                                    TRUE /* pool-used*/);
               }
            }
         }

         break;
      }
      case PATH_DELAY_OP_UPDATE:
      {
         if (0 == path_id)
         {
            TOPO_MSG(topo_ptr->gu.log_id, DBG_ERROR_PRIO, "PATH_DELAY: Path ID must not be zero");
            return AR_EFAILED;
         }

         // either port may be not exist (source/sink) or node may not belong to the path.
         gu_ext_in_port_t * gu_ext_in_port_ptr  = NULL;
         gu_ext_out_port_t *gu_ext_out_port_ptr = NULL;
         spf_list_node_t *  in_node_ptr         = NULL;
         spf_list_node_t *  out_node_ptr        = NULL;
         uint32_t           in_port_id = 0, out_port_id = 0;
         uint32_t           algo_delay = 0, ext_in_delay = 0, ext_out_delay = 0;
         if (out_port_ptr)
         {
            out_node_ptr = gen_topo_find_path_delay_info_of_port(topo_ptr, path_id, &out_port_ptr->common);
            gu_get_ext_out_port_for_last_module(&(topo_ptr->gu), &(out_port_ptr->gu.cmn), &gu_ext_out_port_ptr);
         }

         if (in_port_ptr)
         {
            in_node_ptr        = gen_topo_find_path_delay_info_of_port(topo_ptr, path_id, &in_port_ptr->common);
            gu_ext_in_port_ptr = in_port_ptr->gu.ext_in_port_ptr;
         }

         // if both ports exist or both nodes belong to path
         if (in_node_ptr && out_node_ptr)
         {
            gen_topo_query_module_delay(topo_ptr,
                                        (void *)in_port_ptr->gu.cmn.module_ptr,
                                        &in_port_ptr->common,
                                        &out_port_ptr->common,
                                        &algo_delay);
            in_port_id  = in_port_ptr->gu.cmn.id;
            out_port_id = out_port_ptr->gu.cmn.id;
            if (gu_ext_out_port_ptr)
            {
               ext_out_delay = topo_ptr->topo_to_cntr_vtable_ptr->aggregate_ext_out_port_delay(topo_ptr, gu_ext_out_port_ptr);
            }
            if (gu_ext_in_port_ptr)
            {
               ext_in_delay = topo_ptr->topo_to_cntr_vtable_ptr->aggregate_ext_in_port_delay(topo_ptr, gu_ext_in_port_ptr);
            }
         }
         // only input port exists or for sink modules and only input node belongs to path,
         else if (in_node_ptr)
         {
            algo_delay = module_ptr->algo_delay;
            in_port_id = in_port_ptr->gu.cmn.id;
            if (gu_ext_in_port_ptr)
            {
               ext_in_delay = topo_ptr->topo_to_cntr_vtable_ptr->aggregate_ext_in_port_delay(topo_ptr, gu_ext_in_port_ptr);
            }
         }
         // only output port exists or source modules and only output node belongs to path,
         else
         {
            algo_delay  = module_ptr->algo_delay;
            out_port_id = out_port_ptr ? out_port_ptr->gu.cmn.id : 0;
            if (gu_ext_out_port_ptr)
            {
               ext_out_delay = topo_ptr->topo_to_cntr_vtable_ptr->aggregate_ext_out_port_delay(topo_ptr, gu_ext_out_port_ptr);
            }
         }

         *aggregated_algo_delay_ptr    = *aggregated_algo_delay_ptr + algo_delay;
         *aggregated_ext_in_delay_ptr  = *aggregated_ext_in_delay_ptr + ext_in_delay;
         *aggregated_ext_out_delay_ptr = *aggregated_ext_out_delay_ptr + ext_out_delay;

#ifdef PATH_DELAY_DEBUGGING
         TOPO_MSG(topo_ptr->gu.log_id,
                  DBG_LOW_PRIO,
                  "PATH_DELAY: path-id 0x%lX, Aggregating delay for module 0x%08lX, in-port-id %lX, out-port-id "
                  "%lX,  module contribution: algo_delay %lu us, ext_in_delay %lu us, ext_out_delay %lu us",
                  path_id,
                  module_ptr->gu.module_instance_id,
                  in_port_id,
                  out_port_id,
                  algo_delay,
                  ext_in_delay,
                  ext_out_delay);
         TOPO_MSG(topo_ptr->gu.log_id,
                  DBG_LOW_PRIO,
                  "PATH_DELAY: path-id 0x%lX, Totals; algo_delay %lu us, ext_in_delay %lu us, ext_out_delay %lu us",
                  path_id,
                  *aggregated_algo_delay_ptr,
                  *aggregated_ext_in_delay_ptr,
                  *aggregated_ext_out_delay_ptr);
#endif

         break;
      }
      default:
      {
         TOPO_MSG(topo_ptr->gu.log_id,
                  DBG_ERROR_PRIO,
                  "PATH_DELAY: path-id 0x%lX, Invalid path delay operation %lu module 0x%08lX, port-id %lX",
                  path_id,
                  op,
                  module_ptr->gu.module_instance_id,
                  in_port_ptr->gu.cmn.id);
      }
   }

   return result;
}

static ar_result_t gen_topo_operate_on_delay_path(gen_topo_t *             topo_ptr,
                                                  uint32_t                 path_id,
                                                  gen_topo_path_delay_op_t op,
                                                  uint32_t *               aggregated_algo_delay_ptr,
                                                  uint32_t *               aggregated_ext_in_delay_ptr,
                                                  uint32_t *               aggregated_ext_out_delay_ptr)
{
   ar_result_t result = AR_EOK;

   for (gu_sg_list_t *sg_list_ptr = topo_ptr->gu.sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         gen_topo_module_t *module_ptr = (gen_topo_module_t *)module_list_ptr->module_ptr;

         if (module_ptr->gu.input_port_list_ptr)
         {
            for (gu_input_port_list_t *in_port_list_ptr = module_ptr->gu.input_port_list_ptr;
                 (NULL != in_port_list_ptr);
                 LIST_ADVANCE(in_port_list_ptr))
            {
               gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

               // if input as well as output exist
               if (module_ptr->gu.output_port_list_ptr)
               {
                  for (gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr;
                       (NULL != out_port_list_ptr);
                       LIST_ADVANCE(out_port_list_ptr))
                  {
                     gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
                     result |= gen_topo_operate_on_delay_path_per_port(topo_ptr,
                                                                       module_ptr,
                                                                       path_id,
                                                                       in_port_ptr,
                                                                       out_port_ptr,
                                                                       op,
                                                                       aggregated_algo_delay_ptr,
                                                                       aggregated_ext_in_delay_ptr,
                                                                       aggregated_ext_out_delay_ptr);
                  }
               }
               // if only input exists
               else
               {
                  result |= gen_topo_operate_on_delay_path_per_port(topo_ptr,
                                                                    module_ptr,
                                                                    path_id,
                                                                    in_port_ptr,
                                                                    NULL,
                                                                    op,
                                                                    aggregated_algo_delay_ptr,
                                                                    aggregated_ext_in_delay_ptr,
                                                                    aggregated_ext_out_delay_ptr);
               }
            }
         }
         // if only output exists
         else if (module_ptr->gu.output_port_list_ptr)
         {
            for (gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr;
                 (NULL != out_port_list_ptr);
                 LIST_ADVANCE(out_port_list_ptr))
            {
               gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
               result |= gen_topo_operate_on_delay_path_per_port(topo_ptr,
                                                                 module_ptr,
                                                                 path_id,
                                                                 NULL,
                                                                 out_port_ptr,
                                                                 op,
                                                                 aggregated_algo_delay_ptr,
                                                                 aggregated_ext_in_delay_ptr,
                                                                 aggregated_ext_out_delay_ptr);
            }
         }
      }
   }
   return result;
}

/**
 * delay in the path is aggregated & stored in aggregated_delay_ptr
 */
ar_result_t gen_topo_update_path_delays(void *    v_topo_ptr,
                                        uint32_t  path_id,
                                        uint32_t *aggregated_algo_delay_ptr,
                                        uint32_t *aggregated_ext_in_delay_ptr,
                                        uint32_t *aggregated_ext_out_delay_ptr)
{
   ar_result_t result    = AR_EOK;
   gen_topo_t *topo_ptr  = (gen_topo_t *)v_topo_ptr;
   result                = gen_topo_operate_on_delay_path(topo_ptr,
                                           path_id,
                                           PATH_DELAY_OP_UPDATE,
                                           aggregated_algo_delay_ptr,
                                           aggregated_ext_in_delay_ptr,
                                           aggregated_ext_out_delay_ptr);

   return result;
}

ar_result_t gen_topo_remove_path_delay_info(void *v_topo_ptr, uint32_t path_id)
{
   ar_result_t result   = AR_EOK;
   gen_topo_t *topo_ptr = (gen_topo_t *)v_topo_ptr;
   result               = gen_topo_operate_on_delay_path(topo_ptr, path_id, PATH_DELAY_OP_REMOVE, NULL, NULL, NULL);
   return result;
}

/**
 * for source sink modules in or out can be NULL
 */
ar_result_t gen_topo_query_module_delay(void *    v_topo_ptr,
                                        void *    v_module_ptr,
                                        void *    v_cmn_in_port_ptr,
                                        void *    v_cmn_out_port_ptr,
                                        uint32_t *delay_us_ptr)
{
   ar_result_t result = AR_EOK;
   // gen_topo_t *            topo_ptr     = (gen_topo_t *)v_topo_ptr;
   // gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)v_cmn_out_port_ptr;
   // gen_topo_input_port_t * in_port_ptr  = (gen_topo_input_port_t *)v_cmn_in_port_ptr;
   gen_topo_module_t *module_ptr = (gen_topo_module_t *)v_module_ptr;

   // TODO: to implement for MIMO modules.
   // right now use module's delay value
   *delay_us_ptr = module_ptr->algo_delay;

   return result;
}
