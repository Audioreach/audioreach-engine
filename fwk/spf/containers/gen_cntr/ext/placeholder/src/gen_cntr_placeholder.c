/**
 * \file gen_cntr_placeholder.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"
#include "gen_cntr_utils.h"
#include "apm.h"
#include "cu_events.h"

static ar_result_t gen_cntr_reg_evt_placeholder(gen_cntr_t *       me_ptr,
                                                gen_cntr_module_t *module_ptr,
                                                topo_reg_event_t * event_cfg_payload_ptr,
                                                bool_t             is_register);
static ar_result_t gen_cntr_handle_set_cfg_placeholder_module(
   gen_cntr_t                        *me_ptr,
   gen_cntr_module_t                 *gen_cntr_module_ptr,
   uint32_t                           param_id,
   int8_t                            *param_data_ptr,
   uint32_t                           param_size,
   spf_cfg_data_type_t                cfg_type,
   cu_handle_rest_ctx_for_set_cfg_t **pending_set_cfg_ctx_pptr);

const gen_cntr_fwk_module_vtable_t placeholder_vtable = {
   .set_cfg             = gen_cntr_handle_set_cfg_placeholder_module,
   .reg_evt             = gen_cntr_reg_evt_placeholder,
   .raise_evt           = NULL,
   .raise_ts_disc_event = NULL,
};

ar_result_t gen_cntr_create_placeholder_module(gen_cntr_t *           me_ptr,
                                               gen_topo_module_t *    module_ptr,
                                               gen_topo_graph_init_t *graph_init_ptr)
{

   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   // do nothing else at this point. We will do the capi queries after getting the
   // real module ID
   TRY(result,
       __gpr_cmd_register(module_ptr->gu.module_instance_id,
                          graph_init_ptr->gpr_cb_fn,
                          graph_init_ptr->spf_handle_ptr));

   gen_cntr_module_t *gen_cntr_module_ptr        = (gen_cntr_module_t *)module_ptr;
   gen_cntr_module_ptr->fwk_module_ptr->vtbl_ptr = &placeholder_vtable;

   CATCH(result, GEN_CNTR_MSG_PREFIX, me_ptr->cu.gu_ptr->log_id)
   {
   }

   return result;
}

static ar_result_t gen_cntr_reg_evt_placeholder(gen_cntr_t *       me_ptr,
                                                gen_cntr_module_t *module_ptr,
                                                topo_reg_event_t * event_cfg_payload_ptr,
                                                bool_t             is_register)
{
   return gen_cntr_cache_set_event_prop(me_ptr, module_ptr, event_cfg_payload_ptr, is_register);
}

static inline bool_t gen_cntr_is_placeholder_module_disabled(gen_cntr_module_t *gen_cntr_module_ptr)
{
   return (gen_cntr_module_ptr->fwk_module_ptr && (gen_cntr_module_ptr->topo.flags.disabled));
}

/**
 * can also do this by checking if module_type is framework or not.
 * once real id is set it's no loner framework type module
 */
static inline bool_t gen_cntr_is_real_module_id_set_on_placeholder_module(gen_cntr_module_t *gen_cntr_module_ptr)
{
   return (gen_cntr_module_ptr->fwk_module_ptr && (0 != gen_cntr_module_ptr->fwk_module_ptr->module_id));
}

ar_result_t gen_cntr_placeholder_check_if_real_id_rcvd_at_prepare(cu_base_t *               base_ptr,
                                                                  spf_msg_cmd_graph_mgmt_t *cmd_gmgmt_ptr)
{
   gen_cntr_t *       me_ptr     = (gen_cntr_t *)base_ptr;
   gen_topo_module_t *module_ptr = NULL;

   // if no SG is self prepared, then it's ok even if placeholder is not set.
   if ((0 == cmd_gmgmt_ptr->sg_id_list.num_sub_graph) || (NULL == cmd_gmgmt_ptr->sg_id_list.sg_id_list_ptr))
   {
      return AR_EOK;
   }

   spf_cntr_sub_graph_list_t *spf_sg_list_ptr = &cmd_gmgmt_ptr->sg_id_list;

   for (gu_sg_list_t *sg_list_ptr = base_ptr->gu_ptr->sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      if (!gu_is_sg_id_found_in_spf_array(spf_sg_list_ptr, sg_list_ptr->sg_ptr->id))
      {
         // if SG is not self prepared, then it's ok even if placeholder is not set.
         continue;
      }

      // now, we know that SG ID is in the prepare cmd. Check if placeholder is in the SG. If so, check if it's not
      // disabled and real-module-id is not set.

      for (gu_module_list_t *mod_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != mod_list_ptr);
           LIST_ADVANCE(mod_list_ptr))
      {
         if ((MODULE_ID_PLACEHOLDER_DECODER == mod_list_ptr->module_ptr->module_id) ||
             (MODULE_ID_PLACEHOLDER_ENCODER == mod_list_ptr->module_ptr->module_id))
         {
            module_ptr =
               (gen_topo_module_t *)gu_find_module(&me_ptr->topo.gu, mod_list_ptr->module_ptr->module_instance_id);
            if (NULL == module_ptr)
            {
               GEN_CNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "gen_cntr_prepare: Module not found");
               return AR_EUNSUPPORTED;
            }

            gen_cntr_module_t *gen_cntr_module_ptr = (gen_cntr_module_t *)module_ptr;

            if (!module_ptr->flags.disabled)
            {
               // if enabled, we check if the real ID is received
               if (!gen_cntr_is_real_module_id_set_on_placeholder_module(gen_cntr_module_ptr))
               {
                  GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                               DBG_HIGH_PRIO,
                               "CMD:Prepare Graph: Failing prepare graph command. enabled placeholder module id 0x%lx "
                               "hasn't received real ID, result=0x%lx",
                               mod_list_ptr->module_ptr->module_id,
                               AR_EFAILED);

                  return AR_EFAILED;
               }
            }
         }
      }
   }
   return AR_EOK;
}

/*
   This function is called when the set_cfg is done to the FWK Placeholder module with the REAL PARAM ID.
   That context will create module from amdb, query for static props, initialize, query thresholds etc. It then checks
   the stack req to see if the thread needs to be respun with a new stack size. If so, the new thread will first call
   this function to call set param on the real module with all the cached params, and register for cached events with
   the real module. If not, the same context calls this function.
   Only Regular set is allowed for real-param-id set. Persistent/shared persistent not allowed.


   in thread re-launch case, we need to manually continue with cmd we were processing.
   however, if thread didn't re-launch, then func-return would lead us back to original cmd handling.
  **/

static ar_result_t gen_cntr_handle_real_module_cfg(cu_base_t *base_ptr, void *ctx_ptr, uint32_t real_module_id)
{
   ar_result_t        result                = AR_EOK;
   INIT_EXCEPTION_HANDLING
   gen_cntr_t *       me_ptr                = (gen_cntr_t *)base_ptr;
   gen_topo_module_t *module_ptr            = (gen_topo_module_t *)ctx_ptr;
   gen_topo_t *       topo_ptr              = &me_ptr->topo;
   spf_list_node_t *  client_list_ptr       = NULL;
   spf_list_node_t *  cached_event_list_ptr = NULL;
   gen_cntr_module_t *gen_cntr_module_ptr   = (gen_cntr_module_t *)module_ptr;
   gen_topo_graph_init_t graph_init         = { 0 };

   GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_LOW_PRIO,
                "CMD:SET_GET_CFG: in gen_cntr_handle_real_module_cfg for real_module_id:0x%lx",
                real_module_id);

   graph_init.capi_cb = topo_ptr->capi_cb;
   TRY(result, gen_topo_query_and_create_capi(topo_ptr, &graph_init, module_ptr));

   // store the real id after determining that CAPI is created. hence forth,
   // fwk_module_ptr->module_id could be used to verify if real-module-id was successfully set or not.
   gen_cntr_module_ptr->fwk_module_ptr->module_id = real_module_id;

   /*Do port operations, and get port thresholds for all ports*/
   gen_topo_init_set_get_data_port_properties(module_ptr, topo_ptr, TRUE, &graph_init);

   /**Set input media fmt on the new module if it's valid.
    * In hand-over scenarios, no new MF is going to come, but the module must be made aware of the existing MF*/
   for (gu_input_port_list_t *in_port_list_ptr = module_ptr->gu.input_port_list_ptr; (NULL != in_port_list_ptr);
        LIST_ADVANCE(in_port_list_ptr))
   {
      gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

      if (in_port_ptr->gu.conn_out_port_ptr)
      {
         gen_topo_output_port_t *conn_out_port_ptr = (gen_topo_output_port_t *)in_port_ptr->gu.conn_out_port_ptr;
         if (conn_out_port_ptr->common.flags.is_mf_valid)
         {
            tu_set_media_fmt_from_port(&topo_ptr->mf_utils,
                                       &in_port_ptr->common.media_fmt_ptr,
                                       conn_out_port_ptr->common.media_fmt_ptr);
            in_port_ptr->common.flags.is_mf_valid = TRUE;
            gen_topo_reset_pcm_unpacked_mask(&in_port_ptr->common);
         }
      }
      else if (in_port_ptr->gu.ext_in_port_ptr)
      {
         gen_cntr_ext_in_port_t *ext_in_port_ptr = (gen_cntr_ext_in_port_t *)in_port_ptr->gu.ext_in_port_ptr;

         if (topo_is_valid_media_fmt(&ext_in_port_ptr->cu.media_fmt))
         {
            gen_topo_set_input_port_media_format(topo_ptr, in_port_ptr, &ext_in_port_ptr->cu.media_fmt);
         }
      }

      if (in_port_ptr->common.flags.is_mf_valid)
      {
         gen_topo_capi_set_media_fmt(topo_ptr,
                                     module_ptr,
                                     in_port_ptr->common.media_fmt_ptr,
                                     TRUE /*is_input*/,
                                     in_port_ptr->gu.cmn.index);
      }
   }

   /** in case output MF or other things changed, let's propagate gen_cntr_handle_events_after_set_cfg call inside
    * gen_cntr_handle_events_after_set_cfg takes care.*/

   // we now need to go through the list and set the cached params to the capi module
   for (spf_list_node_t *head_ptr = gen_cntr_module_ptr->fwk_module_ptr->cached_param_list_ptr; (NULL != head_ptr);
        LIST_ADVANCE(head_ptr))
   {
      gen_topo_cached_param_node_t *cfg_node_ptr = (gen_topo_cached_param_node_t *)head_ptr->obj_ptr;
      result |= gen_topo_capi_set_param(topo_ptr->gu.log_id,
                                        module_ptr->capi_ptr,
                                        cfg_node_ptr->param_id,
                                        cfg_node_ptr->payload_ptr,
                                        cfg_node_ptr->param_size);

      // free the cached payload if default
      if (SPF_CFG_DATA_TYPE_DEFAULT == cfg_node_ptr->payload_type)
      {
         MFREE_NULLIFY(cfg_node_ptr->payload_ptr);
      }
   }

   spf_list_delete_list_and_free_objs(&gen_cntr_module_ptr->fwk_module_ptr->cached_param_list_ptr,
                                      TRUE /* pool_used */);

   /* Copying the list to a local variable to avoid infinite loop as gen_topo_set_event_reg_prop_to_capi_modules
    * would add the event to event list upon registration*/
   cached_event_list_ptr                  = gen_cntr_module_ptr->cu.event_list_ptr;
   gen_cntr_module_ptr->cu.event_list_ptr = NULL;

   // we now need to go through the list of events and register the cached event props to the capi module
   for (spf_list_node_t *head_ptr = cached_event_list_ptr; (NULL != head_ptr); LIST_ADVANCE(head_ptr))
   {
      cu_event_info_t *event_node_ptr = (cu_event_info_t *)head_ptr->obj_ptr;

      cu_find_client_info(me_ptr->topo.gu.log_id, event_node_ptr->event_id, cached_event_list_ptr, &client_list_ptr);

      if (NULL == client_list_ptr)
      {
         continue;
      }

      for (cu_client_info_t *client_info_ptr = (cu_client_info_t *)(client_list_ptr->obj_ptr);
           (NULL != client_list_ptr);
           LIST_ADVANCE(client_list_ptr))
      {
         topo_reg_event_t reg_event_payload;
         reg_event_payload.event_id                  = event_node_ptr->event_id;
         reg_event_payload.token                     = client_info_ptr->token;
         reg_event_payload.gpr_heap_index            = GPR_HEAP_INDEX_DEFAULT;
         reg_event_payload.src_port                  = client_info_ptr->src_port;
         reg_event_payload.src_domain_id             = client_info_ptr->src_domain_id;
         reg_event_payload.dest_domain_id            = client_info_ptr->dest_domain_id;
         reg_event_payload.event_cfg.actual_data_len = client_info_ptr->event_cfg.actual_data_len;
         reg_event_payload.event_cfg.data_ptr        = client_info_ptr->event_cfg.data_ptr;
         reg_event_payload.gpr_heap_index            = 0;
         /* sending TRUE for is_register, as for de-register cached event would be deleted
          * from the cached event list */
         bool_t store_client_info = FALSE;
         result |= gen_topo_set_event_reg_prop_to_capi_modules(topo_ptr->gu.log_id,
                                                               module_ptr->capi_ptr,
                                                               module_ptr,
                                                               &reg_event_payload,
                                                               TRUE,
                                                               &store_client_info);

         // free the cached payload
         MFREE_NULLIFY(client_info_ptr->event_cfg.data_ptr);
      }
   }

   cu_delete_all_event_nodes(&cached_event_list_ptr);

   CATCH(result, GEN_CNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   return result;
}

static ar_result_t gen_cntr_handle_rest_of_set_cfg_after_real_module_cfg(cu_base_t *base_ptr, void *ctx_ptr)
{
   ar_result_t        result     = AR_EOK;
   gen_cntr_t *       me_ptr     = (gen_cntr_t *)base_ptr;
   gen_topo_module_t *module_ptr = NULL;

   cu_handle_rest_ctx_for_set_cfg_t *set_cfg_ptr = (cu_handle_rest_ctx_for_set_cfg_t *)ctx_ptr;
   module_ptr                                    = (gen_topo_module_t *)set_cfg_ptr->module_ptr;
   param_id_placeholder_real_module_id_t *id_ptr = (param_id_placeholder_real_module_id_t *)(set_cfg_ptr->param_payload_ptr);

   GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_LOW_PRIO,
                "CMD:SET_GET_CFG: in gen_cntr_handle_rest_of_set_cfg_after_real_module_cfg");

   gen_cntr_handle_real_module_cfg(base_ptr, (void *)module_ptr, id_ptr->real_module_id);

   // ack to GPR or spf_msg is done inside the below handlers.
   switch (me_ptr->cu.cmd_msg.msg_opcode)
   {
      case SPF_MSG_CMD_GPR:
      {
         GEN_CNTR_MSG(me_ptr->topo.gu.log_id, DBG_LOW_PRIO, "CMD:SET_GET_CFG: handling rest of GPR set-cfg");

         gen_cntr_gpr_cmd(&me_ptr->cu);
         break;
      }
      case SPF_MSG_CMD_SET_CFG:
      {
         GEN_CNTR_MSG(me_ptr->topo.gu.log_id, DBG_LOW_PRIO, "CMD:SET_GET_CFG: handling rest of APM set-cfg");

         gen_cntr_set_get_cfg(&me_ptr->cu);
         break;
      }
      case SPF_MSG_CMD_GRAPH_OPEN:
      {
         GEN_CNTR_MSG(me_ptr->topo.gu.log_id, DBG_LOW_PRIO, "CMD:SET_GET_CFG: handling rest of graph open");

         // Handle rest of set cfg
         gen_cntr_handle_rest_of_set_cfgs_in_graph_open(&me_ptr->cu, NULL);
         break;
      }
      default:
      {
      }
   }

   cu_reset_handle_rest(base_ptr);

   return result;
}

static ar_result_t gen_cntr_relaunch_thread_and_handle_rest_of_set_cfg_after_real_module_cfg(cu_base_t *base_ptr,
                                                                                             void      *ctx_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;
   gen_cntr_t *me_ptr = (gen_cntr_t *)base_ptr;

   VERIFY(result, (NULL != ctx_ptr));

   gen_cntr_thread_relaunch_rest_handle_ctx_t *thread_relaunch_open_ctx_ptr =
      (gen_cntr_thread_relaunch_rest_handle_ctx_t *)ctx_ptr;
   bool_t              thread_launched = FALSE;
   posal_thread_prio_t thread_priority = 0;
   char_t              thread_name[POSAL_DEFAULT_NAME_LEN];
   gen_cntr_prepare_to_launch_thread(me_ptr, &thread_priority, thread_name, POSAL_DEFAULT_NAME_LEN);

   TRY(result,
       cu_check_launch_thread(&me_ptr->cu,
                              thread_relaunch_open_ctx_ptr->stack_size,
                              thread_relaunch_open_ctx_ptr->root_stack_size,
                              thread_priority,
                              thread_name,
                              &thread_launched));

   if (thread_launched)
   {
      me_ptr->cu.handle_rest_fn      = gen_cntr_handle_rest_of_set_cfg_after_real_module_cfg;
      me_ptr->cu.handle_rest_ctx_ptr = thread_relaunch_open_ctx_ptr->handle_rest_ctx_ptr;
      return result;
   }

   CATCH(result, GEN_CNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
      switch (me_ptr->cu.cmd_msg.msg_opcode)
      {
         case SPF_MSG_CMD_GPR:
         {
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "GPR command failed due to threadh launch failure.");

            __gpr_cmd_end_command(me_ptr->cu.cmd_msg.payload_ptr, result);
            break;
         }
         case SPF_MSG_CMD_SET_CFG:
         {
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_ERROR_PRIO,
                         "SET-CFG command failed due to threadh launch failure.");

            spf_msg_ack_msg(&me_ptr->cu.cmd_msg, result);
            break;
         }
         case SPF_MSG_CMD_GRAPH_OPEN:
         {
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_ERROR_PRIO,
                         "Graph open command failed due to threadh launch failure.");

             gen_cntr_handle_failure_at_graph_open(me_ptr, result);
             break;
         }
         default:
         {
         }
      }
   }

   return result;
}

static ar_result_t gen_cntr_handle_set_cfg_placeholder_module(
   gen_cntr_t                        *me_ptr,
   gen_cntr_module_t                 *gen_cntr_module_ptr,
   uint32_t                           param_id,
   int8_t                            *param_data_ptr,
   uint32_t                           param_size,
   spf_cfg_data_type_t                cfg_type,
   cu_handle_rest_ctx_for_set_cfg_t **pending_set_cfg_ctx_pptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   gen_topo_module_t *module_ptr = (gen_topo_module_t *)gen_cntr_module_ptr;

   switch (param_id)
   {
      case PARAM_ID_REAL_MODULE_ID:
      {
         spf_list_node_t *     amdb_h_list_ptr = NULL;

         // when we get the real module ID, we need to retrieve the module
         // from amdb, create it, and then set all the cached params to it.
         VERIFY(result, param_size >= sizeof(param_id_placeholder_real_module_id_t));
         param_id_placeholder_real_module_id_t *id_ptr = (param_id_placeholder_real_module_id_t *)(param_data_ptr);

         /*If real id is set to a disabled placeholder, we go ahead a create the module and the disabled"ness" is no
           more honored. Currently, we are not allowing bypassing encoders,decoders,packetizers in GEN_CNTR as well. */
         if (gen_cntr_is_placeholder_module_disabled(gen_cntr_module_ptr))
         {
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_ERROR_PRIO,
                         "Trying to set Real Module ID on a disabled placeholder. "
                         "Please enable it first and set the Real ID.");
            return AR_EFAILED;
         }

         GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_HIGH_PRIO,
                      SPF_LOG_PREFIX "Placeholder 0x%08lX received Real Module ID 0x%lx",
                      module_ptr->gu.module_instance_id,
                      id_ptr->real_module_id);

         TRY(result,
             gu_prepare_for_module_loading(&me_ptr->topo.gu,
                                           &amdb_h_list_ptr,
                                           (gu_module_t *)module_ptr,
                                           TRUE, /*is_placeholder_replaced*/
                                           id_ptr->real_module_id,
                                           me_ptr->cu.heap_id));
         /** AMDB loading or finding module type */
         TRY(result, gu_handle_module_loading(&amdb_h_list_ptr, me_ptr->topo.gu.log_id));

         VERIFY(result, AMDB_MODULE_TYPE_FRAMEWORK != ((gu_module_t *)module_ptr)->module_type);

         VERIFY(result, (AMDB_INTERFACE_TYPE_STUB != module_ptr->gu.itype));

         // Get Capi module's stack size and launch the thead, and then call capi_init()
         uint32_t new_stack_size      = 0;
         uint32_t new_root_stack_size = 0;
         TRY(result, gen_cntr_get_thread_stack_size(me_ptr, &new_stack_size, &new_root_stack_size));

         if (cu_check_thread_relaunch_required(&me_ptr->cu, new_stack_size, new_root_stack_size))
         {
            /*Current context can be in command handling thread therefore need to inform main thread to relaunch the
             * thread and continue the set-cfg*/

            gen_cntr_thread_relaunch_rest_handle_ctx_t *thread_relaunch_set_cfg_ctx_ptr = NULL;
            cu_handle_rest_ctx_for_set_cfg_t           *set_cfg_ptr                     = NULL;

            // context pointer shared with the main thread to relaunch the thread and continue with the set-cfg
            MALLOC_MEMSET(thread_relaunch_set_cfg_ctx_ptr,
                          gen_cntr_thread_relaunch_rest_handle_ctx_t,
                          sizeof(gen_cntr_thread_relaunch_rest_handle_ctx_t),
                          me_ptr->cu.heap_id,
                          result);

            // context pointer used by the main thread to continue with the setc-cfg
            MALLOC_MEMSET(set_cfg_ptr,
                          cu_handle_rest_ctx_for_set_cfg_t,
                          sizeof(cu_handle_rest_ctx_for_set_cfg_t),
                          me_ptr->cu.heap_id,
                          result);

            // set the thread relaunch information
            thread_relaunch_set_cfg_ctx_ptr->stack_size      = new_stack_size;
            thread_relaunch_set_cfg_ctx_ptr->root_stack_size = new_root_stack_size;

            // set the context pointer for the rest handle function after thread relaunch
            thread_relaunch_set_cfg_ctx_ptr->handle_rest_ctx_ptr = set_cfg_ptr;
            set_cfg_ptr->param_payload_ptr                       = param_data_ptr;
            set_cfg_ptr->module_ptr                              = module_ptr;

            if (me_ptr->cu.gu_ptr->data_path_thread_id != posal_thread_get_curr_tid())
            {
               // assign function to handle the thread relaunch in the main thread context and continue with SET-CFG
               // handling
               me_ptr->cu.handle_rest_fn = gen_cntr_relaunch_thread_and_handle_rest_of_set_cfg_after_real_module_cfg;
               me_ptr->cu.handle_rest_ctx_ptr = (void *)thread_relaunch_set_cfg_ctx_ptr;
            }
            else
            {
               result =
                  gen_cntr_relaunch_thread_and_handle_rest_of_set_cfg_after_real_module_cfg(&me_ptr->cu,
                                                                                            thread_relaunch_set_cfg_ctx_ptr);
               MFREE_NULLIFY(thread_relaunch_set_cfg_ctx_ptr);
            }

            if (pending_set_cfg_ctx_pptr)
            {
               *pending_set_cfg_ctx_pptr = set_cfg_ptr;
            }

            return result;
         }

         TRY(result, gen_cntr_handle_real_module_cfg(&me_ptr->cu, (void *)module_ptr, id_ptr->real_module_id));
         break;
      }
      case PARAM_ID_MODULE_ENABLE:
      {
         // note: Module is enabled by default
         VERIFY(result, param_size >= sizeof(param_id_module_enable_t));

         param_id_module_enable_t *enable_ptr = (param_id_module_enable_t *)(param_data_ptr);

         bool_t is_real_module_id_set = gen_cntr_is_real_module_id_set_on_placeholder_module(gen_cntr_module_ptr);
         /**
          * once real module ID is set, set-cfg won't come here as capi_ptr would be present (and if so, fmwk module
          * set-cfg is not called).
          * therefore if we are here, it means real-module-id is not set. just verify that real module-id is not set.
          */
         VERIFY(result, !is_real_module_id_set);

         /**
          * if module was disabled during start and at run time it's enabled
          * without real-module-id being set, then need to error out
          */
         bool_t is_run_time = gen_topo_is_module_sg_started(module_ptr);
         if (module_ptr->flags.disabled && enable_ptr->enable && is_run_time)
         {
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_ERROR_PRIO,
                         "Module 0x%lX: placeholder module cannot enabled at run time without real-module-id being set",
                         module_ptr->gu.module_instance_id);
            return AR_EFAILED;
         }

         module_ptr->flags.disabled = !(enable_ptr->enable);

         /**
          *  if real-ID had been received, then CAPI will raise the process-state and disable itself if necessary
          *   (however, enc/dec cannot be bypassed currently).
          */
         if (module_ptr->flags.disabled)
         {
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_HIGH_PRIO,
                         "Module 0x%lX: placeholder module disabled & bypassed with result 0x%lX",
                         module_ptr->gu.module_instance_id,
                         result);
            // don't call gen_topo_create_bypass here because placeholder will anyway take bypass path.
            // if we create bypass ptr, then it once real-id is set, cleaning up would be messy esp, since we don't
            // support bypass for enc/dec.
         }
         else
         {
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_HIGH_PRIO,
                         "Module 0x%lX: placeholder module enabled with result 0x%lX",
                         module_ptr->gu.module_instance_id,
                         result);
         }

         break;
      }
      case PARAM_ID_RESET_PLACEHOLDER_MODULE:
      {
         if (TOPO_SG_STATE_STOPPED != ((gen_topo_sg_t *)module_ptr->gu.sg_ptr)->state)
         {
            result |= AR_EFAILED;
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_HIGH_PRIO,
                         "Module 0x%lX: placeholder module cannot be reset when SG is not stopped",
                         module_ptr->gu.module_instance_id);
         }
         else
         {
            gen_topo_destroy_module(&me_ptr->topo, module_ptr, TRUE /*reset_capi_dependent_dont_destroy*/);
            gen_cntr_module_ptr->fwk_module_ptr->module_id = 0;
            module_ptr->gu.module_type = AMDB_MODULE_TYPE_FRAMEWORK; // revert back to being fwk module
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_HIGH_PRIO,
                         "Module 0x%lX: placeholder module reset",
                         module_ptr->gu.module_instance_id);
         }
         break;
      }
      default:
      {
         GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_HIGH_PRIO,
                      "Received set cfg for placeholder Module, param ID %lx",
                      param_id);

         // allocate memory for the list node
         gen_topo_cached_param_node_t *cache_cfg_node_ptr =
            (gen_topo_cached_param_node_t *)posal_memory_malloc(sizeof(gen_topo_cached_param_node_t),
                                                                me_ptr->cu.heap_id);
         VERIFY(result, NULL != cache_cfg_node_ptr);
         cache_cfg_node_ptr->param_id     = param_id;
         cache_cfg_node_ptr->param_size   = param_size;
         cache_cfg_node_ptr->payload_type = cfg_type;

         switch (cfg_type)
         {
            case SPF_CFG_DATA_TYPE_DEFAULT:
            {
               // allocate memory for the cached payload
               cache_cfg_node_ptr->payload_ptr =
                  (int8_t *)posal_memory_malloc(cache_cfg_node_ptr->param_size, me_ptr->cu.heap_id);

               VERIFY(result, NULL != cache_cfg_node_ptr->payload_ptr);

               memscpy(cache_cfg_node_ptr->payload_ptr,
                       cache_cfg_node_ptr->param_size,
                       (int8_t *)(param_data_ptr),
                       param_size);
               break;
            }
            case SPF_CFG_DATA_PERSISTENT:
            case SPF_CFG_DATA_SHARED_PERSISTENT:
            {
               cache_cfg_node_ptr->payload_ptr = param_data_ptr;
               break;
            }
            default:
            {
               GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                            DBG_ERROR_PRIO,
                            " Unsupported Payload data type for placeholder cache param: enum = %lu",
                            cfg_type);
               posal_memory_free(cache_cfg_node_ptr);
               return AR_EUNSUPPORTED;
            }
         }
         // before getting the real module ID, we cache any other params that come to the
         // placeholder module in the linked list.
         GEN_CNTR_MSG(me_ptr->topo.gu.log_id, DBG_HIGH_PRIO, "Caching param ID %lx of size %lu", param_id, param_size);

         TRY(result,
             spf_list_insert_tail(&gen_cntr_module_ptr->fwk_module_ptr->cached_param_list_ptr,
                                  cache_cfg_node_ptr,
                                  me_ptr->cu.heap_id,
                                  TRUE /* use_pool*/));
         break;
      }
   }
   CATCH(result, GEN_CNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   return result;
}
