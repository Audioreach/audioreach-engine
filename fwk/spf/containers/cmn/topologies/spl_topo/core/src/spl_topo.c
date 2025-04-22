/**
 * \file adv_topo.c
 * \brief
 *     Main public functions implementation for the advanced topo.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spl_topo_i.h"
#include "gen_topo_ctrl_port.h"
/* =======================================================================
Public Function Definitions
========================================================================== */
/*
 * Table of public functions to be used from CU
 */
// clang-format off
static topo_cu_vtable_t spl_topo_cu_vtable =
{
   .propagate_media_fmt         = spl_topo_propagate_media_fmt,
   .operate_on_modules          = spl_topo_operate_on_modules,
   .operate_on_int_in_port      = spl_topo_operate_on_int_in_port,
   .operate_on_int_out_port     = spl_topo_operate_on_int_out_port,
   .operate_on_int_ctrl_port    = gen_topo_operate_on_int_ctrl_port,

   .get_sg_state                = gen_topo_get_sg_state,

   .ctrl_port_operation         = gen_topo_set_ctrl_port_operation,
   .destroy_all_metadata        = gen_topo_destroy_all_metadata,

   .add_path_delay_info                 = gen_topo_add_path_delay_info,
   .update_path_delays                  = gen_topo_update_path_delays,
   .remove_path_delay_info              = gen_topo_remove_path_delay_info,
   .query_module_delay                  = gen_topo_query_module_delay,

   .propagate_boundary_modules_port_state  =  gen_topo_propagate_boundary_modules_port_state,

   .propagate_port_property             = gen_topo_propagate_port_props,
   .propagate_port_property_forwards    = gen_topo_propagate_port_property_forwards,
   .propagate_port_property_backwards   = gen_topo_propagate_port_property_backwards,

   .get_port_property                   = gen_topo_get_port_property,
   .set_port_property                   = gen_topo_set_port_property,
   .set_param                           = gen_topo_set_param,
   .get_prof_info                       = gen_topo_get_prof_info,

   .rtm_dump_data_port_media_fmt        = gen_topo_rtm_dump_data_port_mf_for_all_ports,
   .check_update_started_sorted_module_list   = gen_topo_check_update_started_sorted_module_list,
   .set_global_sh_mem_msg                     = gen_topo_set_global_sh_mem_msg,
};

static const topo_cu_island_vtable_t spl_topo_cu_island_vtable =
{
   .handle_incoming_ctrl_intent         = gen_topo_handle_incoming_ctrl_intent
};


static gen_topo_vtable_t gen_topo_spl_topo_vtable =
{
   .capi_get_required_fmwk_extensions    = spl_topo_capi_get_required_fmwk_extensions,
   .input_port_is_trigger_present        = spl_topo_int_in_port_is_trigger_present,
   .output_port_is_trigger_present       = spl_topo_out_port_is_trigger_present,
   .input_port_is_trigger_absent         = spl_topo_in_port_is_trigger_absent,
   .output_port_is_trigger_absent        = spl_topo_out_port_is_trigger_absent,
   .output_port_is_size_known            = spl_topo_output_port_is_size_known,
   .get_out_port_data_len                = spl_topo_get_out_port_data_len,
   .is_port_at_nblc_end					 = spl_topo_is_port_at_nblc_end,
   .update_module_info                   = spl_topo_update_module_info,
};
// clang-format on

/* =======================================================================
Public Function Definitions
========================================================================== */

ar_result_t spl_topo_init_topo(spl_topo_t *topo_ptr, gen_topo_init_data_t *init_data_ptr, POSAL_HEAP_ID heap_id)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
   TOPO_MSG(topo_ptr->t_base.gu.log_id, DBG_MED_PRIO, "Topo init begin");
#endif
   TRY(result, gen_topo_init_topo(&topo_ptr->t_base, init_data_ptr, heap_id));
   topo_ptr->t_base.gen_topo_vtable_ptr = &gen_topo_spl_topo_vtable;

   // Initialize all function pointers within topo with spl_topo routines here.
   init_data_ptr->topo_cu_vtbl_ptr        = &spl_topo_cu_vtable;
   init_data_ptr->topo_cu_island_vtbl_ptr = &spl_topo_cu_island_vtable;

   // Initially set to 0, will be set in graph open
   topo_ptr->cntr_frame_len.sample_rate       = 0;
   topo_ptr->cntr_frame_len.frame_len_samples = 0;
   topo_ptr->cntr_frame_len.frame_len_us      = 0;

   // Set all flags to TRUE for first time.
   topo_ptr->simpt_event_flags.word = spl_topo_simp_topo_get_all_event_flag_set();

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id, DBG_ERROR_PRIO, "Error! Topo init failed!");
   }
   return result;
}

/**
 * Topo2-layer deinit necessary when an external port deinits. Note that anything in gu is already handled
 * by gu.
 */
ar_result_t spl_topo_deinit_ext_in_port(spl_topo_t *spl_topo_ptr, gu_ext_in_port_t *ext_in_port_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t            result          = AR_EOK;
   spl_topo_input_port_t *int_in_port_ptr = NULL;

   VERIFY(result, ext_in_port_ptr);

   int_in_port_ptr = (spl_topo_input_port_t *)ext_in_port_ptr->int_in_port_ptr;
   VERIFY(result, int_in_port_ptr);

   gen_topo_destroy_input_port(&spl_topo_ptr->t_base, &int_in_port_ptr->t_base);

   CATCH(result, TOPO_MSG_PREFIX, spl_topo_ptr->t_base.gu.log_id)
   {
   }
   return result;
}

/**
 * Topo2-layer deinit necessary when an external port deinits. Note that anything in gu is already handled
 * by gu.
 */
ar_result_t spl_topo_deinit_ext_out_port(spl_topo_t *spl_topo_ptr, gu_ext_out_port_t *ext_out_port_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t             result           = AR_EOK;
   spl_topo_output_port_t *int_out_port_ptr = NULL;

   VERIFY(result, ext_out_port_ptr);

   int_out_port_ptr = (spl_topo_output_port_t *)ext_out_port_ptr->int_out_port_ptr;
   VERIFY(result, int_out_port_ptr);

   spf_list_delete_list((spf_list_node_t **)&int_out_port_ptr->simp_attached_module_list_ptr, TRUE);

   if (int_out_port_ptr->t_base.gu.attached_module_ptr)
   {
      spl_topo_module_t *     attached_module_ptr = (spl_topo_module_t *)int_out_port_ptr->t_base.gu.attached_module_ptr;
      spl_topo_output_port_t *attached_out_port_ptr =
         (spl_topo_output_port_t *)(attached_module_ptr->t_base.gu.output_port_list_ptr
                                       ? attached_module_ptr->t_base.gu.output_port_list_ptr->op_port_ptr
                                       : NULL);
      VERIFY(result, attached_out_port_ptr);

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
      TOPO_MSG(spl_topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "external output port idx = %ld, miid = 0x%lx is being closed, destroying attached module output port %ld, "
               "miid 0x%lx's internal port structure.",
               ext_out_port_ptr->int_out_port_ptr->cmn.index,
               ext_out_port_ptr->int_out_port_ptr->cmn.module_ptr->module_instance_id,
               attached_out_port_ptr->t_base.gu.cmn.index,
               attached_out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
#endif
      gen_topo_destroy_output_port(&spl_topo_ptr->t_base, &attached_out_port_ptr->t_base);
   }
   else
   {
#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
      TOPO_MSG(spl_topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "external output port idx = %ld, miid = 0x%lx is being closed, destroying internal port structure.",
               ext_out_port_ptr->int_out_port_ptr->cmn.index,
               ext_out_port_ptr->int_out_port_ptr->cmn.module_ptr->module_instance_id);
#endif

      gen_topo_destroy_output_port(&spl_topo_ptr->t_base, &int_out_port_ptr->t_base);
   }

   CATCH(result, TOPO_MSG_PREFIX, spl_topo_ptr->t_base.gu.log_id)
   {
   }
   return result;
}

ar_result_t spl_topo_destroy_topo(spl_topo_t *topo_ptr)
{
// Free up memory associated with process_context etc. here.
// Internally invoke destroy modules with null sg list as well to free
// anything that might be left over.
#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
   TOPO_MSG(topo_ptr->t_base.gu.log_id, DBG_MED_PRIO, "Topo destroy begin");
#endif

   spf_list_delete_list((spf_list_node_t **)&topo_ptr->req_samp_query_start_list_ptr, TRUE);
   spf_list_delete_list((spf_list_node_t **)&topo_ptr->simpt_sorted_module_list_ptr, TRUE);

   spl_topo_fwk_ext_free_dm_req_samples(topo_ptr);
   gen_topo_destroy_topo(&topo_ptr->t_base);

   return AR_EOK;
}

ar_result_t spl_topo_destroy_modules(spl_topo_t *topo_ptr, bool_t b_destroy_all_modules)
{
   gu_sg_list_t *sg_list_ptr = get_gu_ptr_for_current_command_context(&topo_ptr->t_base.gu)->sg_list_ptr;

   for (; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      if (b_destroy_all_modules || GU_STATUS_CLOSING == sg_list_ptr->sg_ptr->gu_status)
      {
         for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
              LIST_ADVANCE(module_list_ptr))
         {
            spl_topo_module_t *module_ptr = (spl_topo_module_t *)module_list_ptr->module_ptr;

            bool_t UNUSED_DATA_WAS_DROPPED = FALSE;

            // return ports buffers before the modules are destroyed.
            spl_topo_module_drop_all_data(topo_ptr, module_ptr, &UNUSED_DATA_WAS_DROPPED);

            for (gu_output_port_list_t *out_port_list_ptr = module_ptr->t_base.gu.output_port_list_ptr;
                 (NULL != out_port_list_ptr);
                 LIST_ADVANCE(out_port_list_ptr))
            {
               spl_topo_output_port_t *out_port_ptr = (spl_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
               spf_list_delete_list((spf_list_node_t **)&out_port_ptr->simp_attached_module_list_ptr, TRUE);
            }

            gen_topo_destroy_module(&topo_ptr->t_base,
                                    &module_ptr->t_base,
                                    FALSE /*reset_capi_dependent_dont_destroy*/);
         }
      }
   }

   return AR_EOK;
}

/**
 * Do a subgraph operation (flush, close, etc) on the modules in the module_list_ptr (this is
 * a list of modules in the self subgraph).
 */
ar_result_t spl_topo_operate_on_modules(void *                     topo_ptr,
                                        uint32_t                   sg_ops,
                                        gu_module_list_t *         module_list_ptr,
                                        spf_cntr_sub_graph_list_t *spf_sg_list_ptr)
{
   SPF_MANAGE_CRITICAL_SECTION
   ar_result_t result       = AR_EOK;
   spl_topo_t *spl_topo_ptr = (spl_topo_t *)topo_ptr;

   // Clear any unconsumed data held at output ports for flush, stop, and reset.
   if ((TOPO_SG_OP_FLUSH | TOPO_SG_OP_STOP) & sg_ops)
   {
      // We don't need to drop upstream data/md since we're looping through all modules.
      for (gu_module_list_t *t_module_list_ptr = module_list_ptr; (NULL != t_module_list_ptr);
           LIST_ADVANCE(t_module_list_ptr))
      {
         spl_topo_module_t *module_ptr = (spl_topo_module_t *)t_module_list_ptr->module_ptr;
         SPF_CRITICAL_SECTION_START(&spl_topo_ptr->t_base.gu);
         for (gu_input_port_list_t *in_port_list_ptr = module_ptr->t_base.gu.input_port_list_ptr; in_port_list_ptr;
              LIST_ADVANCE(in_port_list_ptr))
         {
            spl_topo_input_port_t *in_port_ptr = (spl_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

            spl_topo_reset_input_port(spl_topo_ptr, in_port_ptr);
         }

         for (gu_output_port_list_t *op_port_list_ptr = module_ptr->t_base.gu.output_port_list_ptr; op_port_list_ptr;
              LIST_ADVANCE(op_port_list_ptr))
         {
            spl_topo_output_port_t *out_port_ptr = (spl_topo_output_port_t *)op_port_list_ptr->op_port_ptr;

            spl_topo_reset_output_port(spl_topo_ptr, out_port_ptr);
         }
         SPF_CRITICAL_SECTION_END(&spl_topo_ptr->t_base.gu);
      }
   }

   result = gen_topo_operate_on_modules(topo_ptr, sg_ops, module_list_ptr, spf_sg_list_ptr);
   if (AR_EOK != result)
   {
      return result;
   }

   return result;
}

ar_result_t spl_topo_operate_on_int_in_port(void *                     topo_ptr,
                                            gu_input_port_t *          in_port_ptr,
                                            spf_cntr_sub_graph_list_t *spf_sg_list_ptr,
                                            uint32_t                   sg_ops,
                                            bool_t                     set_port_op)
{
   if ((TOPO_SG_OP_FLUSH | TOPO_SG_OP_STOP) & sg_ops)
   {
      spl_topo_reset_input_port((spl_topo_t *)topo_ptr, (spl_topo_input_port_t *)in_port_ptr);
   }
   else if (TOPO_SG_OP_CLOSE & sg_ops)
   {
      spl_topo_output_port_t *conn_output_port_ptr = (spl_topo_output_port_t *)in_port_ptr->conn_out_port_ptr;

      if (conn_output_port_ptr)
      {
         spf_list_delete_list((spf_list_node_t **)&conn_output_port_ptr->simp_attached_module_list_ptr, TRUE);
      }
   }

   return gen_topo_operate_on_int_in_port(topo_ptr, in_port_ptr, spf_sg_list_ptr, sg_ops, set_port_op);
}

ar_result_t spl_topo_operate_on_int_out_port(void *                     topo_ptr,
                                             gu_output_port_t *         out_port_ptr,
                                             spf_cntr_sub_graph_list_t *spf_sg_list_ptr,
                                             uint32_t                   sg_ops,
                                             bool_t                     set_port_op)
{
   if ((TOPO_SG_OP_FLUSH | TOPO_SG_OP_STOP) & sg_ops)
   {
      spl_topo_reset_output_port((spl_topo_t *)topo_ptr, (spl_topo_output_port_t *)out_port_ptr);
   }
   else if (TOPO_SG_OP_CLOSE & sg_ops)
   {
      spl_topo_output_port_t *output_port_ptr = (spl_topo_output_port_t *)out_port_ptr;
      spf_list_delete_list((spf_list_node_t **)&output_port_ptr->simp_attached_module_list_ptr, TRUE);
   }

   return gen_topo_operate_on_int_out_port(topo_ptr, out_port_ptr, spf_sg_list_ptr, sg_ops, set_port_op);
}

/**
 * Set param for use for internal set params by framework or topo layers. Parses
 * apm_module_param_data struct and passes to gen_topo helper function.
 */
ar_result_t spl_topo_set_param(spl_topo_t *topo_ptr, apm_module_param_data_t *param_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t        result      = AR_EOK;
   uint32_t           mod_inst_id = param_ptr->module_instance_id;
   spl_topo_module_t *module_ptr  = (spl_topo_module_t *)gu_find_module(&topo_ptr->t_base.gu, mod_inst_id);
   int8_t *           payload_ptr = (int8_t *)param_ptr + sizeof(apm_module_param_data_t);
   if (!module_ptr)
   {
      THROW(result, AR_EUNSUPPORTED);
   }

   // Call common util.
   TRY(result,
       gen_topo_capi_set_param(topo_ptr->t_base.gu.log_id,
                               module_ptr->t_base.capi_ptr,
                               param_ptr->param_id,
                               payload_ptr,
                               param_ptr->param_size));

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
      param_ptr->error_code |= result;
   }
   return result;
}

/*
 * Called before a set_cfg is applied. Updates cmd_state so we can determine the context
 * if events are raised during set_cfg.
 */
void spl_topo_set_param_begin(spl_topo_t *topo_ptr)
{
   topo_ptr->cmd_state = TOPO_CMD_STATE_SET_PARAM;
   return;
}

/*
 * Called after a set_cfg is applied. Goes back to default cmd_state and other post set_cfg handling.
 */
void spl_topo_set_param_end(spl_topo_t *topo_ptr)
{
   topo_ptr->cmd_state             = TOPO_CMD_STATE_DEFAULT;
   return;
}

ar_result_t spl_topo_aggregate_hw_acc_proc_delay(spl_topo_t *topo_ptr,
                                                 bool_t      only_aggregate,
                                                 uint32_t *  aggregate_hw_acc_proc_delay_ptr)
{
   return gen_topo_aggregate_hw_acc_proc_delay(&topo_ptr->t_base, only_aggregate, aggregate_hw_acc_proc_delay_ptr);
}
