/**
 * \file spl_cntr_sync_fwk_ext.c
 *
 * \brief
 *     Implementation of sync fwk extn in spl container
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear */

/* =======================================================================
Includes
========================================================================== */
#include "spl_cntr_i.h"

/* =======================================================================
Static function declarations
========================================================================== */
static ar_result_t spl_cntr_fwk_extn_sync_aggregate_handle_threshold_disabled(spl_cntr_t *me_ptr);

/* =======================================================================
Function definitions
========================================================================== */

/*
Function to handle threshold disable/enable event raised by the modules
*/
ar_result_t spl_cntr_fwk_extn_sync_handle_toggle_threshold_buffering_event(
   spl_cntr_t *                      me_ptr,
   spl_topo_module_t *               module_ptr,
   capi_buf_t *                      payload_ptr,
   capi_event_data_to_dsp_service_t *dsp_event_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;
   if (dsp_event_ptr->payload.actual_data_len < sizeof(fwk_extn_sync_event_id_enable_threshold_buffering_t))
   {
      SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id,
                   DBG_ERROR_PRIO,
                   "Error in callback function. The actual size %lu is less than the required size "
                   "%lu for id %lu.",
                   payload_ptr->actual_data_len,
                   sizeof(fwk_extn_sync_event_id_enable_threshold_buffering_t),
                   (uint32_t)(dsp_event_ptr->param_id));
      return AR_EBADPARAM;
   }

   fwk_extn_sync_event_id_enable_threshold_buffering_t *data_ptr =
      (fwk_extn_sync_event_id_enable_threshold_buffering_t *)(dsp_event_ptr->payload.data_ptr);

   SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id,
                DBG_MED_PRIO,
                "Handling toggle threshold buffering event, enable: %ld",
                data_ptr->enable_threshold_buffering);

   module_ptr->t_base.flags.is_threshold_disabled = !data_ptr->enable_threshold_buffering;

   //if there is a dm module at the input of SYNC then need to notify them, so that they can start consuming partial data.
   //need to mark external input port if threshold is disabled.
   gen_topo_fwk_extn_sync_propagate_threshold_state(&me_ptr->topo.t_base);

   for (gu_ext_in_port_list_t *ext_in_port_list_ptr = me_ptr->cu.gu_ptr->ext_in_port_list_ptr; ext_in_port_list_ptr;
        LIST_ADVANCE(ext_in_port_list_ptr))
   {
      spl_cntr_ext_in_port_t *ext_in_port_ptr = (spl_cntr_ext_in_port_t *)ext_in_port_list_ptr->ext_in_port_ptr;

      // if external port has threshold disabled then start preserving prebuffers
      if (TRUE == ((gen_topo_input_port_t *)ext_in_port_ptr->gu.int_in_port_ptr)->flags.is_threshold_disabled_prop)
      {
         ext_in_port_ptr->cu.preserve_prebuffer = TRUE;
      }
      // if external port is in the threshold enabled state and in the same path as the sync module
      else if (ext_in_port_ptr->cu.preserve_prebuffer &&
               (module_ptr->t_base.gu.path_index == ext_in_port_ptr->gu.int_in_port_ptr->cmn.module_ptr->path_index))
      {
         // requeue prebuffers from the port where threshold is enabled.
         cu_ext_in_requeue_prebuffers(&me_ptr->cu, &ext_in_port_ptr->gu);
         ext_in_port_ptr->cu.preserve_prebuffer = FALSE;
      }
   }

   TRY(result, spl_cntr_fwk_extn_sync_aggregate_handle_threshold_disabled(me_ptr));

   CATCH(result, SPL_CNTR_MSG_PREFIX, me_ptr->topo.t_base.gu.log_id)
   {
   }

   return result;
}

/**
 * By default, threshold should be enabled. Only disable if any module asks to disable threshold
 * This is currently only supported by sync and smart sync modules.
 */
static ar_result_t spl_cntr_fwk_extn_sync_aggregate_handle_threshold_disabled(spl_cntr_t *me_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result                  = AR_EOK;
   bool_t      new_threshold_disabled  = FALSE;
   bool_t      prev_threshold_disabled = me_ptr->topo.simpt1_flags.threshold_disabled;

   for (gu_sg_list_t *sg_list_ptr = me_ptr->cu.gu_ptr->sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         spl_topo_module_t *module_ptr = (spl_topo_module_t *)module_list_ptr->module_ptr;
         if (FALSE == module_ptr->t_base.flags.need_sync_extn)
         {
            continue;
         }

         /*If one module is disabled, overall thresh is disabled.
           If all are enabled then thresh is enabled */
         new_threshold_disabled = (new_threshold_disabled || module_ptr->t_base.flags.is_threshold_disabled);

         //#if SPL_CNTR_DEBUG_LEVEL >= SPL_CNTR_DEBUG_LEVEL_2
         SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id,
                      DBG_MED_PRIO,
                      "Aggregating threshold disabled, mid %lx : threshold disabled %d, prev_threshold_disabled %ld, "
                      "new_threshold_disabled %ld",
                      module_ptr->t_base.gu.module_instance_id,
                      module_ptr->t_base.flags.is_threshold_disabled,
                      prev_threshold_disabled,
                      new_threshold_disabled);
         //#endif
      }
   }
   // If threshold_disabled state didn't change, there's nothing to do.
   if (new_threshold_disabled != prev_threshold_disabled)
   {
      me_ptr->topo.simpt1_flags.threshold_disabled = new_threshold_disabled;

      if (!me_ptr->topo.simpt1_flags.threshold_disabled)
      {
         // when threshold is enabled, we have to query samples required for processing the next frame
         TRY(result, spl_cntr_query_topo_req_samples(me_ptr));
      }
   }

   CATCH(result, SPL_CNTR_MSG_PREFIX, me_ptr->topo.t_base.gu.log_id)
   {
   }

   return result;
}
