/**
 * \file capi_priority_sync_event_utils.c
 * \brief
 *     Implementation of utility functions for capi event handling (kpps, bandwidth, any other events, etc).
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_priority_sync_i.h"
#include "module_cmn_api.h"

// TODO(claguna): Check these values
const uint32_t PRIORITY_SYNC_BW   = 1 * 1024 * 1024;
const uint32_t PRIORITY_SYNC_KPPS = 30;

/* =========================================================================
 * Function definitions
 * =========================================================================*/
/**
 * Function to send the toggle threshold change event.
 */
capi_err_t capi_priority_sync_raise_event_toggle_threshold(capi_priority_sync_t *me_ptr, bool_t enable_threshold)
{
   capi_err_t        capi_result = CAPI_EOK;
   capi_event_info_t event_info;
   event_info.port_info.is_valid = FALSE;

   if (NULL == me_ptr->cb_info.event_cb)
   {
      PS_MSG(me_ptr->miid,
             DBG_ERROR_PRIO,
             "capi_priority_sync: capi event callback is not set, unable to raise enable threshold event!");
      return CAPI_EBADPARAM;
   }

   // Package the fwk event within the data_to_dsp capi event.
   capi_event_data_to_dsp_service_t                    evt = { 0 };
   fwk_extn_sync_event_id_enable_threshold_buffering_t event;
   event.enable_threshold_buffering = enable_threshold;

   evt.param_id                = FWK_EXTN_SYNC_EVENT_ID_ENABLE_THRESHOLD_BUFFERING;
   evt.token                   = 0;
   evt.payload.actual_data_len = sizeof(fwk_extn_sync_event_id_enable_threshold_buffering_t);
   evt.payload.data_ptr        = (int8_t *)&event;
   evt.payload.max_data_len    = sizeof(fwk_extn_sync_event_id_enable_threshold_buffering_t);

   event_info.payload.actual_data_len = sizeof(capi_event_data_to_dsp_service_t);
   event_info.payload.data_ptr        = (int8_t *)&evt;
   event_info.payload.max_data_len    = sizeof(capi_event_data_to_dsp_service_t);

   me_ptr->threshold_is_disabled = !enable_threshold;

   PS_MSG(me_ptr->miid,
          DBG_MED_PRIO,
          "capi_priority_sync: raising event for enable threshold, enable = %ld!",
          enable_threshold);

   capi_result = me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_DATA_TO_DSP_SERVICE, &event_info);
   if (CAPI_FAILED(capi_result))
   {
      PS_MSG(me_ptr->miid,
             DBG_ERROR_PRIO,
             "capi_priority_sync: Failed to raise enable threshold event with %lu",
             capi_result);
   }

   // WARNING: this function can be called recursively from the event callback function. so don't update anything in
   // me_ptr here.
   return capi_result;
}

/**
 * Function to raise various events of the priority sync.
 */
capi_err_t capi_priority_sync_raise_event(capi_priority_sync_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr->cb_info.event_cb)
   {
      PS_MSG(me_ptr->miid, DBG_ERROR_PRIO, "capi priority sync:  Event callback is not set. Unable to raise events!");
      CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
      return capi_result;
   }

   capi_result |= capi_cmn_update_kpps_event(&me_ptr->cb_info, PRIORITY_SYNC_KPPS);
   capi_result |= capi_cmn_update_bandwidth_event(&me_ptr->cb_info, 0, PRIORITY_SYNC_BW);
   return capi_result;
}

/**
 * Function to raise trigger policy based on secondary port RT/FTRT property..
 */
capi_err_t capi_priority_sync_handle_tg_policy(capi_priority_sync_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == me_ptr->tg_policy_cb.change_data_trigger_policy_cb_fn)
   {
      PS_MSG(me_ptr->miid, DBG_ERROR_PRIO, "capi priority sync:  callback is not set. Unable to raise trigger policy!");
      CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
      return capi_result;
   }

   // don't update trigger policy when threshold is disabled.
   if (me_ptr->threshold_is_disabled)
   {
      return capi_result;
   }

   bool_t                         IS_PRIMARY_TRUE = TRUE, IS_SECONDARY_FALSE = FALSE;
   capi_priority_sync_tg_policy_t new_policy            = PRIORITY_SYNC_DEFAULT_TRIGGER;
   bool_t                         is_primary_optional   = FALSE;
   bool_t                         is_secondary_optional = FALSE;

   is_primary_optional   = (!capi_priority_sync_is_path_running(me_ptr, IS_PRIMARY_TRUE)) ? TRUE : FALSE;
   is_secondary_optional = (!capi_priority_sync_is_path_running(me_ptr, IS_SECONDARY_FALSE)) ? TRUE : FALSE;

   if (!is_primary_optional && !is_secondary_optional) // both ports are in start and flowing state.
   {
      // If primary input is RT and secondary input is NRT then make secondary input optional.
      if (me_ptr->primary_in_port_info.cmn.prop_state.us_rt && (!me_ptr->secondary_in_port_info.cmn.prop_state.us_rt))
      {
         new_policy = PRIORITY_SYNC_SECONDARY_OPTIONAL;
      }
   }

   is_primary_optional   = (new_policy & PRIORITY_SYNC_PRIMARY_OPTIONAL);
   is_secondary_optional = (new_policy & PRIORITY_SYNC_SECONDARY_OPTIONAL);

   if (PRIORITY_SYNC_DEFAULT_TRIGGER == new_policy)
   {
      // move to default trigger if
      // 1. Both are mandatory and rt
      // 2. both are optional.
      // 3. one is mandatory and other is at-gap

      if (me_ptr->tg_policy != new_policy)
      {
         capi_result = me_ptr->tg_policy_cb.change_data_trigger_policy_cb_fn(me_ptr->tg_policy_cb.context_ptr,
                                                                             NULL,
                                                                             FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY,
                                                                             0,
                                                                             NULL);
      }
   }
   else
   {
      if (me_ptr->tg_policy != new_policy)
      {
         fwk_extn_port_trigger_affinity_t  inp_affinity[PRIORITY_SYNC_MAX_IN_PORTS];
         fwk_extn_port_trigger_affinity_t  out_affinity[PRIORITY_SYNC_MAX_OUT_PORTS];
         fwk_extn_port_nontrigger_policy_t inp_nontgp[PRIORITY_SYNC_MAX_IN_PORTS];
         fwk_extn_port_nontrigger_policy_t out_nontgp[PRIORITY_SYNC_MAX_OUT_PORTS];

         fwk_extn_port_trigger_group_t triggerable_group = { .in_port_grp_affinity_ptr  = &inp_affinity[0],
                                                             .out_port_grp_affinity_ptr = &out_affinity[0] };

         fwk_extn_port_nontrigger_group_t nontriggerable_group = { .in_port_grp_policy_ptr  = &inp_nontgp[0],
                                                                   .out_port_grp_policy_ptr = &out_nontgp[0] };

         for (uint32_t i = 0; i < PRIORITY_SYNC_MAX_IN_PORTS; i++)
         {
            inp_affinity[i] = FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;
            inp_nontgp[i]   = FWK_EXTN_PORT_NON_TRIGGER_INVALID;
         }

         for (uint32_t i = 0; i < PRIORITY_SYNC_MAX_IN_PORTS; i++)
         {
            out_affinity[i] = FWK_EXTN_PORT_TRIGGER_AFFINITY_PRESENT;
            out_nontgp[i]   = FWK_EXTN_PORT_NON_TRIGGER_INVALID;
         }

         if (is_secondary_optional && (DATA_PORT_STATE_CLOSED != me_ptr->secondary_in_port_info.cmn.state))
         {
            inp_affinity[me_ptr->secondary_in_port_info.cmn.index] = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
            inp_nontgp[me_ptr->secondary_in_port_info.cmn.index]   = FWK_EXTN_PORT_NON_TRIGGER_OPTIONAL;
         }
         if (is_secondary_optional && (DATA_PORT_STATE_CLOSED != me_ptr->secondary_out_port_info.cmn.state))
         {
            out_affinity[me_ptr->secondary_out_port_info.cmn.index] = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
            out_nontgp[me_ptr->secondary_out_port_info.cmn.index]   = FWK_EXTN_PORT_NON_TRIGGER_OPTIONAL;
         }

         if (is_primary_optional && (DATA_PORT_STATE_CLOSED != me_ptr->primary_in_port_info.cmn.state))
         {
            inp_affinity[me_ptr->primary_in_port_info.cmn.index] = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
            inp_nontgp[me_ptr->primary_in_port_info.cmn.index]   = FWK_EXTN_PORT_NON_TRIGGER_OPTIONAL;
         }
         if (is_primary_optional && (DATA_PORT_STATE_CLOSED != me_ptr->primary_out_port_info.cmn.state))
         {
            out_affinity[me_ptr->primary_out_port_info.cmn.index] = FWK_EXTN_PORT_TRIGGER_AFFINITY_NONE;
            out_nontgp[me_ptr->primary_out_port_info.cmn.index]   = FWK_EXTN_PORT_NON_TRIGGER_OPTIONAL;
         }

         capi_result = me_ptr->tg_policy_cb.change_data_trigger_policy_cb_fn(me_ptr->tg_policy_cb.context_ptr,
                                                                             &nontriggerable_group,
                                                                             FWK_EXTN_PORT_TRIGGER_POLICY_MANDATORY,
                                                                             1,
                                                                             &triggerable_group);
      }
   }

   if (CAPI_SUCCEEDED(capi_result))
   {
      PS_MSG(me_ptr->miid,
             DBG_HIGH_PRIO,
             "capi priority sync:  trigger policy updated, is_primary_optional %lu, is_secondary_optional %lu",
             is_primary_optional,
             is_secondary_optional);
      me_ptr->tg_policy = new_policy;
   }

   return capi_result;
}
