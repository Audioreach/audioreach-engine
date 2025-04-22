/**
 * \file spl_topo_mimo_proc_state_intf_extn.c
 *
 * \brief
 *
 *     functions for managing disable logic for mimo modules.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spl_topo_mimo_proc_state_intf_extn.h"
#include "spl_topo_i.h"

/* =======================================================================
 Static Function Definitions
 ========================================================================== */

/* =======================================================================
 Function Definitions
 ========================================================================== */

// check and return if module can be bypassed for simplified topo.
bool_t spl_topo_intf_extn_mimo_module_process_state_is_module_bypassable(spl_topo_t *             topo_ptr,
                                                                         spl_topo_module_t *      module_ptr,
                                                                         spl_topo_input_port_t ** ip_port_pptr,
                                                                         spl_topo_output_port_t **out_port_pptr)
{
   if (ip_port_pptr)
   {
      *ip_port_pptr = NULL;
   }
   if (out_port_pptr)
   {
      *out_port_pptr = NULL;
   }

   // return false if mimo module is enabled.
   if (!module_ptr->flags.is_mimo_module_disabled)
   {
      return FALSE;
   }

   spl_topo_input_port_t * input_port_ptr      = NULL;
   spl_topo_output_port_t *output_port_ptr     = NULL;
   uint32_t                num_active_in_ports = 0, num_active_out_ports = 0;

   // get the active number of ports.
   for (gu_input_port_list_t *in_list_ptr = module_ptr->t_base.gu.input_port_list_ptr; (NULL != in_list_ptr);
        LIST_ADVANCE(in_list_ptr))
   {
      spl_topo_input_port_t *in_port_ptr = (spl_topo_input_port_t *)in_list_ptr->ip_port_ptr;

      if (!in_port_ptr->flags.port_inactive)
      {
         num_active_in_ports++;
         input_port_ptr = in_port_ptr;
      }
   }

   for (gu_output_port_list_t *out_list_ptr = module_ptr->t_base.gu.output_port_list_ptr; (NULL != out_list_ptr);
        LIST_ADVANCE(out_list_ptr))
   {

      spl_topo_output_port_t *out_port_ptr = (spl_topo_output_port_t *)out_list_ptr->op_port_ptr;

      if (!out_port_ptr->flags.port_inactive)
      {
         num_active_out_ports++;
         output_port_ptr = out_port_ptr;
      }
   }

   // if module wants to disable itself then must be operating in SISO mode
   // Algo delay must be zero to because module will be byapssed only from simplified topo context and will still be
   // called from special topo context. So if module maintains any delay buffer then that can get discontinuous data
   // if fwk switches between simplified and special topo.
   if ((1 != num_active_in_ports) || (1 != num_active_out_ports) || (0 != module_ptr->t_base.algo_delay))
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_ERROR_PRIO,
               "MIMO_DISABLE: can't disable module (0x%lX). active input ports %d, active output ports %d, algo "
               "delay %d. Enabling it.",
               module_ptr->t_base.gu.module_instance_id,
               num_active_in_ports,
               num_active_out_ports,
               module_ptr->t_base.algo_delay);

      return FALSE;
   }

   // must have same valid input/output media format. since we are not maintaining the bypass_ptr therefore different
   // media format can't be handled.
   if (!input_port_ptr->t_base.common.flags.is_mf_valid || !output_port_ptr->t_base.common.flags.is_mf_valid ||
       tu_has_media_format_changed(input_port_ptr->t_base.common.media_fmt_ptr,
                                   output_port_ptr->t_base.common.media_fmt_ptr))
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_ERROR_PRIO,
               "MIMO_DISABLE: can't disable module (0x%lX). either media format is not valid or different media "
               "format on input and output port. Enabling it.");

      return FALSE;
   }

   // return the siso port pointers.
   if(ip_port_pptr)
   {
      *ip_port_pptr = input_port_ptr;
   }

   if(out_port_pptr)
   {
      *out_port_pptr = output_port_ptr;
   }

   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_MED_PRIO,
            "MIMO_DISABLE: module 0x%lx disabled",
            module_ptr->t_base.gu.module_instance_id);

   return TRUE;
}

// disable/enable mimo module
ar_result_t spl_topo_intf_extn_mimo_module_process_state_handle_event(spl_topo_t *                      topo_ptr,
                                                                      spl_topo_module_t *               module_ptr,
                                                                      capi_event_data_to_dsp_service_t *event_info_ptr)
{
   capi_buf_t *                                    payload = &event_info_ptr->payload;
   intf_extn_event_id_mimo_module_process_state_t *data_ptr =
      (intf_extn_event_id_mimo_module_process_state_t *)payload->data_ptr;

   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_HIGH_PRIO,
            "MIMO_DISABLE: Received disable=%lu CB from miid 0x%lX",
            data_ptr->is_disabled,
            module_ptr->t_base.gu.module_instance_id);

   if (module_ptr->flags.is_mimo_module_disabled != data_ptr->is_disabled)
   {
      module_ptr->flags.is_mimo_module_disabled      = data_ptr->is_disabled;

      GEN_TOPO_SET_ONE_CAPI_EVENT_FLAG(&topo_ptr->t_base, process_state);
   }
   return AR_EOK;
}
