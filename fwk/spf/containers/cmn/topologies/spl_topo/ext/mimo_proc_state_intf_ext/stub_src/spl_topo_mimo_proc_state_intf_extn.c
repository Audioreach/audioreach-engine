/**
 * \file spl_topo_mimo_proc_state_intf_extn.c
 *
 * \brief
 *
 *     stub implementation.
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
                                                                         spl_topo_output_port_t **ou_port_pptr)
{
   if (ip_port_pptr)
   {
      *ip_port_pptr = NULL;
   }
   if (ou_port_pptr)
   {
      *ou_port_pptr = NULL;
   }

   return FALSE;
}

// disable/enable mimo module
ar_result_t spl_topo_intf_extn_mimo_module_process_state_handle_event(spl_topo_t *                      topo_ptr,
                                                                      spl_topo_module_t *               module_ptr,
                                                                      capi_event_data_to_dsp_service_t *event_info_ptr)
{

   return AR_EOK;
}
