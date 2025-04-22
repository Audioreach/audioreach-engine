/**
 * \file gen_topo_ctrl_port_island.c
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

ar_result_t gen_topo_handle_incoming_ctrl_intent(void *   ctx_ptr,
                                                 void *   intent_buf,
                                                 uint32_t max_size,
                                                 uint32_t actual_size)
{
   ar_result_t result = AR_EOK;
   SPF_MANAGE_CRITICAL_SECTION
   gen_topo_ctrl_port_t                    *topo_ctrl_port_ptr = (gen_topo_ctrl_port_t *)ctx_ptr;
   gen_topo_module_t                       *module_ptr         = (gen_topo_module_t *)topo_ctrl_port_ptr->gu.module_ptr;
   intf_extn_param_id_imcl_incoming_data_t *intent_hdr_ptr     = (intf_extn_param_id_imcl_incoming_data_t *)intent_buf;

   if (intent_hdr_ptr->port_id != topo_ctrl_port_ptr->gu.id)
   {
      TOPO_MSG_ISLAND(module_ptr->topo_ptr->gu.log_id,
                      DBG_ERROR_PRIO,
                      "Module: 0x%lx received a control message with ctrl port id: 0x%lx; actual port_id from gu is "
                      "0x%lx",
                      module_ptr->gu.module_instance_id,
                      intent_hdr_ptr->port_id,
                      topo_ctrl_port_ptr->gu.id);
      return AR_EFAILED;
   }

   capi_buf_t buf;
   buf.data_ptr        = (int8_t *)intent_buf;
   buf.actual_data_len = actual_size;
   buf.max_data_len    = max_size;

   capi_port_info_t port_info;
   port_info.is_valid = FALSE;

   TOPO_MSG_ISLAND(module_ptr->topo_ptr->gu.log_id,
                   DBG_LOW_PRIO,
                   "Module: 0x%lx received a control message on the ctrl port id: 0x%lx ",
                   module_ptr->gu.module_instance_id,
                   topo_ctrl_port_ptr->gu.id);

   SPF_CRITICAL_SECTION_START(&module_ptr->topo_ptr->gu);

   result |= module_ptr->capi_ptr->vtbl_ptr->set_param(module_ptr->capi_ptr,
                                                       INTF_EXTN_PARAM_ID_IMCL_INCOMING_DATA,
                                                       &port_info,
                                                       &buf);

   SPF_CRITICAL_SECTION_END(&module_ptr->topo_ptr->gu);

   // return  to buf mgr and make it NULL.
   if (CAPI_FAILED(result))
   {
      TOPO_MSG_ISLAND(module_ptr->topo_ptr->gu.log_id,
                      DBG_ERROR_PRIO,
                      "Sending incoming intent set param to module 0x%x failed result %d",
                      module_ptr->gu.module_instance_id,
                      result);
   }
   else
   {
      TOPO_MSG_ISLAND(module_ptr->topo_ptr->gu.log_id,
                      DBG_LOW_PRIO,
                      "Sending incoming intent set param to module 0x%x is done.",
                      module_ptr->gu.module_instance_id);
   }

   return AR_EOK;
}
