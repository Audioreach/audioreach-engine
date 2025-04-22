/**
 * \file cu_utils_island.c
 *
 * \brief
 *
 *     CU's utility
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "container_utils.h"
#include "cu_i.h"
#include "capi_intf_extn_imcl.h"


ar_result_t cu_handle_event_from_dsp_service_topo_cb(cu_base_t *        cu_ptr,
                                                     gu_module_t *      module_ptr,
                                                     capi_event_info_t *event_info_ptr)
{
   ar_result_t                             result  = AR_EOK;
   capi_buf_t *                            payload = &event_info_ptr->payload;
   capi_event_get_data_from_dsp_service_t *dsp_event_ptr =
      (capi_event_get_data_from_dsp_service_t *)(payload->data_ptr);

   switch (dsp_event_ptr->param_id)
   {
      case INTF_EXTN_EVENT_ID_IMCL_GET_RECURRING_BUF:
      case INTF_EXTN_EVENT_ID_IMCL_GET_ONE_TIME_BUF:
      {
         // Need to exit island if ctrl port lib is in nlpi
         cu_exit_lpi_temporarily_if_ctrl_port_lib_in_nlpi(cu_ptr);

         result = cu_handle_imcl_event(cu_ptr, module_ptr, event_info_ptr);
         break;
      }
      default:
      {
         CU_MSG_ISLAND(cu_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "Module 0x%lX: Error in callback function. ID 0x%lx not supported.",
                module_ptr->module_instance_id,
                (dsp_event_ptr->param_id));
         return AR_EUNSUPPORTED;
      }
   }

   return result;
}
