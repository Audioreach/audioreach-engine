/**
 * \file capi_cmn_imcl_utils.c
 *  
 * \brief
 *        Common utility functions for Inter Module Communication utils
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi.h"
#include "capi_intf_extn_imcl.h"
#include "capi_cmn_imcl_utils.h"
#include "ar_msg.h"
#ifdef __cplusplus
extern "C" {
#endif
capi_err_t capi_cmn_imcl_register_for_recurring_bufs(capi_event_callback_info_t *event_cb_info_ptr,
                                                     uint32_t                    ctrl_port_id,
                                                     uint32_t                    buf_size,
                                                     uint32_t                    num_bufs)
{
   capi_err_t result = CAPI_EOK;

   /** Populate event payload */
   event_id_imcl_recurring_buf_info_t event_payload;
   event_payload.port_id  = ctrl_port_id;
   event_payload.buf_size = buf_size;
   event_payload.num_bufs = num_bufs;

   /** Populate event type header */
   capi_event_data_to_dsp_service_t event_header;
   event_header.param_id                = INTF_EXTN_EVENT_ID_IMCL_RECURRING_BUF_INFO;
   event_header.payload.actual_data_len = sizeof(event_id_imcl_recurring_buf_info_t);
   event_header.payload.max_data_len    = sizeof(event_id_imcl_recurring_buf_info_t);
   event_header.payload.data_ptr        = (int8_t *)&event_payload;

   /** Create event info */
   capi_event_info_t event_info;
   event_info.port_info.is_input_port = FALSE;
   event_info.port_info.is_valid      = FALSE;
   event_info.payload.actual_data_len = sizeof(event_header);
   event_info.payload.max_data_len    = sizeof(event_header);
   event_info.payload.data_ptr        = (int8_t *)&event_header;

   /** Invoke event callback */
   if (CAPI_EOK !=
       (result =
           event_cb_info_ptr->event_cb(event_cb_info_ptr->event_context, CAPI_EVENT_DATA_TO_DSP_SERVICE, &event_info)))
   {
      AR_MSG(DBG_ERROR_PRIO, "Failed to register for recurring bufs on ctrl_port_id[0x%lX]", ctrl_port_id);
   }
   else
   {
      AR_MSG(DBG_MED_PRIO,
             "Registered for [%lu] recurring bufs of size[%lu  bytes] on ctrl_port_id[0x%lX]",
             num_bufs,
             buf_size,
             ctrl_port_id);
   }

   return result;
}


capi_err_t capi_cmn_imcl_return_to_fwk(capi_event_callback_info_t *event_cb_info_ptr,
                                       capi_buf_t *                ctrl_data_buf_ptr,
                                       uint32_t                    ctrl_port_id)
{
   imcl_outgoing_data_flag_t flags;
   flags.should_send = FALSE;
   flags.is_trigger  = FALSE;
   return capi_cmn_imcl_data_send(event_cb_info_ptr, ctrl_data_buf_ptr, ctrl_port_id, flags);
}
#ifdef __cplusplus
}
#endif
