/**
 * \file capi_cmn_imcl_utils_island.c
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
capi_err_t capi_cmn_imcl_get_recurring_buf(capi_event_callback_info_t *event_cb_info_ptr,
                                           uint32_t                    ctrl_port_id,
                                           capi_buf_t *                rec_buf_ptr)
{
   capi_err_t result;

   /** Populate event payload */
   event_id_imcl_get_recurring_buf_t event_payload = { 0 };
   event_payload.port_id                           = ctrl_port_id;

   /** Populate event type header */
   capi_event_get_data_from_dsp_service_t event_header;
   event_header.param_id                = INTF_EXTN_EVENT_ID_IMCL_GET_RECURRING_BUF;
   event_header.payload.actual_data_len = sizeof(event_id_imcl_get_recurring_buf_t);
   event_header.payload.max_data_len    = sizeof(event_id_imcl_get_recurring_buf_t);
   event_header.payload.data_ptr        = (int8_t *)&event_payload;

   /** Create event info */
   capi_event_info_t event_info;
   event_info.port_info.is_input_port = FALSE;
   event_info.port_info.is_valid      = FALSE;
   event_info.payload.actual_data_len = sizeof(event_header);
   event_info.payload.max_data_len    = sizeof(event_header);
   event_info.payload.data_ptr        = (int8_t *)&event_header;

   /** Invoke event callback */
   if (CAPI_EOK != (result = event_cb_info_ptr->event_cb(event_cb_info_ptr->event_context,
                                                         CAPI_EVENT_GET_DATA_FROM_DSP_SERVICE,
                                                         &event_info)))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "Failed to get recurring buf on ctrl_port_id[0x%lX]", ctrl_port_id);

      rec_buf_ptr->data_ptr     = NULL;
      rec_buf_ptr->max_data_len = 0;

      return result;
   }

   /** Populate the return pointer */
   rec_buf_ptr->data_ptr     = event_payload.buf.data_ptr;
   rec_buf_ptr->max_data_len = event_payload.buf.max_data_len;

   return result;
}

capi_err_t capi_cmn_imcl_get_one_time_buf(capi_event_callback_info_t *event_cb_info_ptr,
                                          uint32_t                    ctrl_port_id,
                                          uint32_t                    req_buf_size,
                                          capi_buf_t *                ot_buf_ptr)
{
   capi_err_t result;

   /** Populate event payload */
   event_id_imcl_get_one_time_buf_t event_payload;
   event_payload.port_id             = ctrl_port_id;
   event_payload.buf.actual_data_len = req_buf_size;

   /** Populate event type header */
   capi_event_get_data_from_dsp_service_t event_header;
   event_header.param_id                = INTF_EXTN_EVENT_ID_IMCL_GET_ONE_TIME_BUF;
   event_header.payload.actual_data_len = sizeof(event_id_imcl_get_one_time_buf_t);
   event_header.payload.max_data_len    = sizeof(event_id_imcl_get_one_time_buf_t);
   event_header.payload.data_ptr        = (int8_t *)&event_payload;

   /** Create event info */
   capi_event_info_t event_info;
   event_info.port_info.is_input_port = FALSE;
   event_info.port_info.is_valid      = FALSE;
   event_info.payload.actual_data_len = sizeof(event_header);
   event_info.payload.max_data_len    = sizeof(event_header);
   event_info.payload.data_ptr        = (int8_t *)&event_header;

   /** Invoke event callback */
   if (CAPI_EOK != (result = event_cb_info_ptr->event_cb(event_cb_info_ptr->event_context,
                                                         CAPI_EVENT_GET_DATA_FROM_DSP_SERVICE,
                                                         &event_info)))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "Failed to send request to get one-time buf on ctrl_port_id[0x%lX]", ctrl_port_id);

      ot_buf_ptr->data_ptr     = NULL;
      ot_buf_ptr->max_data_len = 0;

      return result;
   }

   /** Populate the return pointer */
   ot_buf_ptr->data_ptr        = event_payload.buf.data_ptr;
   ot_buf_ptr->max_data_len    = event_payload.buf.max_data_len;
   ot_buf_ptr->actual_data_len = event_payload.buf.actual_data_len;

   return result;
}

capi_err_t capi_cmn_imcl_data_send(capi_event_callback_info_t *event_cb_info_ptr,
                                          capi_buf_t *                ctrl_data_buf_ptr,
                                          uint32_t                    ctrl_port_id,
                                          imcl_outgoing_data_flag_t   flags)
{
   capi_err_t result = CAPI_EOK;

   /** Populate event payload */
   event_id_imcl_outgoing_data_t event_payload;
   event_payload.port_id             = ctrl_port_id;
   event_payload.buf.actual_data_len = ctrl_data_buf_ptr->actual_data_len;
   event_payload.buf.data_ptr        = ctrl_data_buf_ptr->data_ptr;
   event_payload.buf.max_data_len    = ctrl_data_buf_ptr->max_data_len;
   event_payload.flags               = flags;

   /** Populate event type header */
   capi_event_data_to_dsp_service_t event_header;
   event_header.param_id                = INTF_EXTN_EVENT_ID_IMCL_OUTGOING_DATA;
   event_header.payload.actual_data_len = sizeof(event_id_imcl_outgoing_data_t);
   event_header.payload.max_data_len    = sizeof(event_id_imcl_outgoing_data_t);
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
      AR_MSG_ISLAND(DBG_ERROR_PRIO, "Failed to sent ctrl data buf on ctrl port id[0x%lX]", ctrl_port_id);
   }
   else
   {

#ifdef DEBUG_IMCL
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
             "Sent ctrl data Buf of size[%lu bytes] on ctrl port ID[0xlX], send_to_peer[%d, 0/1: Returned/Sent]",
             data_len,
             ctrl_port_id,
             send_to_peer);
#endif /* DEBUG_IMCL */
   }

   return result;
}

capi_err_t capi_cmn_imcl_send_to_peer(capi_event_callback_info_t *event_cb_info_ptr,
                                      capi_buf_t *                ctrl_data_buf_ptr,
                                      uint32_t                    ctrl_port_id,
                                      imcl_outgoing_data_flag_t   flags)
{
   return capi_cmn_imcl_data_send(event_cb_info_ptr, ctrl_data_buf_ptr, ctrl_port_id, flags);
}

#ifdef __cplusplus
}
#endif