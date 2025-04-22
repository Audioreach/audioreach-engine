/**
 *   \file capi_spr_path_delay.c
 *   \brief
 *        This file contains CAPI implementation of SPR path delay
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_spr_i.h"
#include "capi_fwk_extns_signal_triggered_module.h"
#include "capi_intf_extn_path_delay.h"
#include "imcl_fwk_intent_api.h"

/*------------------------------------------------------------------------------
  Function name: spr_reset_path_delay
   Resets the path delay information cached inside the SPR
 * ------------------------------------------------------------------------------*/
capi_err_t spr_reset_path_delay(capi_spr_t *me_ptr, spr_output_port_info_t *out_port_info_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (out_port_info_ptr->path_delay.delay_us_pptr)
   {
      SPR_MSG(me_ptr->miid, DBG_LOW_PRIO, "PATH_DELAY: reset path-id 0x%lX", out_port_info_ptr->path_delay.path_id);

      posal_memory_free(out_port_info_ptr->path_delay.delay_us_pptr);
      out_port_info_ptr->path_delay.delay_us_pptr = NULL;
   }

   memset(&out_port_info_ptr->path_delay, 0 , sizeof(out_port_info_ptr->path_delay));

   return result;
}

/*------------------------------------------------------------------------------
  Function name: spr_set_response_to_path_delay_event
   Stores the response to the path delay query
 * ------------------------------------------------------------------------------*/
capi_err_t spr_set_response_to_path_delay_event(capi_spr_t *me_ptr, capi_buf_t *params_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (params_ptr->actual_data_len < sizeof(intf_extn_path_delay_response_t))
   {
      SPR_MSG(me_ptr->miid,
              DBG_ERROR_PRIO,
              "PATH_DELAY: Param INTF_EXTN_PARAM_ID_RESPONSE_PATH_DELAY Bad param size %lu",
              params_ptr->actual_data_len);
      return CAPI_EFAILED;
   }

   intf_extn_path_delay_response_t *rsp_ptr = (intf_extn_path_delay_response_t *)params_ptr->data_ptr;
   if (rsp_ptr->src_module_instance_id != me_ptr->miid)
   {
      SPR_MSG(me_ptr->miid,
              DBG_ERROR_PRIO,
              "PATH_DELAY: module instance ID 0x%08lX doesn't match with mine 0x%08lX",
              rsp_ptr->src_module_instance_id,
              me_ptr->miid);
      return CAPI_EFAILED;
   }

   uint32_t arr_index = spr_get_arr_index_from_port_id(me_ptr, rsp_ptr->src_port_id);
   if (UMAX_32 == arr_index)
   {
      SPR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "PATH_DELAY: source port id 0x%lX, doesn't exist", rsp_ptr->src_port_id);
      return CAPI_EFAILED;
   }

   spr_output_port_info_t *out_port_info_ptr = &me_ptr->out_port_info_arr[arr_index];

   {
      if ((0 != out_port_info_ptr->path_delay.path_id) || (NULL != out_port_info_ptr->path_delay.delay_us_pptr))
      {
         SPR_MSG(me_ptr->miid,
                 DBG_ERROR_PRIO,
                 "PATH_DELAY: path ID 0x%08lX already present, destroying and recreating",
                 out_port_info_ptr->path_delay.path_id);
         spr_reset_path_delay(me_ptr, out_port_info_ptr);
      }

      out_port_info_ptr->path_delay.path_id            = rsp_ptr->path_id;
      out_port_info_ptr->path_delay.dst_module_iid     = rsp_ptr->dst_module_instance_id;
      out_port_info_ptr->path_delay.dst_module_port_id = rsp_ptr->dst_port_id;

      uint32_t sz                                      = sizeof(uint32_t) * rsp_ptr->num_delay_ptrs;
      out_port_info_ptr->path_delay.delay_us_pptr      = (volatile uint32_t **)posal_memory_malloc(sz, me_ptr->heap_id);
      if (NULL == out_port_info_ptr->path_delay.delay_us_pptr)
      {
         SPR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "PATH_DELAY: Failed malloc of delay array");
         return CAPI_ENOMEMORY;
      }
      out_port_info_ptr->path_delay.num_delay_ptrs = rsp_ptr->num_delay_ptrs;
      memscpy(out_port_info_ptr->path_delay.delay_us_pptr, sz, rsp_ptr->delay_us_pptr, sz);

      SPR_MSG(me_ptr->miid,
              DBG_LOW_PRIO,
              "PATH_DELAY: path-id 0x%lX, module instance ID or port ID src(0x%08lX, 0x%08lX) and dst "
              "(0x%08lX, 0x%08lX). Copied %lu delay pointers. Delay %lu us",
              out_port_info_ptr->path_delay.path_id,
              rsp_ptr->src_module_instance_id,
              rsp_ptr->src_port_id,
              rsp_ptr->dst_module_instance_id,
              rsp_ptr->dst_port_id,
              out_port_info_ptr->path_delay.num_delay_ptrs,
              spr_aggregate_path_delay(me_ptr, out_port_info_ptr));
   }

   return result;
}

/*------------------------------------------------------------------------------
  Function name: spr_set_destroy_path_delay_cfg
   Destroys the path delay information associated with the given path(s)
 * ------------------------------------------------------------------------------*/
capi_err_t spr_set_destroy_path_delay_cfg(capi_spr_t *me_ptr, capi_buf_t *params_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (params_ptr->actual_data_len < sizeof(intf_extn_path_delay_destroy_t))
   {
      SPR_MSG(me_ptr->miid,
              DBG_ERROR_PRIO,
              "PATH_DELAY: Param INTF_EXTN_PARAM_ID_DESTROY_PATH_DELAY Bad param size %lu",
              params_ptr->actual_data_len);
      return CAPI_EFAILED;
   }

   intf_extn_path_delay_destroy_t *rsp_ptr = (intf_extn_path_delay_destroy_t *)params_ptr->data_ptr;
   if (rsp_ptr->src_module_instance_id != me_ptr->miid)
   {
      SPR_MSG(me_ptr->miid,
              DBG_ERROR_PRIO,
              "PATH_DELAY: module instance ID 0x%08lX doesn't match with mine 0x%08lX",
              rsp_ptr->src_module_instance_id,
              me_ptr->miid);
      return CAPI_EFAILED;
   }

   uint32_t arr_index = spr_get_arr_index_from_port_id(me_ptr, rsp_ptr->src_port_id);
   if (UMAX_32 == arr_index)
   {
      SPR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "PATH_DELAY: source port id 0x%lX, doesn't exist", rsp_ptr->src_port_id);
      return CAPI_EFAILED;
   }

   spr_output_port_info_t *out_port_info_ptr = &me_ptr->out_port_info_arr[arr_index];

   if (out_port_info_ptr->port_state != DATA_PORT_STATE_CLOSED)
   {
      if (rsp_ptr->path_id != out_port_info_ptr->path_delay.path_id)
      {
         SPR_MSG(me_ptr->miid,
                 DBG_ERROR_PRIO,
                 "PATH_DELAY: path ID 0x%08lX doesn't match with mine 0x%08lX",
                 rsp_ptr->path_id,
                 out_port_info_ptr->path_delay.path_id);

         return CAPI_EFAILED;
      }
      else
      {
         spr_reset_path_delay(me_ptr, out_port_info_ptr);
      }
   }

   return result;
}

/*------------------------------------------------------------------------------
  Function name: spr_request_path_delay
   Raises a request for a path delay when PARAM_ID_SPR_DELAY_PATH_END is set
 * ------------------------------------------------------------------------------*/
capi_err_t spr_request_path_delay(capi_spr_t *me_ptr, uint32_t end_module_iid)
{
   capi_err_t result = CAPI_EOK;

   // until data port, control port are opened and path delay intent defined, no need to request

   intf_extn_event_id_request_path_delay_t event_payload;
   event_payload.dst_module_instance_id = end_module_iid;
   event_payload.dst_port_id            = 0; // APM will give the port id
   event_payload.src_module_instance_id = me_ptr->miid;
   event_payload.src_port_id            = 0; // APM will give the port id

   /* Create event */
   capi_event_data_to_dsp_service_t to_send;
   to_send.param_id                = INTF_EXTN_EVENT_ID_REQUEST_PATH_DELAY;
   to_send.payload.actual_data_len = sizeof(intf_extn_event_id_request_path_delay_t);
   to_send.payload.max_data_len    = sizeof(intf_extn_event_id_request_path_delay_t);
   to_send.payload.data_ptr        = (int8_t *)&event_payload;

   /* Create event info */
   capi_event_info_t event_info;
   event_info.port_info.is_input_port = FALSE;
   event_info.port_info.is_valid      = FALSE;
   event_info.payload.actual_data_len = sizeof(to_send);
   event_info.payload.max_data_len    = sizeof(to_send);
   event_info.payload.data_ptr        = (int8_t *)&to_send;

   result =
      me_ptr->event_cb_info.event_cb(me_ptr->event_cb_info.event_context, CAPI_EVENT_DATA_TO_DSP_SERVICE, &event_info);

   if (CAPI_EOK != result)
   {
      SPR_MSG(me_ptr->miid,
              DBG_ERROR_PRIO,
              "PATH_DELAY: Failed to request path delay for src 0x%08lX, dst 0x%08lX",
              event_payload.src_module_instance_id,
              event_payload.dst_module_instance_id);
   }
   else
   {
      SPR_MSG(me_ptr->miid,
              DBG_LOW_PRIO,
              "PATH_DELAY: request path delay sent for src 0x%08lX, dst 0x%08lX",
              event_payload.src_module_instance_id,
              event_payload.dst_module_instance_id);
   }
   return result;
}

//// raise capi events on receiving the input and output port configurations.
// static capi_err_t capi_raise_mpps_and_bw_events(capi_spr_t *me_ptr)
//{
//   // Need to profile this.
//   return CAPI_EOK;
//}
