/**
 *   \file capi_sh_mem_push_lab_imc_utils.c
 *   \brief
 *        This file contains CAPI IMC utils implementation of Sh mem Push Lab module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "sh_mem_push_lab_api.h"
#include "capi_sh_mem_push_lab.h"
#include "push_lab.h"
#include "capi_properties.h"
#include "capi.h"
#include "ar_msg.h"
#include "imcl_dam_detection_api.h"
#include "capi_cmn_imcl_utils.h"
#include "imcl_dam_detection_api.h"

/*==============================================================================
   Local Defines
==============================================================================*/
static inline uint32_t align_to_8_byte(const uint32_t num)
{
   return ((num + 7) & (0xFFFFFFF8));
}
/*==============================================================================
   Local Function forward declaration
==============================================================================*/

/*==============================================================================
   Public Function Implementation
==============================================================================*/

uint32_t capi_push_lab_get_ctrl_port_arr_idx_from_ctrl_port_id(capi_push_lab_t *me_ptr, uint32_t ctrl_port_id)
{
   uint32_t available_ctrl_arr_idx = PUSH_LAB_MAX_CTRL_PORT;

   for (uint32_t idx = 0; idx < PUSH_LAB_MAX_CTRL_PORT; idx++)
   {
      if (ctrl_port_id == me_ptr->imcl_port_info_arr[idx].port_id)
      {
         return idx;
      }
      else if (!me_ptr->imcl_port_info_arr[idx].port_id)
      {
         available_ctrl_arr_idx = idx;
      }
   }

   if (available_ctrl_arr_idx != PUSH_LAB_MAX_CTRL_PORT)
   {
      AR_MSG(DBG_LOW_PRIO,
             "capi_push_lab: Mapping Ctrl Port ID =0x%x to index=0x%x",
             ctrl_port_id,
             available_ctrl_arr_idx);
      me_ptr->imcl_port_info_arr[available_ctrl_arr_idx].port_id = ctrl_port_id;
      return available_ctrl_arr_idx;
   }

   AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Ctrl Port ID = 0x%lx to index mapping not found.", ctrl_port_id);
   return UMAX_32;
}

capi_err_t capi_push_lab_imcl_register_for_recurring_bufs(capi_push_lab_t *me_ptr,
                                                          uint32_t         port_id,
                                                          uint32_t         buf_size,
                                                          uint32_t         num_bufs)
{
   capi_err_t result = CAPI_EOK;

   event_id_imcl_recurring_buf_info_t event_payload;
   event_payload.port_id  = port_id;
   event_payload.buf_size = buf_size;
   event_payload.num_bufs = num_bufs;

   /* Create event */
   capi_event_data_to_dsp_service_t to_send;

   to_send.param_id                = INTF_EXTN_EVENT_ID_IMCL_RECURRING_BUF_INFO;
   to_send.payload.actual_data_len = sizeof(event_id_imcl_recurring_buf_info_t);
   to_send.payload.max_data_len    = sizeof(event_id_imcl_recurring_buf_info_t);
   to_send.payload.data_ptr        = (int8_t *)&event_payload;

   /* Create event info */
   capi_event_info_t event_info;
   event_info.port_info.is_input_port = FALSE;
   event_info.port_info.is_valid      = FALSE;
   event_info.payload.actual_data_len = sizeof(to_send);
   event_info.payload.max_data_len    = sizeof(to_send);
   event_info.payload.data_ptr        = (int8_t *)&to_send;

   result = me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_DATA_TO_DSP_SERVICE, &event_info);

   if (CAPI_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Failed to Register for Recurring Bufs on port_id %lu", port_id);
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi_push_lab: Registered for %lu Recurring Bufs of size %lu on port ID 0x%lx",
             num_bufs,
             buf_size,
             port_id);
   }
   return result;
}

uint32_t sh_mem_push_lab_calculate_start_index(capi_push_lab_t *me)
{
   uint32_t start_index;
   // TODO: devise logic for erroneous case where offset is greater than write index when the buffer has not been
   // completely filled even once. Currently, only raising an warning message for all scenarios including error and
   // non-error
   // TODO: If offset greater than circular buffer size, offset should be resized to circular buffer size
   if (me->push_lab_info.detection_offset_in_bytes > me->push_lab_info.current_write_index)
   {
      start_index = me->push_lab_info.shared_circ_buf_size -
                    (me->push_lab_info.detection_offset_in_bytes - me->push_lab_info.current_write_index);
      AR_MSG(DBG_HIGH_PRIO, "capi_push_lab: Warning: offset is greater than write index");
   }
   else
   {
      start_index = (me->push_lab_info.current_write_index - me->push_lab_info.detection_offset_in_bytes);
   }
   return start_index;
}

capi_err_t capi_sh_mem_push_lab_imc_set_param_handler(capi_push_lab_t *me_ptr,
                                                      uint32_t         ctrl_port_id,
                                                      capi_buf_t *     intent_buf_ptr)
{
   capi_err_t result = CAPI_EOK;
   // accessing the wrong payload.. need to do + sizeof(incoming payload struct to access the actual data)
   int8_t * payload_ptr  = intent_buf_ptr->data_ptr + sizeof(intf_extn_param_id_imcl_incoming_data_t);
   uint32_t payload_size = intent_buf_ptr->actual_data_len - sizeof(intf_extn_param_id_imcl_incoming_data_t);

   uint32_t op_arr_index = 0; // one output port
   if (IS_INVALID_PORT_INDEX(op_arr_index))
   {
      return CAPI_EBADPARAM;
   }

   uint32_t ctrl_arr_index = 0; // one control port

   if (IS_INVALID_PORT_INDEX(ctrl_arr_index))
   {
      return CAPI_EBADPARAM;
   }

   if (me_ptr->imcl_port_info_arr[ctrl_arr_index].state != CTRL_PORT_PEER_CONNECTED)
   {
      AR_MSG(DBG_HIGH_PRIO, "capi_push_lab: Warning: IMC set param handler, ctrl port is not started yet. ");
   }

   while (payload_size >= sizeof(vw_imcl_header_t))
   {
      vw_imcl_header_t *header_ptr = (vw_imcl_header_t *)payload_ptr;

      payload_ptr += sizeof(vw_imcl_header_t);
      payload_size -= sizeof(vw_imcl_header_t);

      switch (header_ptr->opcode)
      {
         case PARAM_ID_AUDIO_DAM_RESIZE:
         {
            if (header_ptr->actual_data_len < sizeof(param_id_audio_dam_buffer_resize_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_push_lab: IMC Param id 0x%lx Invalid payload size for incoming data %d",
                      header_ptr->opcode,
                      header_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }
            // using IMCL Dam detection API structures
            param_id_audio_dam_buffer_resize_t *cfg_ptr = (param_id_audio_dam_buffer_resize_t *)payload_ptr;

            me_ptr->push_lab_info.resize_in_us    = cfg_ptr->resize_in_us;
            me_ptr->push_lab_info.resize_in_bytes = 0;

            if (me_ptr->push_lab_info.circ_buf_allocated == 1)
            {
               me_ptr->push_lab_info.resize_in_bytes = sh_mem_push_lab_us_to_bytes(me_ptr, cfg_ptr->resize_in_us);
               if (me_ptr->push_lab_info.resize_in_bytes > me_ptr->push_lab_info.shared_circ_buf_size)
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi_push_lab: Buffer size: allocated by client: %d is less than buffer size required by Voice wakeup: %d",
                         me_ptr->push_lab_info.shared_circ_buf_size,
                         me_ptr->push_lab_info.resize_in_bytes);
               }
            }

            return CAPI_EOK;
            break;
         }
         case PARAM_ID_AUDIO_DAM_DATA_FLOW_CTRL:
         {
            if (header_ptr->actual_data_len < sizeof(param_id_audio_dam_data_flow_ctrl_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_push_lab: Param id 0x%lx Bad param size %lu",
                      (uint32_t)header_ptr->opcode,
                      intent_buf_ptr->actual_data_len);
               return CAPI_ENEEDMORE;
            }

            param_id_audio_dam_data_flow_ctrl_t *cfg_ptr = (param_id_audio_dam_data_flow_ctrl_t *)payload_ptr;

            if (cfg_ptr->is_gate_open)
            {
               me_ptr->push_lab_info.is_gate_opened = TRUE;
			   me_ptr->push_lab_info.acc_data = 0;
               // TODO: check for non-zero offset
               uint32_t read_offset_in_bytes = sh_mem_push_lab_us_to_bytes(me_ptr, cfg_ptr->read_offset_in_us);

               me_ptr->push_lab_info.detection_offset_in_bytes = read_offset_in_bytes;
               // On detection, first watermark event raised from here
               AR_MSG(DBG_HIGH_PRIO,
                      "capi_push_lab: cfg_ptr->read_offset_in_us %lu read_offset_in_bytes %lu",
                      cfg_ptr->read_offset_in_us,
                      read_offset_in_bytes);

               me_ptr->push_lab_info.start_index                = sh_mem_push_lab_calculate_start_index(me_ptr);
               me_ptr->push_lab_info.last_watermark_level_index = me_ptr->push_lab_info.current_write_index;
               result                         = capi_push_lab_populate_payload_raise_watermark_event(me_ptr);

               uint32_t unread_len_in_us = cfg_ptr->read_offset_in_us;
               result |= capi_push_lab_imcl_send_unread_len(me_ptr, unread_len_in_us, ctrl_port_id);
               AR_MSG(DBG_HIGH_PRIO, "capi_push_lab: Sent unread lenght event with result: %d", result);
            }
            else
            {
               if (TRUE == me_ptr->push_lab_info.is_gate_opened)
               {
                  me_ptr->push_lab_info.is_gate_opened = FALSE;
				  me_ptr->push_lab_info.acc_data = 0;
               }
            }
            break;
         }
         case PARAM_ID_AUDIO_DAM_OUTPUT_CH_CFG:
         {
            if (header_ptr->actual_data_len < sizeof(param_id_audio_dam_output_ch_cfg_t))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_push_lab: Param id 0x%lx Bad param size %lu",
                      (uint32_t)header_ptr->opcode,
                      header_ptr->actual_data_len);

               return CAPI_ENEEDMORE;
            }

            param_id_audio_dam_output_ch_cfg_t *cfg_ptr = (param_id_audio_dam_output_ch_cfg_t *)payload_ptr;
            uint32_t                            out_cfg_size =
               sizeof(param_id_audio_dam_output_ch_cfg_t) + (sizeof(uint32_t) * cfg_ptr->num_channels);

            if (header_ptr->actual_data_len < out_cfg_size)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_push_lab: IMC Param id 0x%lx Bad param size %lu required size : %u",
                      (uint32_t)header_ptr->opcode,
                      header_ptr->actual_data_len,
                      out_cfg_size);
               return CAPI_ENEEDMORE;
            }

            if (0 == cfg_ptr->num_channels)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: num_channels = 0 is not avalid param");
               return CAPI_EBADPARAM;
            }


            param_id_audio_dam_output_ch_cfg_t *out_cfg_params_received =
               (param_id_audio_dam_output_ch_cfg_t *)posal_memory_malloc(out_cfg_size, (POSAL_HEAP_ID)(me_ptr->heap_mem.heap_id));

            if (NULL == out_cfg_params_received)
            {
            	AR_MSG(DBG_ERROR_PRIO,
                       "capi_push_lab: Param id 0x%lx, memory couldn't be allocated for the internal "
                       "struts.");
               result |= CAPI_ENOMEMORY;
               break;
            }


            memscpy(out_cfg_params_received, out_cfg_size, cfg_ptr, out_cfg_size);

            me_ptr->push_lab_info.dam_output_ch_cfg_received = out_cfg_params_received;

            push_lab_update_dam_output_ch_cfg(me_ptr);

            if (me_ptr->push_lab_info.is_media_fmt_populated == 1)
            {
               me_ptr->push_lab_info.watermark_interval_in_bytes =
                  sh_mem_push_lab_us_to_bytes(me_ptr, me_ptr->push_lab_info.watermark_interval_in_us);
               AR_MSG(DBG_HIGH_PRIO,
                      "capi_push_lab: updated watermark_interval_in_bytes"
                      "watermark_interval_in_us %d, "
                      "watermark_interval_in_bytes %d",
                      me_ptr->push_lab_info.watermark_interval_in_us,
                      me_ptr->push_lab_info.watermark_interval_in_bytes);
            }

            break;
         }
         default:
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi_push_lab: Unsupported opcode for incoming data over IMCL %d",
                   header_ptr->opcode);
            return CAPI_EUNSUPPORTED;
         }
      }

      AR_MSG(DBG_HIGH_PRIO,
             "capi_push_lab: IMC Set param 0x%x done. payload size = %lu",
             header_ptr->opcode,
             header_ptr->actual_data_len);

      payload_ptr += header_ptr->actual_data_len;
      payload_size -= header_ptr->actual_data_len;
   }

   return result;
}

capi_err_t capi_push_lab_imcl_send_unread_len(capi_push_lab_t *me_ptr, uint32_t unread_len_in_us, uint32_t ctrl_port_id)
{
   capi_err_t result = CAPI_EOK;
   capi_buf_t buf;
   buf.data_ptr                = NULL;
   buf.max_data_len            = 0;
   uint32_t total_payload_size = sizeof(vw_imcl_header_t) + sizeof(param_id_audio_dam_unread_bytes_t);

   // Get one time buf from the queue.
   capi_cmn_imcl_get_recurring_buf(&me_ptr->cb_info, ctrl_port_id, &buf);

   if (!buf.data_ptr || buf.max_data_len < total_payload_size)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Getting one time buffer failed");
      return CAPI_EFAILED;
   }

   vw_imcl_header_t *out_cfg_ptr = (vw_imcl_header_t *)buf.data_ptr;

   out_cfg_ptr->opcode          = PARAM_ID_AUDIO_DAM_UNREAD_DATA_LENGTH;
   out_cfg_ptr->actual_data_len = sizeof(param_id_audio_dam_unread_bytes_t);

   param_id_audio_dam_unread_bytes_t *unread_cfg_ptr =
      (param_id_audio_dam_unread_bytes_t *)(buf.data_ptr + sizeof(vw_imcl_header_t));

   unread_cfg_ptr->unread_in_us = unread_len_in_us;

   buf.actual_data_len = total_payload_size;

   // Send data ready to the peer module
   imcl_outgoing_data_flag_t flags;
   flags.should_send = TRUE;
   flags.is_trigger  = TRUE;
   result            = capi_cmn_imcl_send_to_peer(&me_ptr->cb_info, &buf, ctrl_port_id, flags);

   return result;
}
