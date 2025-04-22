/**
 * \file example_encoder_module_utils.c
 *  
 * \brief
 *  
 *     Example Encoder Module
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "capi_example_enc_module_structs.h"

/*------------------------------------------------------------------------
 * Functions
 * -----------------------------------------------------------------------*/

/* =========================================================================
 * FUNCTION : capi_example_enc_module_module_init_media_fmt_v2
 * DESCRIPTION: Function to initialize the media format of the example encoder module
 * =========================================================================*/
capi_err_t capi_example_enc_module_init_media_fmt_v2(capi_example_enc_inp_media_fmt_t *media_fmt_ptr)
{
   media_fmt_ptr->header.format_header.data_format = CAPI_FIXED_POINT;
   media_fmt_ptr->format.minor_version             = CAPI_DATA_FORMAT_INVALID_VAL;
   media_fmt_ptr->format.bits_per_sample           = CAPI_DATA_FORMAT_INVALID_VAL;
   media_fmt_ptr->format.bitstream_format          = CAPI_DATA_FORMAT_INVALID_VAL;
   media_fmt_ptr->format.data_interleaving         = CAPI_INVALID_INTERLEAVING;
   media_fmt_ptr->format.data_is_signed            = CAPI_DATA_FORMAT_INVALID_VAL;
   media_fmt_ptr->format.num_channels              = CAPI_DATA_FORMAT_INVALID_VAL;
   media_fmt_ptr->format.q_factor                  = CAPI_DATA_FORMAT_INVALID_VAL;
   media_fmt_ptr->format.sampling_rate             = CAPI_DATA_FORMAT_INVALID_VAL;

   for (uint32_t j = 0; j < CAPI_MAX_CHANNELS_V2; j++)
   {
      media_fmt_ptr->channel_type[j] = (uint16_t)CAPI_DATA_FORMAT_INVALID_VAL;
   }
   return CAPI_EOK;
}

/* =========================================================================
 * FUNCTION : capi_example_enc_module_init_capi
 * DESCRIPTION: Function to initialize/ re-initialize the capi
 * =========================================================================*/
capi_err_t capi_example_enc_module_init_capi(capi_example_enc_module_t *me_ptr)
{
   me_ptr->capi_init_done = TRUE;

   example_enc_init(me_ptr->enc_cfg_abc, me_ptr->enc_cfg_xyz);

   AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module: init capi done.");
   return CAPI_EOK;
}

/* =========================================================================
 * FUNCTION : capi_example_enc_module_set_bit_rate
 * DESCRIPTION: Function to set enc bitrate and reinit
 * =========================================================================*/
capi_err_t capi_example_enc_module_set_bit_rate(capi_example_enc_module_t *me_ptr, capi_buf_t *params_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (params_ptr->actual_data_len < sizeof(param_id_enc_bitrate_param_t))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module: set enc bitrate param size not big enough!");
      return CAPI_EBADPARAM;
   }
   param_id_enc_bitrate_param_t *param_bitrate   = (param_id_enc_bitrate_param_t *)params_ptr->data_ptr;
   uint32_t                      new_bitrate     = param_bitrate->bitrate;
   uint32_t                      current_bitrate = me_ptr->bitrate;

   if (new_bitrate != current_bitrate)
   {
      me_ptr->bitrate = new_bitrate;
      /* Reinit capi with new bitrate*/
      result = capi_example_enc_module_init_capi(me_ptr);
   }

   if (CAPI_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module: Failed to set encoder bit rate param!");
      return CAPI_EFAILED;
   }
   else
   {
      AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module: Success setting encoder bit rate to %lu!", me_ptr->bitrate);
   }

   return result;
}

/* =========================================================================
 * FUNCTION : capi_example_enc_module_set_enc_cfg_blk
 * DESCRIPTION: Function to set the encoder configuration block
 * =========================================================================*/
capi_err_t capi_example_enc_module_set_enc_cfg_blk(capi_example_enc_module_t *me_ptr,
                                                         capi_buf_t *               params_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (params_ptr->actual_data_len < sizeof(param_id_encoder_output_config_t))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module: set enc cfg blk param size not big enough!");
      return CAPI_EBADPARAM;
   }

   param_id_encoder_output_config_t *enc_cfg_ptr        = (param_id_encoder_output_config_t *)params_ptr->data_ptr;
   uint8_t *payload = (uint8_t *)(enc_cfg_ptr + 1);
   example_enc_cfg_t *               example_config_ptr = (example_enc_cfg_t *)payload;

   /* Caching encoder config*/
   me_ptr->enc_cfg_abc = example_config_ptr->abc;
   me_ptr->enc_cfg_xyz = example_config_ptr->xyz;

   /* 0 sample rate or 0 channels means native mode, so hold off initialization until
    we get input media format */
   if ((me_ptr->input_media_format.format.num_channels) && (me_ptr->input_media_format.format.sampling_rate))
   {
      /* Reinit capi with new configuration*/
      result = capi_example_enc_module_init_capi(me_ptr);

      if (CAPI_EOK != result)
      {
         AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module: Failed to set encoder cfg params!");
         result = CAPI_EBADPARAM;
      }
      else
      {
         AR_MSG(DBG_HIGH_PRIO,
                "capi_example_enc_module: Success setting encoder cfg params with abc %lu and xyz %lu!",
                me_ptr->enc_cfg_abc,
                me_ptr->enc_cfg_xyz);
      }
   }

   /* Raise events for kpps and threshold since they might change with configuration */
   result |= capi_example_enc_module_raise_kpps_event(me_ptr, CAPI_EXAMPLE_ENC_DEFAULT_KPPS);

   result |= capi_example_enc_module_raise_port_data_threshold_event(me_ptr,
                                                                        EXAMPLE_ENC_INP_BUF_SIZE,
                                                                        TRUE /* is input port*/,
                                                                        0 /*port index*/);

   if (CAPI_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module: Failed to raise events after setting encoder cfg params!");
      result = CAPI_EBADPARAM;
   }

   return result;
}

/* =========================================================================
 * FUNCTION : capi_example_enc_module_module_init_media_fmt_v2
 * DESCRIPTION: Function to initialize the media format of the example encoder module
 * =========================================================================*/
capi_err_t capi_example_enc_module_raise_port_data_threshold_event(capi_example_enc_module_t *me_ptr,
                                                                         uint32_t                      threshold_bytes,
                                                                         bool_t                        is_input_port,
                                                                         uint32_t                      port_index)
{
   capi_err_t result = CAPI_EOK;

   if (NULL == me_ptr->cb_info.event_cb)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi_example_enc_module : Event callback is not set, Unable to raise threshold event!");
      return CAPI_EBADPARAM;
   }

   /* Raise event only if threshold changed */
   if (me_ptr->port_threshold != threshold_bytes)
   {
      capi_port_data_threshold_change_t event;
      event.new_threshold_in_bytes = threshold_bytes;

      capi_event_info_t event_info;
      event_info.port_info.is_input_port = is_input_port;
      event_info.port_info.is_valid      = TRUE;
      event_info.port_info.port_index    = port_index;
      event_info.payload.actual_data_len = sizeof(capi_port_data_threshold_change_t);
      event_info.payload.data_ptr        = (int8_t *)&event;
      event_info.payload.max_data_len    = sizeof(capi_port_data_threshold_change_t);

      result =
         me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_PORT_DATA_THRESHOLD_CHANGE, &event_info);

      if (CAPI_FAILED(result))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "capi_example_enc_module: Failed to send output media format updated event v2 with %d",
                (int)result);
         return result;
      }

      /* If success, cache the new threshold*/
      me_ptr->port_threshold = threshold_bytes;
   }
   return result;
}

/* =========================================================================
 * FUNCTION : capi_example_enc_module_raise_process_event
 * DESCRIPTION: Function to raise the process check event using the
 * callback function to inform the framework if module is enabled or disabled
 * =========================================================================*/
capi_err_t capi_example_enc_module_raise_kpps_event(capi_example_enc_module_t *me_ptr, uint32_t kpps)
{
   capi_err_t result = CAPI_EOK;

   /* Raise event only if the code or data bandwidth changed*/
   if (me_ptr->kpps != kpps)
   {
      /* Event struct for kpps */
      capi_event_KPPS_t event;
      event.KPPS = kpps;

      /* Event info parent struct used for all events */
      capi_event_info_t event_info;

      /* This is set to FALSE if the event being raised is not port specific */
      event_info.port_info.is_valid = FALSE;

      /* Populate the actual data length of the event payload*/
      event_info.payload.actual_data_len = event_info.payload.max_data_len = sizeof(capi_event_KPPS_t);
      event_info.payload.data_ptr                                          = (int8_t *)&event;

      /* Raise callback event with opcode CAPI_EVENT_KPPS*/
      result = me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_KPPS, &event_info);
      if (CAPI_FAILED(result))
      {
         AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module : Failed to send KPPS raise event with %lu", result);
         return result;
      }

      /* If success, save the new values in the capi struct*/
      me_ptr->kpps = kpps;
   }
   return result;
}

/* =========================================================================
 * FUNCTION : capi_example_enc_module_raise_process_event
 * DESCRIPTION: Function to raise the process check event using the
 * callback function to inform the framework if module is enabled or disabled
 * =========================================================================*/
capi_err_t capi_example_enc_module_raise_bandwidth_event(capi_example_enc_module_t *me_ptr,
                                                               uint32_t                      code_bandwidth,
                                                               uint32_t                      data_bandwidth)
{
   capi_err_t result = CAPI_EOK;

   /* Raise event only if the code or data bandwidth changed*/
   if ((me_ptr->code_bw != code_bandwidth) || (me_ptr->data_bw != data_bandwidth))
   {
      /* Event struct for bandwidth */
      capi_event_bandwidth_t event;
      event.code_bandwidth = code_bandwidth;
      event.data_bandwidth = data_bandwidth;

      /* Event info parent struct used for all events */
      capi_event_info_t event_info;
      /* This is set to FALSE if the event being raised is not port specific */
      event_info.port_info.is_valid = FALSE;
      /* Populate the actual data length of the event payload*/
      event_info.payload.actual_data_len = sizeof(event);
      event_info.payload.max_data_len    = sizeof(event);
      event_info.payload.data_ptr        = (int8_t*)&event;

      /* Raise callback event with opcode CAPI_EVENT_BANDWIDTH*/
      result = me_ptr->cb_info.event_cb(me_ptr->cb_info.event_context, CAPI_EVENT_BANDWIDTH, &event_info);
      if (CAPI_FAILED(result))
      {
         AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module : Failed to send bandwidth raise event with %lu", result);
         return result;
      }

      /* If success, Save the new values in the capi struct*/
      me_ptr->code_bw = code_bandwidth;
      me_ptr->data_bw = data_bandwidth;
   }
   return result;
}

/* =========================================================================
 * FUNCTION : capi_example_enc_module_output_media_fmt_event_v2
 * DESCRIPTION: Function to raise output media format event
 * =========================================================================*/
capi_err_t capi_example_enc_module_raise_output_media_fmt_event_v2(capi_event_callback_info_t *cb_info_ptr)
{
   capi_err_t result = CAPI_EOK;
   if (NULL == cb_info_ptr->event_cb)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi_example_enc_module : Event callback is not set or media fmt is NULL, Unable to raise output "
             "media fmt v2 event!");
      return CAPI_EBADPARAM;
   }

   // Raising output media fmt
   capi_example_enc_out_media_fmt_t out_media_fmt;
   out_media_fmt.format.bitstream_format          = MEDIA_FMT_ID_EXAMPLE;
   out_media_fmt.header.format_header.data_format = CAPI_RAW_COMPRESSED;

   capi_event_info_t event_info;
   event_info.port_info.port_index    = 0;
   event_info.port_info.is_valid      = TRUE;
   event_info.port_info.is_input_port = FALSE;
   event_info.payload.actual_data_len = sizeof(capi_example_enc_out_media_fmt_t);
   event_info.payload.data_ptr        = (int8_t *)(&out_media_fmt);

   result =
      cb_info_ptr->event_cb(cb_info_ptr->event_context, CAPI_EVENT_OUTPUT_MEDIA_FORMAT_UPDATED_V2, &event_info);

   if (CAPI_FAILED(result))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi_example_enc_module: Failed to send output media format updated event V2 with result %d",
             (int)result);
   }
   return result;
}

/* =========================================================================
 * FUNCTION : capi_example_enc_module_process_set_properties
 * DESCRIPTION:  Implementation of set properties
 * =========================================================================*/
capi_err_t capi_example_enc_module_process_set_properties(capi_example_enc_module_t *me_ptr,
                                                                capi_proplist_t *          proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK, capi_result2 = CAPI_EOK;

   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module : Set property received null ptr");
      return CAPI_EBADPARAM;
   }

   AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module Set");
   capi_prop_t *prop_array = proplist_ptr->prop_ptr;
   uint32_t        i          = 0;

   for (i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &(prop_array[i].payload);

      switch (prop_array[i].id)
      {
         case CAPI_EVENT_CALLBACK_INFO:
         {
            AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module Set");

            if (payload_ptr->actual_data_len >= sizeof(capi_event_callback_info_t))
            {
               AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module Set");

               capi_event_callback_info_t *data_ptr = (capi_event_callback_info_t *)payload_ptr->data_ptr;
               me_ptr->cb_info.event_cb                = data_ptr->event_cb;
               me_ptr->cb_info.event_context           = data_ptr->event_context;
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_example_enc_module: Set property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_HEAP_ID:
         {
            AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module Set");

            if (payload_ptr->actual_data_len >= sizeof(capi_heap_id_t))
            {
               AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module Set");

               capi_heap_id_t *data_ptr = (capi_heap_id_t *)payload_ptr->data_ptr;
               me_ptr->heap_info.heap_id   = data_ptr->heap_id;
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_example_enc_module: Set property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_PORT_NUM_INFO:
         {
            AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module Set");

            if (payload_ptr->actual_data_len >= sizeof(capi_port_num_info_t))
            {
               AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module Set");

               capi_port_num_info_t *data_ptr = (capi_port_num_info_t *)payload_ptr->data_ptr;
               if ((EXAMPLE_ENC_MODULE_DATA_MAX_INPUT_PORTS < data_ptr->num_input_ports) ||
                   (EXAMPLE_ENC_MODULE_DATA_MAX_OUTPUT_PORTS < data_ptr->num_output_ports))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi_example_enc_module: Set property id 0x%lx, number of input/output ports cannot be "
                         "more than %d",
                         (uint32_t)prop_array[i].id,
                         EXAMPLE_ENC_MODULE_DATA_MAX_OUTPUT_PORTS);
                  capi_result |= CAPI_ENEEDMORE;
               }
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_example_enc_module: Set property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module Set");

            if (payload_ptr->actual_data_len >=
                (sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t)))
            {
               capi_set_get_media_format_t *data_ptr = (capi_set_get_media_format_t *)payload_ptr->data_ptr;
               if (NULL == me_ptr || NULL == data_ptr ||
                   (prop_array[i].port_info.is_valid && prop_array[i].port_info.port_index != 0) ||
                   ((data_ptr->format_header.data_format != CAPI_FIXED_POINT)))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi_example_enc_module Set, failed to set Param id 0x%lx due "
                         "to invalid/unexpected values",
                         (uint32_t)prop_array[i].id);

                  // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
                  payload_ptr->actual_data_len = 0;
                  return (capi_result | CAPI_EFAILED);
               }

               bool_t any_set_done = FALSE;

               capi_standard_data_format_v2_t *std_ptr = (capi_standard_data_format_v2_t *)(data_ptr + 1);

               if (std_ptr->num_channels != CAPI_DATA_FORMAT_INVALID_VAL)
               {
                  AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module Set, channels  %lu", std_ptr->num_channels);
                  me_ptr->input_media_format.format.num_channels = std_ptr->num_channels;
                  any_set_done                                   = TRUE;
               }

               if (std_ptr->sampling_rate != CAPI_DATA_FORMAT_INVALID_VAL)
               {
                  AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module Set, sampling_rate  %lu", std_ptr->sampling_rate);
                  me_ptr->input_media_format.format.sampling_rate = std_ptr->sampling_rate;
                  any_set_done                                    = TRUE;
               }

               if (any_set_done)
               {
                  AR_MSG(DBG_HIGH_PRIO,
                         "capi_example_enc_module Set inp mf sampling rate %lu, num channels %lu ",
                         me_ptr->input_media_format.format.sampling_rate,
                         me_ptr->input_media_format.format.num_channels);

                  /* Reinit capi if num channels or sampling rate changes*/
                  capi_result |= capi_example_enc_module_init_capi(me_ptr);
               }

               // raise output mf event
               capi_result |= capi_example_enc_module_raise_output_media_fmt_event_v2(&me_ptr->cb_info);

               // raise port data threshold event
               capi_result |= capi_example_enc_module_raise_port_data_threshold_event(me_ptr,
                                                                                            EXAMPLE_ENC_INP_BUF_SIZE,
                                                                                            TRUE /*is input port*/,
                                                                                            0 /*port index*/);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_example_enc_module: Set property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_ALGORITHMIC_RESET:
         {
            capi_result |= capi_example_enc_module_init_capi(me_ptr);
            break;
         }
         case CAPI_INTERFACE_EXTENSIONS:
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            /* No implementation for this module*/
            break;
         }
         default:
         {
            AR_MSG(DBG_HIGH_PRIO, "capi_example_enc_module: Set property id %#x. Not supported.", prop_array[i].id);
            break;
         }
      }
      /* If the result is EOK or EUNSUPPORTED, it is not a real error*/
      if ((CAPI_EOK != capi_result) && (CAPI_EUNSUPPORTED != capi_result))
      {
         CAPI_SET_ERROR(capi_result2, capi_result);
      }
   }

   return capi_result2;
}

/* =========================================================================
 * FUNCTION : capi_example_enc_module_process_get_properties
 * DESCRIPTION:  Combined implementation of get static properties and get properties
 * =========================================================================*/
capi_err_t capi_example_enc_module_process_get_properties(capi_example_enc_module_t *me_ptr,
                                                                capi_proplist_t *          proplist_ptr)
{
   capi_err_t   capi_result  = CAPI_EOK;
   capi_err_t   capi_result2 = CAPI_EOK;
   capi_prop_t *prop_array      = proplist_ptr->prop_ptr;

   for (uint32_t i = 0; i < proplist_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_array[i].payload;
      if (NULL == payload_ptr->data_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "capi_example_enc_module: Get property id 0x%x, received null buffer",
                prop_array[i].id);
         capi_result |= CAPI_EBADPARAM;
         break;
      }
      switch (prop_array[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         {
            /* Memory to be allocate for the example encoder module capi*/
            if (payload_ptr->max_data_len >= sizeof(capi_init_memory_requirement_t))
            {
               capi_init_memory_requirement_t *data_ptr =
                  (capi_init_memory_requirement_t *)(payload_ptr->data_ptr);
               data_ptr->size_in_bytes      = sizeof(capi_example_enc_module_t);
               payload_ptr->actual_data_len = sizeof(capi_init_memory_requirement_t);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_example_enc_module: Get property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_STACK_SIZE:
         {
            /* example encoder module stack size*/
            if (payload_ptr->max_data_len >= sizeof(capi_stack_size_t))
            {
               capi_stack_size_t *data_ptr = (capi_stack_size_t *)payload_ptr->data_ptr;
               data_ptr->size_in_bytes        = EXAMPLE_ENC_STACK_SIZE;
               payload_ptr->actual_data_len   = sizeof(capi_stack_size_t);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_example_enc_module: Get property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->actual_data_len);
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_IS_INPLACE:
         {
            if (payload_ptr->max_data_len >= sizeof(capi_is_inplace_t))
            {
               capi_is_inplace_t *data_ptr = (capi_is_inplace_t *)payload_ptr->data_ptr;
               data_ptr->is_inplace           = FALSE;
               payload_ptr->actual_data_len   = sizeof(capi_is_inplace_t);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_example_enc_module: Get  property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->max_data_len);
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_REQUIRES_DATA_BUFFERING:
         {
            if (payload_ptr->max_data_len >= sizeof(capi_requires_data_buffering_t))
            {
               capi_requires_data_buffering_t *data_ptr = (capi_requires_data_buffering_t *)payload_ptr->data_ptr;
               data_ptr->requires_data_buffering           = TRUE;
               payload_ptr->actual_data_len                = sizeof(capi_requires_data_buffering_t);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_example_enc_module: Get  property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->max_data_len);
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE:
         {
            if (payload_ptr->max_data_len >= sizeof(capi_output_media_format_size_t))
            {
               capi_output_media_format_size_t *data_ptr =
                  (capi_output_media_format_size_t *)payload_ptr->data_ptr;
               data_ptr->size_in_bytes      = sizeof(capi_raw_compressed_data_format_t);
               payload_ptr->actual_data_len = sizeof(capi_output_media_format_size_t);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_ldac_enc: Get, Property 0x%lx Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->max_data_len);
               payload_ptr->actual_data_len = 0;
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module : null ptr while querying output mf");
               capi_result |= CAPI_EBADPARAM;
               break;
            }

            if (payload_ptr->max_data_len >= sizeof(capi_set_get_media_format_t))
            {
               AR_MSG(DBG_HIGH_PRIO, "size of payload: %lu", payload_ptr->max_data_len);
               capi_set_get_media_format_t *main_ptr = (capi_set_get_media_format_t *)payload_ptr->data_ptr;

               if (NULL == me_ptr || NULL == main_ptr ||
                   (prop_array[i].port_info.is_valid && prop_array[i].port_info.port_index != 0))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi_example_enc_module Get, failed to get Param id 0x%lx due "
                         "to invalid/unexpected values",
                         (uint32_t)prop_array[i].id);

                  // according to CAPI V2, actual len should be num of bytes read(set)/written(get)
                  payload_ptr->actual_data_len = 0;
                  return (capi_result | CAPI_EFAILED);
               }

               main_ptr->format_header.data_format = CAPI_RAW_COMPRESSED;
               capi_raw_compressed_data_format_t *raw_fmt_ptr;
               raw_fmt_ptr =
                  (capi_raw_compressed_data_format_t *)((int8_t *)main_ptr + sizeof(capi_set_get_media_format_t));
               raw_fmt_ptr->bitstream_format = MEDIA_FMT_ID_EXAMPLE;
               payload_ptr->actual_data_len =
                  sizeof(capi_set_get_media_format_t) + sizeof(capi_raw_compressed_data_format_t);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_example_enc_module: Get property_id 0x%lx, Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->max_data_len);
               payload_ptr->actual_data_len = 0;
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_PORT_DATA_THRESHOLD:
         {
            if (NULL == me_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module : null ptr while querying threshold");
               return CAPI_EBADPARAM;
            }
            if (payload_ptr->max_data_len >= sizeof(capi_port_data_threshold_t))
            {
               capi_port_data_threshold_t *data_ptr = (capi_port_data_threshold_t *)payload_ptr;
               if (!prop_array[i].port_info.is_valid)
               {
                  AR_MSG(DBG_ERROR_PRIO, "capi_example_enc_module : Get port threshold port id not valid");
                  capi_result |= CAPI_EBADPARAM;
               }
               if (0 != prop_array[i].port_info.port_index)
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi_example_enc_module: Get property_id 0x%lx, max in/out port is 1. asking for %lu",
                         (uint32_t)prop_array[i].id,
                         prop_array[i].port_info.port_index);
                  capi_result |= CAPI_EBADPARAM;
               }

               if (prop_array[i].port_info.is_input_port)
               {
                  if (0 != prop_array[i].port_info.port_index)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "capi_example_enc_module: Get, Param id 0x%lx max input port "
                            "is 1. asking for %lu",
                            (uint32_t)prop_array[i].id,
                            prop_array[i].port_info.port_index);
                     payload_ptr->actual_data_len = 0;
                     return (capi_result | CAPI_EBADPARAM);
                  }

                  data_ptr->threshold_in_bytes = EXAMPLE_ENC_INP_BUF_SIZE;
               }
               else
               {
                  if (0 != prop_array[i].port_info.port_index)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "capi_example_enc_module: Get, Param id 0x%lx max output port "
                            "is 1. asking for %lu",
                            (uint32_t)prop_array[i].id,
                            prop_array[i].port_info.port_index);
                     payload_ptr->actual_data_len = 0;
                     return (capi_result | CAPI_EBADPARAM);
                  }
                  data_ptr->threshold_in_bytes = EXAMPLE_ENC_OUT_BUF_SIZE;
               }
               payload_ptr->actual_data_len = sizeof(capi_port_data_threshold_t);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_example_enc_module: Get property_id 0x%lx, Bad param size %lu",
                      (uint32_t)prop_array[i].id,
                      payload_ptr->max_data_len);
               payload_ptr->actual_data_len = 0;
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_INTERFACE_EXTENSIONS:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_NEEDED_FRAMEWORK_EXTENSIONS:
         {
            /* Since it does not support any framework or interface extensions*/
            break;
         }
         default:
         {
            AR_MSG(DBG_HIGH_PRIO,
                   "capi_example_enc_module: Get property for ID %#x. Not supported.",
                   prop_array[i].id);
            capi_result |= CAPI_EUNSUPPORTED;
            break;
         }
      }

      if ((CAPI_EOK != capi_result) && (CAPI_EUNSUPPORTED != capi_result))
      {
         CAPI_SET_ERROR(capi_result2, capi_result);
      }
   }
   return capi_result2;
}
