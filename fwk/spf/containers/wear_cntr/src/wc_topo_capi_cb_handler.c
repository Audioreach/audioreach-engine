/**
 * \file wc_topo_capi_cb_handler.c
 * \brief
 *     This file contains functions for WCNTR
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "wc_topo.h"
#include "wc_topo_capi.h"

/** GPR/API related */
#include "gpr_api_inline.h"
#include "gpr_ids_domains.h"

/** port field is invalid if its zero. */
#define COMPARE_FMT_AND_SET(                                                                                           \
   capi_med_fmt_ptr, field1, port_fmt_ptr, field2, is_mf_valid_and_changed_ptr, num_invalid_values)                    \
   if (WCNTR_IS_VALID_CAPI_VALUE(capi_med_fmt_ptr->field1))                                                                  \
   {                                                                                                                   \
      if (port_fmt_ptr->pcm.field2 != capi_med_fmt_ptr->field1)                                                        \
      {                                                                                                                \
         *is_mf_valid_and_changed_ptr = TRUE;                                                                          \
         port_fmt_ptr->pcm.field2     = capi_med_fmt_ptr->field1;                                                      \
      }                                                                                                                \
   }                                                                                                                   \
   else                                                                                                                \
   {                                                                                                                   \
      if (0 == port_fmt_ptr->pcm.field2)                                                                               \
      {                                                                                                                \
         num_invalid_values++;                                                                                         \
      }                                                                                                                \
   }

static capi_err_t wcntr_topo_handle_pcm_packetized_out_mf_event(capi_buf_t *       payload,
                                                              wcntr_topo_media_fmt_t * port_fmt_ptr,
                                                              capi_event_info_t *event_info_ptr,
                                                              bool_t *           is_mf_valid_and_changed_ptr)
{
   uint32_t num_invalid_values = 0;
   if ((NULL == payload) || (NULL == port_fmt_ptr) || (NULL == is_mf_valid_and_changed_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "received null pointer(s).");
      return AR_EBADPARAM;
   }
   capi_set_get_media_format_t *data_ptr = (capi_set_get_media_format_t *)(payload->data_ptr);

   if (payload->actual_data_len < (sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_t)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Payload size %lu is lesser than the min required size %lu",
             payload->actual_data_len,
             sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_t));
      return CAPI_ENEEDMORE;
   }

   if (!event_info_ptr->port_info.is_valid || event_info_ptr->port_info.is_input_port)
   {
      AR_MSG(DBG_ERROR_PRIO, "Port is not valid or is an input port");
      return CAPI_EBADPARAM;
   }

   capi_standard_data_format_t *capi_med_fmt_ptr =
      (capi_standard_data_format_t *)(payload->data_ptr + sizeof(capi_set_get_media_format_t));

   COMPARE_FMT_AND_SET(capi_med_fmt_ptr,
                       bits_per_sample,
                       port_fmt_ptr,
                       bits_per_sample,
                       is_mf_valid_and_changed_ptr,
                       num_invalid_values);

   if (WCNTR_IS_VALID_CAPI_VALUE(capi_med_fmt_ptr->q_factor))
   {
      if (port_fmt_ptr->pcm.q_factor != capi_med_fmt_ptr->q_factor)
      {
         port_fmt_ptr->pcm.q_factor   = capi_med_fmt_ptr->q_factor;
         port_fmt_ptr->pcm.bit_width  = WCNTR_TOPO_QFORMAT_TO_BIT_WIDTH(port_fmt_ptr->pcm.q_factor);
         *is_mf_valid_and_changed_ptr = TRUE;
      }
   }
   else
   {
      if (0 == port_fmt_ptr->pcm.q_factor)
      {
         num_invalid_values++;
      }
   }

   if (WCNTR_IS_VALID_CAPI_VALUE(capi_med_fmt_ptr->num_channels) && (capi_med_fmt_ptr->num_channels <= CAPI_MAX_CHANNELS))
   {
      if (port_fmt_ptr->pcm.num_channels != capi_med_fmt_ptr->num_channels)
      {
         port_fmt_ptr->pcm.num_channels = capi_med_fmt_ptr->num_channels;
         *is_mf_valid_and_changed_ptr   = TRUE;
      }

      for (uint32_t i = 0; ((i < SIZE_OF_ARRAY(port_fmt_ptr->pcm.chan_map)) && (i < capi_med_fmt_ptr->num_channels));
           i++)
      {
         if (port_fmt_ptr->pcm.chan_map[i] != capi_med_fmt_ptr->channel_type[i])
         {
            port_fmt_ptr->pcm.chan_map[i] = capi_med_fmt_ptr->channel_type[i];
            *is_mf_valid_and_changed_ptr  = TRUE;
         }
      }
   }
   else
   {
      if (0 == port_fmt_ptr->pcm.num_channels)
      {
         num_invalid_values++;
      }
   }

   COMPARE_FMT_AND_SET(capi_med_fmt_ptr,
                       sampling_rate,
                       port_fmt_ptr,
                       sample_rate,
                       is_mf_valid_and_changed_ptr,
                       num_invalid_values);

   if (CAPI_INVALID_INTERLEAVING != capi_med_fmt_ptr->data_interleaving)
   {
      uint16_t prev_interleaved = port_fmt_ptr->pcm.interleaving;
      port_fmt_ptr->pcm.interleaving =
         wcntr_topo_convert_capi_interleaving_to_topo_interleaving(capi_med_fmt_ptr->data_interleaving);

      if (prev_interleaved != port_fmt_ptr->pcm.interleaving)
      {
         *is_mf_valid_and_changed_ptr = TRUE;
      }
   }
   else
   {
      if (WCNTR_TOPO_INTERLEAVING_UNKNOWN == port_fmt_ptr->pcm.interleaving)
      {
         num_invalid_values++;
      }
   }

   port_fmt_ptr->pcm.endianness = WCNTR_TOPO_LITTLE_ENDIAN;

   // many CAPIs don't initialize this. hence assume by default if the data format is FIXED_POINT
   if (CAPI_FIXED_POINT == data_ptr->format_header.data_format)
   {
      if ((0 == capi_med_fmt_ptr->bitstream_format) ||
          (CAPI_DATA_FORMAT_INVALID_VAL == capi_med_fmt_ptr->bitstream_format))
      {
         capi_med_fmt_ptr->bitstream_format = MEDIA_FMT_ID_PCM;
      }
   }

   if (port_fmt_ptr->fmt_id != capi_med_fmt_ptr->bitstream_format)
   {
      *is_mf_valid_and_changed_ptr = TRUE;
      port_fmt_ptr->fmt_id         = capi_med_fmt_ptr->bitstream_format;
   }

   // if at least one value is invalid, then until that value becomes valid, don't propagate media fmt.
   if (num_invalid_values)
   {
      *is_mf_valid_and_changed_ptr = FALSE;
   }

   return AR_EOK;
}

static capi_err_t wcntr_topo_handle_pcm_packetized_v2_out_mf_event(capi_buf_t *       payload,
                                                                 wcntr_topo_media_fmt_t * port_fmt_ptr,
                                                                 capi_event_info_t *event_info_ptr,
                                                                 bool_t *           is_mf_valid_and_changed_ptr)
{
   uint32_t num_invalid_values = 0;

   if ((NULL == payload) || (NULL == port_fmt_ptr) || (NULL == is_mf_valid_and_changed_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "received null pointer(s).");
      return AR_EBADPARAM;
   }
   capi_set_get_media_format_t *data_ptr = (capi_set_get_media_format_t *)(payload->data_ptr);

   uint32_t min_req_size = sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t);
   if (payload->actual_data_len < min_req_size)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Payload size %lu is lesser than the min required size %lu",
             payload->actual_data_len,
             min_req_size);
      return CAPI_ENEEDMORE;
   }

   capi_standard_data_format_v2_t *capi_med_fmt_ptr =
      (capi_standard_data_format_v2_t *)(payload->data_ptr + sizeof(capi_set_get_media_format_t));

   // another size validation
   if (payload->actual_data_len < (min_req_size + (capi_med_fmt_ptr->num_channels * sizeof(capi_channel_type_t))))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Payload size %lu is lesser than the min required size %lu",
             payload->actual_data_len,
             min_req_size + (capi_med_fmt_ptr->num_channels * sizeof(capi_channel_type_t)));
      return CAPI_ENEEDMORE;
   }

   if (!event_info_ptr->port_info.is_valid || event_info_ptr->port_info.is_input_port)
   {
      AR_MSG(DBG_ERROR_PRIO, "Port is not valid or is an input port");
      return CAPI_EBADPARAM;
   }

   COMPARE_FMT_AND_SET(capi_med_fmt_ptr,
                       bits_per_sample,
                       port_fmt_ptr,
                       bits_per_sample,
                       is_mf_valid_and_changed_ptr,
                       num_invalid_values);

   if (WCNTR_IS_VALID_CAPI_VALUE(capi_med_fmt_ptr->q_factor))
   {
      if (port_fmt_ptr->pcm.q_factor != capi_med_fmt_ptr->q_factor)
      {
         port_fmt_ptr->pcm.q_factor   = capi_med_fmt_ptr->q_factor;
         port_fmt_ptr->pcm.bit_width  = WCNTR_TOPO_QFORMAT_TO_BIT_WIDTH(port_fmt_ptr->pcm.q_factor);
         *is_mf_valid_and_changed_ptr = TRUE;
      }
   }
   else
   {
      if (0 == port_fmt_ptr->pcm.q_factor)
      {
         num_invalid_values++;
      }
   }

   if (WCNTR_IS_VALID_CAPI_VALUE(capi_med_fmt_ptr->num_channels))
   {
      if (port_fmt_ptr->pcm.num_channels != capi_med_fmt_ptr->num_channels)
      {
         port_fmt_ptr->pcm.num_channels = capi_med_fmt_ptr->num_channels;
         *is_mf_valid_and_changed_ptr   = TRUE;
      }

      for (uint32_t i = 0; ((i < SIZE_OF_ARRAY(port_fmt_ptr->pcm.chan_map)) && (i < capi_med_fmt_ptr->num_channels));
           i++)
      {
         if (port_fmt_ptr->pcm.chan_map[i] != capi_med_fmt_ptr->channel_type[i])
         {
            port_fmt_ptr->pcm.chan_map[i] = capi_med_fmt_ptr->channel_type[i];
            *is_mf_valid_and_changed_ptr  = TRUE;
         }
      }
   }
   else
   {
      if (0 == port_fmt_ptr->pcm.num_channels)
      {
         num_invalid_values++;
      }
   }

   COMPARE_FMT_AND_SET(capi_med_fmt_ptr,
                       sampling_rate,
                       port_fmt_ptr,
                       sample_rate,
                       is_mf_valid_and_changed_ptr,
                       num_invalid_values);

   if (CAPI_INVALID_INTERLEAVING != capi_med_fmt_ptr->data_interleaving)
   {
      uint16_t prev_interleaved = port_fmt_ptr->pcm.interleaving;
      port_fmt_ptr->pcm.interleaving =
         wcntr_topo_convert_capi_interleaving_to_topo_interleaving(capi_med_fmt_ptr->data_interleaving);

      if (prev_interleaved != port_fmt_ptr->pcm.interleaving)
      {
         *is_mf_valid_and_changed_ptr = TRUE;
      }
   }
   else
   {
      if (WCNTR_TOPO_INTERLEAVING_UNKNOWN == port_fmt_ptr->pcm.interleaving)
      {
         num_invalid_values++;
      }
   }

   port_fmt_ptr->pcm.endianness = WCNTR_TOPO_LITTLE_ENDIAN;

   // many CAPIs don't initialize this. hence assume by default if the data format is FIXED_POINT
   if (CAPI_FIXED_POINT == data_ptr->format_header.data_format)
   {
      if ((0 == capi_med_fmt_ptr->bitstream_format) ||
          (CAPI_DATA_FORMAT_INVALID_VAL == capi_med_fmt_ptr->bitstream_format))
      {
         capi_med_fmt_ptr->bitstream_format = MEDIA_FMT_ID_PCM;
      }
   }

   if (port_fmt_ptr->fmt_id != capi_med_fmt_ptr->bitstream_format)
   {
      *is_mf_valid_and_changed_ptr = TRUE;
      port_fmt_ptr->fmt_id         = capi_med_fmt_ptr->bitstream_format;
   }

   // if at least one value is invalid, then until that value becomes valid, don't propagate media fmt.
   if (num_invalid_values)
   {
      *is_mf_valid_and_changed_ptr = FALSE;
   }

   return AR_EOK;
}

capi_err_t wcntr_topo_handle_output_media_format_event(void *             ctxt_ptr,
                                                     void *             module_ctxt_ptr,
                                                     capi_event_info_t *event_info_ptr,
                                                     bool_t             is_std_fmt_v2,
                                                     bool_t             is_pending_data_valid)
{
   capi_err_t                   result     = CAPI_EOK;
   wcntr_topo_t *                 topo_ptr   = (wcntr_topo_t *)ctxt_ptr;
   wcntr_topo_module_t *          module_ptr = (wcntr_topo_module_t *)module_ctxt_ptr;
   capi_buf_t *                 payload    = &event_info_ptr->payload;
   capi_set_get_media_format_t *data_ptr   = (capi_set_get_media_format_t *)(payload->data_ptr);
   uint32_t                     port_ind   = event_info_ptr->port_info.port_index;
   wcntr_topo_output_port_t *     out_port_ptr =
      (wcntr_topo_output_port_t *)wcntr_gu_find_output_port_by_index(&module_ptr->gu, port_ind);

   // TRUE if mf changed and all fields are valid values.
   bool_t is_mf_valid_and_changed = FALSE;

   WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
            DBG_HIGH_PRIO,
            "handle_out_mf_event: from miid 0x%lX START is_std_fmt_v2 %u  ",
            module_ptr->gu.module_instance_id,
            is_std_fmt_v2);

   if (NULL == out_port_ptr)
   {
      WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
               DBG_HIGH_PRIO,
               "Warning: Module 0x%lX doesnt have output port, but module raised output media format",
               module_ptr->gu.module_instance_id);
      return result;
   }

   bool_t            mf_event_pending = out_port_ptr->common.flags.media_fmt_event;
   wcntr_topo_media_fmt_t *port_fmt_ptr     = NULL;
   if (module_ptr->bypass_ptr) // only for single port modules.
   {
      port_fmt_ptr = &module_ptr->bypass_ptr->media_fmt;
   }
   else
   {
      port_fmt_ptr = &out_port_ptr->common.media_fmt;
   }



   //   uint32_t new_output_fmt_id = CAPI_DATA_FORMAT_INVALID_VAL;

   if ((CAPI_MAX_FORMAT_TYPE == data_ptr->format_header.data_format) ||
       (CAPI_DATA_FORMAT_INVALID_VAL == (uint32_t)data_ptr->format_header.data_format))
   {
      WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
               DBG_HIGH_PRIO,
               "Module 0x%lX: Port 0x%lx: module output media format: invalid data format 0x%lx. "
               "propagating? %u,",
               module_ptr->gu.module_instance_id,
               out_port_ptr->gu.cmn.id,
               data_ptr->format_header.data_format,
               out_port_ptr->common.flags.media_fmt_event);
      return CAPI_EFAILED;
   }

   spf_data_format_t wcntr_df =
      wcntr_topo_convert_capi_data_format_to_spf_data_format(data_ptr->format_header.data_format);

   if (wcntr_df != port_fmt_ptr->data_format)
   {
      is_mf_valid_and_changed   = TRUE;
      port_fmt_ptr->data_format = wcntr_df;
   }

   switch (data_ptr->format_header.data_format)
   {
      case CAPI_FIXED_POINT:
      {
         if (is_std_fmt_v2)
         {
            if (AR_EOK != (result = wcntr_topo_handle_pcm_packetized_v2_out_mf_event(payload,
                                                                                   port_fmt_ptr,
                                                                                   event_info_ptr,
                                                                                   &is_mf_valid_and_changed)))
            {
               WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_ERROR_PRIO,
                        "Module 0x%lX: Error in output MF V2 handling function",
                        module_ptr->gu.module_instance_id);
               return result;
            }
         }
         else
         {
            if (AR_EOK != (result = wcntr_topo_handle_pcm_packetized_out_mf_event(payload,
                                                                                port_fmt_ptr,
                                                                                event_info_ptr,
                                                                                &is_mf_valid_and_changed)))
            {
               WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_ERROR_PRIO,
                        "Module 0x%lX: Error in output MF V1 handling function",
                        module_ptr->gu.module_instance_id);
               return result;
            }
         }

         // update the change status only if the media format has changed, otherwise, 2 times event can reset
         // media_fmt_event flag.
         if (is_mf_valid_and_changed)
         {
            out_port_ptr->common.flags.media_fmt_event = is_mf_valid_and_changed;
            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                     DBG_HIGH_PRIO,
                     "Module 0x%lX: Port id 0x%lx: media_fmt_event flag is set ",
                     module_ptr->gu.module_instance_id,
                     out_port_ptr->gu.cmn.id);
         }
         break;
      }

      case CAPI_MAX_FORMAT_TYPE:
         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                  DBG_HIGH_PRIO,
                  "Module 0x%lX: Port id 0x%lx: module output media format: invalid data format. "
                  "propagating? %u,",
                  module_ptr->gu.module_instance_id,
                  out_port_ptr->gu.cmn.id,
                  out_port_ptr->common.flags.media_fmt_event);
         return CAPI_EUNSUPPORTED;
      default:
         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                  DBG_ERROR_PRIO,
                  "Module 0x%lX: Error in media format function. Changing media type not supported for format 0x%lX",
                  module_ptr->gu.module_instance_id,
                  data_ptr->format_header.data_format);
         return CAPI_EUNSUPPORTED;
   }

   WCNTR_TOPO_PRINT_MEDIA_FMT(topo_ptr->gu.log_id, module_ptr, out_port_ptr, port_fmt_ptr, "module output ");

   if (module_ptr->bypass_ptr)
   {
      // clear MF event when module is bypassed if event was not already existing
      if (!mf_event_pending)
      {
         out_port_ptr->common.flags.media_fmt_event = FALSE;
      }
   }
   else
   {
      out_port_ptr->common.flags.is_mf_valid = wcntr_topo_is_valid_media_fmt(&out_port_ptr->common.media_fmt);
   }

   // Handle event only if the module is not in suspended/stopped state
   if (!wcntr_topo_is_module_sg_stopped_or_suspended(module_ptr) && out_port_ptr->common.flags.media_fmt_event)
   {
      topo_ptr->capi_event_flag.media_fmt_event = TRUE;
   }

   WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
            DBG_HIGH_PRIO,
            "handle_out_mf_event: from miid 0x%lX END with current capi_event_flag 0x%lX ",
            module_ptr->gu.module_instance_id,topo_ptr->capi_event_flag.word);

   return result;
}

capi_err_t wcntr_topo_handle_process_state_event(wcntr_topo_t *       topo_ptr,
                                               wcntr_topo_module_t *module_ptr,
                                               capi_event_info_t *event_info_ptr)
{
   capi_err_t                  result   = CAPI_EOK;
   capi_buf_t *                payload  = &event_info_ptr->payload;
   capi_event_process_state_t *data_ptr = (capi_event_process_state_t *)payload->data_ptr;

   // Only update process state event if new process state is different than before.
   if (((!module_ptr->flags.disabled) && (FALSE == data_ptr->is_enabled)) ||
       ((module_ptr->flags.disabled) && (TRUE == data_ptr->is_enabled)))
   {
      module_ptr->flags.disabled = !data_ptr->is_enabled;
      WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
               DBG_HIGH_PRIO,
               SPF_LOG_PREFIX "Module 0x%lX: process state set to %lu",
               module_ptr->gu.module_instance_id,
               (uint32_t)(data_ptr->is_enabled));

      bool_t is_run_time = !wcntr_topo_is_module_sg_stopped_or_suspended(module_ptr);

      if (is_run_time)
      {
         topo_ptr->capi_event_flag.process_state = TRUE;
		  WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
               DBG_HIGH_PRIO,
               SPF_LOG_PREFIX "Module 0x%lX: called process state set to %lu in runtime",
               module_ptr->gu.module_instance_id,
               (uint32_t)(data_ptr->is_enabled));
      }

      if (module_ptr->flags.disabled)
      {
         result = wcntr_topo_check_create_bypass_module(topo_ptr, module_ptr);
      }
      else
      {
         result = wcntr_topo_check_destroy_bypass_module(topo_ptr, module_ptr);
      }
   }
   return result;
}

capi_err_t wcntr_topo_capi_callback(void *context_ptr, capi_event_id_t id, capi_event_info_t *event_info_ptr)
{
   capi_err_t result = CAPI_EOK;

   if (!context_ptr || !event_info_ptr)
   {
      return CAPI_EFAILED;
   }

   capi_buf_t *         payload    = &event_info_ptr->payload;
   wcntr_topo_module_t *module_ptr = (wcntr_topo_module_t *)(context_ptr);
   wcntr_topo_t *       topo_ptr   = module_ptr->topo_ptr;

   if (!topo_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Error in callback function. Data possibly corrupted. ");
      return CAPI_EFAILED;
   }

   if (payload->actual_data_len > payload->max_data_len)
   {
      WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                     DBG_ERROR_PRIO,
                     "Module 0x%lX: Error in callback function. The actual size %lu is greater than the max size %lu "
                     "for "
                     "id %lu.",
                     module_ptr->gu.module_instance_id,
                     payload->actual_data_len,
                     payload->max_data_len,
                     (uint32_t)(id));
      return CAPI_EBADPARAM;
   }

   switch (id)
   {
      case CAPI_EVENT_KPPS:
      {
         if (payload->actual_data_len < sizeof(capi_event_KPPS_t))
         {
            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_ERROR_PRIO,
                           "Module 0x%lX: Error in callback function. The actual size %lu is less than the required "
                           "size "
                           "%zu for id %lu.",
                           module_ptr->gu.module_instance_id,
                           payload->actual_data_len,
                           sizeof(capi_event_bandwidth_t),
                           (uint32_t)(id));
            return CAPI_ENEEDMORE;
         }

         capi_event_KPPS_t *kpps_ptr = (capi_event_KPPS_t *)(payload->data_ptr);

         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_HIGH_PRIO,
                        "topo_capi_callback : Module 0x%lX: called CAPI_EVENT_KPPS with KPPS  %lu",
                        module_ptr->gu.module_instance_id,
                        (uint32_t)(kpps_ptr->KPPS));

         if (module_ptr->bypass_ptr)
         {
            module_ptr->bypass_ptr->kpps = kpps_ptr->KPPS;
         }
         else
         {
            if (module_ptr->kpps != kpps_ptr->KPPS)
            {
               module_ptr->kpps = kpps_ptr->KPPS;
               if (!wcntr_topo_is_module_sg_stopped_or_suspended(module_ptr))
               {
                  topo_ptr->capi_event_flag.kpps = TRUE;
               }
            }
         }

         return CAPI_EOK;
      }

      case CAPI_EVENT_BANDWIDTH:
      {
         if (payload->actual_data_len < sizeof(capi_event_bandwidth_t))
         {
            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_ERROR_PRIO,
                           "Module 0x%lX: Error in callback function. The actual size %lu is less than the required "
                           "size "
                           "%zu for id %lu.",
                           module_ptr->gu.module_instance_id,
                           payload->actual_data_len,
                           sizeof(capi_event_process_state_t),
                           (uint32_t)(id));
            return CAPI_ENEEDMORE;
         }

         capi_event_bandwidth_t *bw_ptr = (capi_event_bandwidth_t *)(payload->data_ptr);
         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_HIGH_PRIO,
                        "topo_capi_callback : Module 0x%lX: called CAPI_EVENT_BANDWIDTH with (code_bw,data_bw) "
                        "(%lu,%lu)",
                        module_ptr->gu.module_instance_id,
                        bw_ptr->code_bandwidth,
                        bw_ptr->data_bandwidth);

         if (module_ptr->bypass_ptr)
         {
            module_ptr->bypass_ptr->code_bw = bw_ptr->code_bandwidth;
            module_ptr->bypass_ptr->data_bw = bw_ptr->data_bandwidth;
         }
         else
         {
            if (module_ptr->code_bw != bw_ptr->code_bandwidth)
            {
               module_ptr->code_bw = bw_ptr->code_bandwidth;
               if (!wcntr_topo_is_module_sg_stopped_or_suspended(module_ptr))
               {
                  topo_ptr->capi_event_flag.bw = TRUE;
               }
            }
            if (module_ptr->data_bw != bw_ptr->data_bandwidth)
            {
               module_ptr->data_bw = bw_ptr->data_bandwidth;
               if (!wcntr_topo_is_module_sg_stopped_or_suspended(module_ptr))
               {
                  topo_ptr->capi_event_flag.bw = TRUE;
               }
            }
         }
         return CAPI_EOK;
      }

      case CAPI_EVENT_OUTPUT_MEDIA_FORMAT_UPDATED_V2:
      {
         if (payload->actual_data_len < sizeof(capi_set_get_media_format_t))
         {
            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_ERROR_PRIO,
                           "Module 0x%lX: Error in callback function. The actual size %lu is less than the required "
                           "size "
                           "%zu for id %lu.",
                           module_ptr->gu.module_instance_id,
                           payload->actual_data_len,
                           sizeof(capi_set_get_media_format_t),
                           (uint32_t)(id));
            return CAPI_ENEEDMORE;
         }

         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_HIGH_PRIO,
                        "topo_capi_callback : Module 0x%lX: called CAPI_EVENT_OUTPUT_MEDIA_FORMAT_UPDATED_V2",
                        module_ptr->gu.module_instance_id);

         return wcntr_topo_handle_output_media_format_event(topo_ptr, module_ptr, event_info_ptr, TRUE, FALSE);
      }
      case CAPI_EVENT_OUTPUT_MEDIA_FORMAT_UPDATED:
      {
         if (payload->actual_data_len < sizeof(capi_set_get_media_format_t))
         {
            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_ERROR_PRIO,
                           "Module 0x%lX: Error in callback function. The actual size %lu is less than the required "
                           "size "
                           "%zu for id %lu.",
                           module_ptr->gu.module_instance_id,
                           payload->actual_data_len,
                           sizeof(capi_set_get_media_format_t),
                           (uint32_t)(id));
            return CAPI_ENEEDMORE;
         }

         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_HIGH_PRIO,
                        "topo_capi_callback :Module 0x%lX: called CAPI_EVENT_OUTPUT_MEDIA_FORMAT_UPDATED",
                        module_ptr->gu.module_instance_id);

         return wcntr_topo_handle_output_media_format_event(topo_ptr, module_ptr, event_info_ptr, FALSE, FALSE);
      }

      case CAPI_EVENT_PORT_DATA_THRESHOLD_CHANGE:
      {
         // A module implementing frame duration extension must always use 1 byte threshold. Not doing so can result in
         // inf loop.
         // container aggregates threshold, sets frame dur on module and then module raises threshold, which in turn
         // makes fwk set cntr frame len to module. For receiving configured threshold need_thresh_cfg ext should be
         // used. Test Module does this and hence not failing for now.
         if (module_ptr->flags.need_cntr_frame_dur_extn)
         {
            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_ERROR_PRIO,
                           "Module 0x%lX: Warning: A module that needs container frame duration extension must not "
                           "raise "
                           "threshold.",
                           module_ptr->gu.module_instance_id);
         }

         if (payload->actual_data_len < sizeof(capi_port_data_threshold_change_t))
         {
            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_ERROR_PRIO,
                           "Module 0x%lX: Error in callback function. The actual size %lu is less than the required "
                           "size "
                           "%zu for id %lu.",
                           module_ptr->gu.module_instance_id,
                           payload->actual_data_len,
                           sizeof(capi_port_data_threshold_change_t),
                           (uint32_t)(id));
            return CAPI_ENEEDMORE;
         }

         if (!event_info_ptr->port_info.is_valid)
         {
            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_ERROR_PRIO,
                           "Module 0x%lX: Error in callback function. port is not valid ",
                           module_ptr->gu.module_instance_id);
            return CAPI_EBADPARAM;
         }

         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_HIGH_PRIO,
                        "topo_capi_callback :Module 0x%lX: called CAPI_EVENT_PORT_DATA_THRESHOLD_CHANGE",
                        module_ptr->gu.module_instance_id);

         capi_port_data_threshold_change_t *data_ptr = (capi_port_data_threshold_change_t *)(payload->data_ptr);
         if (event_info_ptr->port_info.is_input_port)
         {
            wcntr_topo_input_port_t *in_port_ptr =
               (wcntr_topo_input_port_t *)wcntr_gu_find_input_port_by_index(&module_ptr->gu,
                                                                            event_info_ptr->port_info.port_index);

            if (!in_port_ptr)
            {
               WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                              DBG_ERROR_PRIO,
                              "Module 0x%lX: Input port index 0x%lx not present",
                              module_ptr->gu.module_instance_id,
                              event_info_ptr->port_info.port_index);
               return CAPI_EFAILED;
            }

            if (module_ptr->bypass_ptr)
            {
               module_ptr->bypass_ptr->in_thresh_bytes_all_ch = data_ptr->new_threshold_in_bytes;

               WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                              DBG_HIGH_PRIO,
                              "Module 0x%lX: Input port id 0x%lx event with new port-threshold = %lu raised while "
                              "module is "
                              "bypassed, caching to handle later.",
                              module_ptr->gu.module_instance_id,
                              in_port_ptr->gu.cmn.id,
                              data_ptr->new_threshold_in_bytes);
            }
            else
            {
               /** Mark threshold event asa */
               if (in_port_ptr->common.port_event_new_threshold != data_ptr->new_threshold_in_bytes)
               {

                  if (in_port_ptr->common.bufs_ptr && in_port_ptr->common.bufs_ptr[0].actual_data_len != 0)
                  {
                     WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                                    DBG_HIGH_PRIO,
                                    "Module 0x%lX: Warning: Threshold changed when buffer had valid data %lu",
                                    module_ptr->gu.module_instance_id,
                                    in_port_ptr->common.bufs_ptr[0].actual_data_len);
                  }

                  in_port_ptr->common.port_event_new_threshold = data_ptr->new_threshold_in_bytes;
                  if (in_port_ptr->common.port_event_new_threshold > 1)
                  {
                     in_port_ptr->common.flags.port_has_threshold = TRUE;
                  }
                  else
                  {
                     in_port_ptr->common.flags.port_has_threshold = FALSE;
                  }
                  if (!wcntr_topo_is_module_sg_stopped_or_suspended(module_ptr))
                  {
                     topo_ptr->capi_event_flag.port_thresh = TRUE;
                  }

                  WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                                 DBG_HIGH_PRIO,
                                 "Module 0x%lX: Input port id 0x%lx event with new port-threshold = %lu bytes",
                                 module_ptr->gu.module_instance_id,
                                 in_port_ptr->gu.cmn.id,
                                 data_ptr->new_threshold_in_bytes);
               }
            }
         }
         else
         {
            wcntr_topo_output_port_t *out_port_ptr =
               (wcntr_topo_output_port_t *)wcntr_gu_find_output_port_by_index(&module_ptr->gu,
                                                                              event_info_ptr->port_info.port_index);

            if (!out_port_ptr)
            {
               WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                              DBG_ERROR_PRIO,
                              "Module 0x%lX: Output port index %lu not present",
                              module_ptr->gu.module_instance_id,
                              event_info_ptr->port_info.port_index);
               return CAPI_EFAILED;
            }

            if (module_ptr->bypass_ptr)
            {
               module_ptr->bypass_ptr->out_thresh_bytes_all_ch = data_ptr->new_threshold_in_bytes;

               WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                              DBG_HIGH_PRIO,
                              "Module 0x%lX: Output port id 0x%lx event with new port-threshold = %lu raised while "
                              "module is "
                              "bypassed, caching to handle later.",
                              module_ptr->gu.module_instance_id,
                              out_port_ptr->gu.cmn.id,
                              data_ptr->new_threshold_in_bytes);
            }
            else
            {
               if (out_port_ptr->common.port_event_new_threshold != data_ptr->new_threshold_in_bytes)
               {

                  if (out_port_ptr->common.bufs_ptr && out_port_ptr->common.bufs_ptr[0].actual_data_len != 0)
                  {
                     WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                                    DBG_HIGH_PRIO,
                                    "Module 0x%lX: Warning: Threshold changed when buffer had valid data %lu",
                                    module_ptr->gu.module_instance_id,
                                    out_port_ptr->common.bufs_ptr[0].actual_data_len);
                  }

                  out_port_ptr->common.port_event_new_threshold = data_ptr->new_threshold_in_bytes;
                  if (out_port_ptr->common.port_event_new_threshold > 1)
                  {
                     out_port_ptr->common.flags.port_has_threshold = TRUE;
                  }
                  else
                  {
                     out_port_ptr->common.flags.port_has_threshold = FALSE;
                  }
                  if (!wcntr_topo_is_module_sg_stopped_or_suspended(module_ptr))
                  {
                     topo_ptr->capi_event_flag.port_thresh = TRUE;
                  }

                  WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                                 DBG_HIGH_PRIO,
                                 "Module 0x%lX: Output port id 0x%lx event with new port-threshold = %lu.",
                                 module_ptr->gu.module_instance_id,
                                 out_port_ptr->gu.cmn.id,
                                 data_ptr->new_threshold_in_bytes);
               }
            }
         }

         return CAPI_EOK;
      }

      case CAPI_EVENT_ALGORITHMIC_DELAY:
      {
         capi_event_algorithmic_delay_t *delay_ptr =
            (capi_event_algorithmic_delay_t *)(event_info_ptr->payload.data_ptr);

         if (module_ptr->bypass_ptr)
         {
            if (module_ptr->bypass_ptr->algo_delay != delay_ptr->delay_in_us)
            {
               module_ptr->bypass_ptr->algo_delay = delay_ptr->delay_in_us;
               WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                              DBG_HIGH_PRIO,
                              "Module 0x%lX: wcntr_topo_capi_cb_func (bypass): updated algo delay(%lu) us",
                              module_ptr->gu.module_instance_id,
                              module_ptr->bypass_ptr->algo_delay);
            }
         }
         else
         {
            if (module_ptr->algo_delay != delay_ptr->delay_in_us)
            {
               module_ptr->algo_delay = delay_ptr->delay_in_us;
               WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                              DBG_HIGH_PRIO,
                              "topo_capi_callback :Module 0x%lX: called updated CAPI_EVENT_ALGORITHMIC_DELAY with algo "
                              "delay(%lu) us",
                              module_ptr->gu.module_instance_id,
                              module_ptr->algo_delay);
            }
         }

         break;
      }
      case CAPI_EVENT_DATA_TO_DSP_CLIENT:
      case CAPI_EVENT_DATA_TO_DSP_CLIENT_V2:
      {
         return ar_result_to_capi_err(
            topo_ptr->topo_to_cntr_vtable_ptr->handle_capi_event(module_ptr, id, event_info_ptr));
      }

      case CAPI_EVENT_GET_DATA_FROM_DSP_SERVICE:
      {
         if (payload->actual_data_len < sizeof(capi_event_get_data_from_dsp_service_t))
         {
            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_ERROR_PRIO,
                           "Module 0x%lX: Error in callback function. The actual size %lu is less than the required "
                           "size "
                           "%lu for id %lu.",
                           module_ptr->gu.module_instance_id,
                           payload->actual_data_len,
                           sizeof(capi_event_get_data_from_dsp_service_t),
                           (uint32_t)(id));
            return CAPI_ENEEDMORE;
         }
         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_HIGH_PRIO,
                        "topo_capi_callback  Module 0x%lX: called CAPI_EVENT_GET_DATA_FROM_DSP_SERVICE",
                        module_ptr->gu.module_instance_id);

         return ar_result_to_capi_err(
            topo_ptr->topo_to_cntr_vtable_ptr->raise_data_from_dsp_service_event(module_ptr, event_info_ptr));
      }

      case CAPI_EVENT_PROCESS_STATE:
      {
         if (payload->actual_data_len < sizeof(capi_event_process_state_t))
         {
            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_ERROR_PRIO,
                           "Module 0x%lX: Error in callback function. The actual size %lu is less than the required "
                           "size "
                           "%zu for "
                           "id %lu.",
                           module_ptr->gu.module_instance_id,
                           payload->actual_data_len,
                           sizeof(capi_event_process_state_t),
                           (uint32_t)(id));
            return CAPI_ENEEDMORE;
         }

         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_HIGH_PRIO,
                        "topo_capi_callback_base : Module 0x%lX: called CAPI_EVENT_PROCESS_STATE",
                        module_ptr->gu.module_instance_id);
         return wcntr_topo_handle_process_state_event(topo_ptr, module_ptr, event_info_ptr);
      }
      case CAPI_EVENT_DATA_TO_DSP_SERVICE:
      {
         if (payload->actual_data_len < sizeof(capi_event_data_to_dsp_service_t))
         {
            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_ERROR_PRIO,
                           "Module 0x%lX: Error in callback function. The actual size %lu is less than the required "
                           "size "
                           "%lu for id %lu.",
                           module_ptr->gu.module_instance_id,
                           payload->actual_data_len,
                           sizeof(capi_event_data_to_dsp_service_t),
                           (uint32_t)(id));
            return CAPI_ENEEDMORE;
         }

         // Some events can be handled within topo itself
         capi_buf_t *                      payload       = &event_info_ptr->payload;
         capi_event_data_to_dsp_service_t *dsp_event_ptr = (capi_event_data_to_dsp_service_t *)(payload->data_ptr);

         switch (dsp_event_ptr->param_id)
         {

            case FWK_EXTN_EVENT_ID_DATA_TRIGGER_IN_ST_CNTR:
            {
               WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                              DBG_ERROR_PRIO,
                              "Module 0x%lX: Error in callback function. ID %lu not supported.",
                              module_ptr->gu.module_instance_id,
                              (uint32_t)(id));
               break;
            }
            default:
            {
               return ar_result_to_capi_err(
                  topo_ptr->topo_to_cntr_vtable_ptr->raise_data_to_dsp_service_event(module_ptr, event_info_ptr));
            }
         }
         break;
      }
      default:
      {
         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_ERROR_PRIO,
                        "Module 0x%lX: Error in callback function. ID %lu not supported.",
                        module_ptr->gu.module_instance_id,
                        (uint32_t)(id));
         return CAPI_EUNSUPPORTED;
      }
   }

   return result;
}
