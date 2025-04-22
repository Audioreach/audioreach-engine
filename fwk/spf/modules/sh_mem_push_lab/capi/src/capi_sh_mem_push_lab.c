/* ===========================================================================
  @file capi_sh_mem_push_lab.c
  @brief This file contains CAPI implementation of shared memory
         push lab Module

   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

/*--------------------------------------------------------------------------
 * Include files and Macro definitions
 * ------------------------------------------------------------------------ */
#include "capi_sh_mem_push_lab.h"
#include "sh_mem_push_lab_api.h"
#include "push_lab.h"
#include "ar_msg.h"
#include "capi.h"
#include "capi_properties.h"
#include "imcl_dam_detection_api.h"

#define CAPI_PUSH_LAB_STACK_SIZE 4096 // TODO: To be measured

/* Number of CAPI Framework extension needed */
#define PM_NUM_FRAMEWORK_EXTENSIONS 1

/*------------------------------------------------------------------------
 * Static declarations
 * -----------------------------------------------------------------------*/

static inline uint32_t align_to_8_byte(const uint32_t num)
{
   return ((num + 7) & (0xFFFFFFF8));
}

static capi_err_t capi_push_lab_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);

static capi_err_t capi_push_lab_end(capi_t *_pif);

static capi_err_t capi_push_lab_set_param(capi_t *                _pif,
                                          uint32_t                param_id,
                                          const capi_port_info_t *port_info_ptr,
                                          capi_buf_t *            params_ptr);

static capi_err_t capi_push_lab_get_param(capi_t *                _pif,
                                          uint32_t                param_id,
                                          const capi_port_info_t *port_info_ptr,
                                          capi_buf_t *            params_ptr);

static capi_err_t capi_push_lab_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_err_t capi_push_lab_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static const capi_vtbl_t push_lab_vtbl = { capi_push_lab_process,        capi_push_lab_end,
                                           capi_push_lab_set_param,      capi_push_lab_get_param,
                                           capi_push_lab_set_properties, capi_push_lab_get_properties };

static capi_err_t capi_push_lab_process_get_properties(capi_push_lab_t *me_ptr, capi_proplist_t *proplist_ptr);

static capi_err_t capi_push_lab_raise_output_media_fmt_event(capi_push_lab_t *me_ptr);

static capi_err_t capi_push_lab_raise_output_media_fmt_event(capi_push_lab_t *me_ptr)
{
   capi_err_t result = CAPI_EOK;

   capi_media_fmt_v2_t media_fmt;
   media_fmt.header.format_header.data_format = CAPI_FIXED_POINT;
   media_fmt.format.bitstream_format          = me_ptr->push_lab_info.media_fmt.fmt_id;
   media_fmt.format.bits_per_sample           = me_ptr->push_lab_info.media_fmt.bits_per_sample;
   media_fmt.format.num_channels              = me_ptr->push_lab_info.media_fmt.num_channels;
   media_fmt.format.q_factor                  = me_ptr->push_lab_info.media_fmt.Q_format;
   media_fmt.format.data_is_signed            = me_ptr->push_lab_info.media_fmt.is_signed;
   media_fmt.format.sampling_rate             = me_ptr->push_lab_info.media_fmt.sample_rate;
   media_fmt.format.data_interleaving         = CAPI_DEINTERLEAVED_UNPACKED;

   for (uint32_t i = 0; i < media_fmt.format.num_channels; i++)
   {
      media_fmt.channel_type[i] = me_ptr->push_lab_info.media_fmt.channel_map[i];
   }

   result = capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info, &media_fmt, FALSE, 0);

   return result;
}

uint32_t sh_mem_push_lab_us_to_bytes(capi_push_lab_t *me_ptr, uint64_t time_us)
{
   uint32_t num_channels_to_write = push_lab_get_num_out_channels_to_write(&(me_ptr->push_lab_info));

   uint32_t bps = (me_ptr->push_lab_info.media_fmt.sample_rate) * (num_channels_to_write) *
                  ((me_ptr->push_lab_info.media_fmt.bits_per_sample) / 8);

   return ((time_us * bps) / 1000000);
}

capi_err_t capi_push_lab_raise_watermark_event(capi_push_lab_t *                      me,
                                               event_cfg_sh_mem_push_lab_watermark_t *water_mark_event)
{
   capi_err_t                         result = CAPI_EOK;
   capi_event_info_t                  event_info;
   capi_event_data_to_dsp_client_v2_t event;

   if (0 != me->push_lab_info.watermark_event_client_info.watermark_event_dest_address)
   {
      event.event_id                     = EVENT_ID_SH_MEM_PUSH_LAB_WATERMARK;
      event.payload.actual_data_len      = sizeof(event_cfg_sh_mem_push_lab_watermark_t);
      event.payload.max_data_len         = sizeof(event_cfg_sh_mem_push_lab_watermark_t);
      event.payload.data_ptr             = (int8_t *)water_mark_event;
      event_info.port_info.is_valid      = false;
      event_info.payload.actual_data_len = sizeof(capi_event_data_to_dsp_client_v2_t);
      event_info.payload.data_ptr        = (int8_t *)&event;
      event_info.payload.max_data_len    = sizeof(capi_event_data_to_dsp_client_v2_t);
      event.dest_address                 = me->push_lab_info.watermark_event_client_info.watermark_event_dest_address;
      event.token                        = me->push_lab_info.watermark_event_client_info.watermark_event_token;

      result = me->cb_info.event_cb(me->cb_info.event_context, CAPI_EVENT_DATA_TO_DSP_CLIENT_V2, &event_info);
      if (CAPI_EOK != result)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "capi_push_lab_raise_watermark_event: Failed to raise event for "
                "watermark");
      }
   }

   return result;
}

capi_err_t capi_push_lab_populate_payload_raise_watermark_event(capi_push_lab_t *capi_ptr)
{
   capi_err_t                            result = CAPI_EOK;
   push_lab_t *                          pm_ptr = &(capi_ptr->push_lab_info);
   event_cfg_sh_mem_push_lab_watermark_t custom_water_mark_event;
   custom_water_mark_event.start_index                  = pm_ptr->start_index;
   custom_water_mark_event.current_write_position_index = pm_ptr->current_write_index;
   result = capi_push_lab_raise_watermark_event(capi_ptr, &custom_water_mark_event);
   if (CAPI_FAILED(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: check_send_watermark_event: Failed to send water mark event!");
   }
   else
   {
      AR_MSG(DBG_LOW_PRIO, "capi_push_lab: check_send_watermark_event: Sent Water Mark event to the client");
   }
   return result;
}

static capi_err_t capi_create_port_structures(capi_push_lab_t *me_ptr)
{
   // Allocate memory for control port info structures
   uint32_t alloc_size = PUSH_LAB_MAX_CTRL_PORT * sizeof(imcl_port_info_t);
   me_ptr->imcl_port_info_arr =
      (imcl_port_info_t *)posal_memory_malloc(alloc_size, (POSAL_HEAP_ID)(me_ptr->heap_mem.heap_id));
   if (NULL == me_ptr->imcl_port_info_arr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Couldn't allocate memory for control port info.");
      return CAPI_ENOMEMORY;
   }
   memset(me_ptr->imcl_port_info_arr, 0, alloc_size);

   return CAPI_EOK;
}

capi_err_t capi_sh_mem_push_lab_get_static_properties(capi_proplist_t *init_set_properties,
                                                      capi_proplist_t *static_properties)
{
   AR_MSG(DBG_LOW_PRIO, "capi_push_lab: Enter get static prop");

   capi_err_t capi_result = CAPI_EOK;

   if (NULL != static_properties)
   {
      capi_result = capi_push_lab_process_get_properties((capi_push_lab_t *)NULL, static_properties);

      if (CAPI_FAILED(capi_result))
      {
         return capi_result;
      }
   }

   return capi_result;
}

static capi_err_t capi_push_lab_process_init(capi_push_lab_t *me_ptr, capi_proplist_t *init_set_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   AR_MSG(DBG_LOW_PRIO, "capi_push_lab: Enter init---------");

   if (NULL == me_ptr || NULL == init_set_properties)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Init received bad pointer, 0x%lx, 0x%lx",
             (uint32_t)me_ptr,
             (uint32_t)init_set_properties);

      return CAPI_EBADPARAM;
   }
   capi_result = capi_push_lab_set_properties((capi_t *)me_ptr, init_set_properties);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Set basic properties failed with result %lu", capi_result);
      return capi_result;
   }

   return capi_result;
}

capi_err_t capi_sh_mem_push_lab_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t capi_result = CAPI_EOK;

   AR_MSG(DBG_HIGH_PRIO, "capi_push_lab: Enter capi_push_lab_init------");

   if (NULL == _pif || NULL == init_set_properties)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Init received bad pointer, 0x%lx, 0x%lx", (uint32_t)_pif, (uint32_t)init_set_properties);

      return CAPI_EBADPARAM;
   }

   capi_push_lab_t *me_ptr = (capi_push_lab_t *)_pif;
   memset((void *)me_ptr, 0, sizeof(capi_push_lab_t));

   me_ptr->vtbl.vtbl_ptr = &push_lab_vtbl;

   memset(&me_ptr->push_lab_info.media_fmt, 0, sizeof(push_lab_media_fmt_t));

   me_ptr->push_lab_info.media_fmt.endianness = PCM_LITTLE_ENDIAN;
   me_ptr->push_lab_info.media_fmt.alignment  = PCM_LSB_ALIGNED;
   AR_MSG(DBG_HIGH_PRIO,
          "capi_push_lab: Initialized with PCM_LITTLE_ENDIAN to populate endianness if container does not support framework "
          "extension");

   capi_result = capi_push_lab_process_init(me_ptr, init_set_properties);

   return capi_result;
}

static capi_err_t capi_push_lab_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t       result = CAPI_EFAILED;
   capi_push_lab_t *me_ptr = (capi_push_lab_t *)_pif;
   capi_buf_t *     p_module_buffer;
   uint32_t         i = 0;

   /*TODO - Similar to VW. Check if CAPI_PORT_NUM_INFO get property is needed and
    * use a for loop for each port*/
   capi_stream_data_t *inp_buf_list = input[0];
   capi_buf_t *        inp_buf_ptr  = NULL;
   capi_stream_data_t *out_buf_list = NULL;
   capi_buf_t *        out_buf_ptr  = NULL;

   if (me_ptr->push_lab_info.is_disabled)
   {
      if (me_ptr->port_info.num_output_ports && out_buf_list)
      {
         AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Module disabled but doing a memcpy for process call");
         for (i = 0; i < inp_buf_list->bufs_num; i++)
         {
            inp_buf_ptr = inp_buf_list->buf_ptr + i;
            out_buf_ptr = out_buf_list->buf_ptr + i;

            if ((NULL != inp_buf_ptr) && (NULL != out_buf_ptr))
            {
               memscpy(out_buf_ptr->data_ptr,
                       out_buf_ptr->max_data_len,
                       inp_buf_ptr->data_ptr,
                       inp_buf_ptr->actual_data_len);

               out_buf_ptr->actual_data_len = inp_buf_ptr->actual_data_len;
            }
            else
            {
               return CAPI_EFAILED;
            }
         }
      }
   }

   uint64_t timestamp;
   if ((*input)->flags.is_timestamp_valid)
   {
      timestamp = (*input)->timestamp;
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: received invalid capture timestamp.");
      // just print an error since API definition expects capture timestamp.

      timestamp = posal_timer_get_time();
   }

   p_module_buffer = (capi_buf_t *)(*input)->buf_ptr;

   // enable this for any need to check data reaching process call
   /*   FILE *fp;
      fp = fopen("file.pcm", "a");
      fwrite(p_module_buffer->data_ptr, p_module_buffer->actual_data_len, 1, fp);
      fclose(fp);*/

   result = push_lab_write_output(me_ptr, p_module_buffer, timestamp);

   if (me_ptr->port_info.num_output_ports)
   {
      // if Push Lab is not the sink module
      out_buf_list = output[0];
   }

   if (0 == inp_buf_list->bufs_num)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Invalid Param: Input num_bufs");
      return CAPI_EFAILED;
   }

   // if Push lab is not the SINK
   if (me_ptr->port_info.num_output_ports && out_buf_list)
   {
      // Pass through the data.
      for (i = 0; i < inp_buf_list->bufs_num; i++)
      {
         inp_buf_ptr = inp_buf_list->buf_ptr + i;
         out_buf_ptr = out_buf_list->buf_ptr + i;

         if ((NULL != inp_buf_ptr) && (NULL != out_buf_ptr))
         {
            memscpy(out_buf_ptr->data_ptr,
                    out_buf_ptr->max_data_len,
                    inp_buf_ptr->data_ptr,
                    inp_buf_ptr->actual_data_len);

            out_buf_ptr->actual_data_len = inp_buf_ptr->actual_data_len;
         }
         else
         {
            return CAPI_EFAILED;
         }
      }
   }

   return result;
}

static capi_err_t capi_push_lab_end(capi_t *_pif)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == _pif)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: End received bad pointer, 0x%lx", (uint32_t)_pif);
      return CAPI_EBADPARAM;
   }
   capi_push_lab_t *me_ptr = (capi_push_lab_t *)_pif;

   push_lab_deinit(&(me_ptr->push_lab_info));

   if (me_ptr->imcl_port_info_arr)
   {
      posal_memory_free(me_ptr->imcl_port_info_arr);
   }

   me_ptr->vtbl.vtbl_ptr = NULL;

   return capi_result;
}

static capi_err_t capi_push_lab_set_param(capi_t *                _pif,
                                          uint32_t                param_id,
                                          const capi_port_info_t *port_info_ptr,
                                          capi_buf_t *            params_ptr)
{
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi_push_lab: Set param received bad pointer, 0x%lx, 0x%lx",
             (uint32_t)_pif,
             (uint32_t)params_ptr);

      return CAPI_EBADPARAM;
   }

   capi_err_t       capi_result = CAPI_EOK;
   capi_push_lab_t *me_ptr      = (capi_push_lab_t *)_pif;

   void *param_payload_ptr = (void *)params_ptr->data_ptr;

   if (NULL == param_payload_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Set param received NULL payload pointer");

      return CAPI_EBADPARAM;
   }

   AR_MSG(DBG_HIGH_PRIO, "capi_push_lab: set param received for param id, 0x%lX", param_id);

   switch (param_id)
   {
      case PARAM_ID_SH_MEM_PUSH_LAB_CFG:
      {
         if (params_ptr->actual_data_len < sizeof(sh_mem_push_lab_cfg_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi_push_lab: Set param param id 0x%lX. Size not enough %lu",
                   param_id,
                   params_ptr->actual_data_len);
            capi_result = CAPI_EBADPARAM;
         }
         else
         {
            sh_mem_push_lab_cfg_t *push_lab_cfg_ptr = (sh_mem_push_lab_cfg_t *)params_ptr->data_ptr;
            capi_result                             = push_lab_init(&(me_ptr->push_lab_info), push_lab_cfg_ptr);
            if (capi_result)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Set param failed for param id 0x%lX, result %d", param_id, capi_result);
            }
            AR_MSG(DBG_ERROR_PRIO, "capi_push_lab_set_param: PARAM_ID_SH_MEM_PUSH_LAB_CFG push_lab_init done");
         }
         break;
      }
      case PARAM_ID_MEDIA_FORMAT:
      {
         if (params_ptr->actual_data_len < sizeof(media_format_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi_push_lab: Set param param id 0x%lX. Size not enough %lu",
                   param_id,
                   params_ptr->actual_data_len);
            capi_result = CAPI_EBADPARAM;
         }
         else
         {
            media_format_t *media_fmt = (media_format_t *)params_ptr->data_ptr;
            if ((MEDIA_FMT_ID_PCM != media_fmt->fmt_id) || (DATA_FORMAT_FIXED_POINT != media_fmt->data_format))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_push_lab: Set param param id 0x%lX. Invalid fmt_id 0x%lX OR "
                      "data_format 0x%lX",
                      param_id,
                      media_fmt->fmt_id,
                      media_fmt->data_format);
               capi_result = CAPI_EBADPARAM;
            }
            else
            {
               if ((params_ptr->actual_data_len < (sizeof(media_format_t) + sizeof(payload_media_fmt_pcm_t))) ||
                   (media_fmt->payload_size < sizeof(payload_media_fmt_pcm_t)))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi_push_lab: Set param param id 0x%lX. Size not enough %lu. "
                         "media_fmt->payload_size = %lu",
                         param_id,
                         sizeof(media_format_t) + sizeof(payload_media_fmt_pcm_t),
                         media_fmt->payload_size);
                  capi_result = CAPI_EBADPARAM;
               }
               else
               {
                  push_lab_media_fmt_t *med_ptr = &me_ptr->push_lab_info.media_fmt;
                  push_lab_media_fmt_t *cfg_ptr = &me_ptr->push_lab_info.cfg_media_fmt;

                  // copy incoming media fmt as cfg media fmt.
                  capi_err_t local_result = push_lab_set_inp_media_fmt(&(me_ptr->push_lab_info), media_fmt, cfg_ptr);
                  if (CAPI_EOK == local_result)
                  {
                     return local_result;
                  }

                  // if input media fmt has arrived, then check if it matches
                  // configured media fmt
                  if (0 != med_ptr->num_channels)
                  {
                     if ((med_ptr->num_channels != cfg_ptr->num_channels) ||
                         (med_ptr->sample_rate != cfg_ptr->sample_rate) ||
                         (med_ptr->bits_per_sample != cfg_ptr->bits_per_sample) ||
                         (med_ptr->Q_format != cfg_ptr->Q_format))
                     {
                        AR_MSG(DBG_ERROR_PRIO,
                               "capi_push_lab: Push mode: Media format from client must "
                               "match input media format.");
                        capi_result = CAPI_EBADPARAM;
                     }
                  }
               }
            }
         }

         break;
      }
      // TODO: Check if requireed
      case FWK_EXTN_PCM_PARAM_ID_MEDIA_FORMAT_EXTN:
      {
         if ((params_ptr->actual_data_len >= sizeof(fwk_extn_pcm_param_id_media_fmt_extn_t)) &&
             (TRUE == port_info_ptr->is_input_port))
         {
            fwk_extn_pcm_param_id_media_fmt_extn_t *extn_ptr =
               (fwk_extn_pcm_param_id_media_fmt_extn_t *)(params_ptr->data_ptr);

            AR_MSG(DBG_HIGH_PRIO, "capi_push_lab : FWK_EXTN_PCM_PARAM_ID_MEDIA_FORMAT_EXTN, entry");

            push_lab_set_fwk_ext_inp_media_fmt(&(me_ptr->push_lab_info), extn_ptr);

            push_lab_media_fmt_t *med_ptr = &me_ptr->push_lab_info.media_fmt;
            push_lab_media_fmt_t *cfg_ptr = &me_ptr->push_lab_info.cfg_media_fmt;

            // if cfg media format has come has arrived, then check if it matches
            // configured media fmt
            if (0 != cfg_ptr->num_channels)
            {
               if ((med_ptr->num_channels != cfg_ptr->num_channels) || (med_ptr->sample_rate != cfg_ptr->sample_rate) ||
                   (med_ptr->bits_per_sample != cfg_ptr->bits_per_sample) || (med_ptr->Q_format != cfg_ptr->Q_format))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi_push_lab: Push mode: Media format from client must match "
                         "input media format.");
               }
            }
         }
         else
         {
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            AR_MSG(DBG_ERROR_PRIO,
                   "capi_push_lab: set param failed because of length issues, 0x%p, "
                   "0x%p, in_len = %d, needed_len = "
                   "%d",
                   _pif,
                   param_id,
                   params_ptr->actual_data_len,
                   sizeof(fwk_extn_pcm_param_id_media_fmt_extn_t));
         }
         break;
      }

      case PARAM_ID_SH_MEM_PUSH_LAB_WATERMARK_PERIOD:
      {
         sh_mem_push_lab_watermark_period_t *payload_ptr = (sh_mem_push_lab_watermark_period_t *)params_ptr->data_ptr;
         me_ptr->push_lab_info.watermark_interval_in_us  = 1000 * (payload_ptr->watermark_period_in_ms);
         // if media fmt available, convert to bytes. Else, cache and convert on
         // getting media fmt
         // TODO: Check w.r.t. usecase if this conversion can always be handled
         // after media format comes to avoid code duplication
         if (me_ptr->push_lab_info.is_media_fmt_populated == 1)
         {
            me_ptr->push_lab_info.watermark_interval_in_bytes =
               sh_mem_push_lab_us_to_bytes(me_ptr, me_ptr->push_lab_info.watermark_interval_in_us);
         }
         else
         {
            me_ptr->push_lab_info.watermark_interval_in_bytes = 0;
         }
         AR_MSG(DBG_HIGH_PRIO,
                "capi_push_lab: Set param id 0x%lx, watermark_period_in_ms "
                "%d,watermark_interval_in_us %d, "
                "watermark_interval_in_bytes %d",
                param_id,
                payload_ptr->watermark_period_in_ms,
                me_ptr->push_lab_info.watermark_interval_in_us,
                me_ptr->push_lab_info.watermark_interval_in_bytes);
         break;
      }
      case INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION:
      {
         if (NULL == params_ptr->data_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Set param id 0x%lx, received null buffer", param_id);
            capi_result |= CAPI_EBADPARAM;
            break;
         }
         if (params_ptr->actual_data_len < sizeof(intf_extn_param_id_imcl_port_operation_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi_push_lab: Invalid payload size for ctrl port operation %d",
                   params_ptr->actual_data_len);
            capi_result |= CAPI_ENEEDMORE;
            break;
         }
         intf_extn_param_id_imcl_port_operation_t *port_op_ptr =
            (intf_extn_param_id_imcl_port_operation_t *)(params_ptr->data_ptr);
         switch (port_op_ptr->opcode)
         {
            case INTF_EXTN_IMCL_PORT_OPEN:
            {
               if (port_op_ptr->op_payload.data_ptr)
               {
                  intf_extn_imcl_port_open_t *port_open_ptr =
                     (intf_extn_imcl_port_open_t *)port_op_ptr->op_payload.data_ptr;
                  /*Size Validation*/
                  uint32_t num_ports = port_open_ptr->num_ports;

                  if (num_ports > PUSH_LAB_MAX_CTRL_PORT)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "capi_push_lab: Push Lab only supports a max of %lu "
                            "control ports. Trying to open %lu",
                            PUSH_LAB_MAX_CTRL_PORT,
                            num_ports);
                     return CAPI_EBADPARAM;
                  }

                  uint32_t valid_size =
                     sizeof(intf_extn_imcl_port_open_t) + (num_ports * sizeof(intf_extn_imcl_id_intent_map_t));
                  if (port_op_ptr->op_payload.actual_data_len < valid_size)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "capi_push_lab: Invalid payload size for ctrl port OPEN %d",
                            params_ptr->actual_data_len);
                     return CAPI_ENEEDMORE;
                  }

                  for (uint32_t iter = 0; iter < num_ports; iter++)
                  {
                     /*Size Validation*/
                     valid_size += (port_open_ptr->intent_map[iter].num_intents * sizeof(uint32_t));

                     // Push lab always expects just one intent per ctrl port
                     if ((port_open_ptr->intent_map[iter].num_intents != PUSH_LAB_MAX_INTENTS_PER_CTRL_PORT) ||
                         (port_op_ptr->op_payload.actual_data_len < valid_size))
                     {
                        AR_MSG(DBG_ERROR_PRIO,
                               "capi_push_lab: Note: Push lab always expects just one "
                               "intent per ctrl port;"
                               "Invalid payload size for ctrl port OPEN %d",
                               params_ptr->actual_data_len);
                        return CAPI_ENEEDMORE;
                     }

                     uint32_t ctrl_port_id = port_open_ptr->intent_map[iter].port_id;

                     // Get control port index corresponding to the control port ID.
                     uint32_t cp_arr_idx = capi_push_lab_get_ctrl_port_arr_idx_from_ctrl_port_id(me_ptr, ctrl_port_id);
                     if (IS_INVALID_PORT_INDEX(cp_arr_idx))
                     {
                        AR_MSG(DBG_ERROR_PRIO, "Capi_push_lab: Ctrl port 0x%lx mapping not found.", ctrl_port_id);
                        return CAPI_EBADPARAM;
                     }

                     // Cache the intents in the cp_arr_idx
                     me_ptr->imcl_port_info_arr[cp_arr_idx].port_id     = ctrl_port_id;
                     me_ptr->imcl_port_info_arr[cp_arr_idx].state       = CTRL_PORT_OPEN;
                     me_ptr->imcl_port_info_arr[cp_arr_idx].num_intents = port_open_ptr->intent_map[iter].num_intents;

                     for (uint32_t i = 0; i < port_open_ptr->intent_map[iter].num_intents; i++)
                     {
                        me_ptr->imcl_port_info_arr[cp_arr_idx].intent_list_arr[i] =
                           port_open_ptr->intent_map[iter].intent_arr[i];
                     }

                     /* Register with the fwk for IMCL Recurring buffers */ // TBD:size
                                                                            // and num
                                                                            // like
                                                                            // DAM
                     capi_result |=
                        capi_push_lab_imcl_register_for_recurring_bufs(me_ptr,
                                                                       ctrl_port_id,
                                                                       PUSH_LAB_MAX_RECURRING_BUF_SIZE,  /*buf_size*/
                                                                       PUSH_LAB_MAX_NUM_RECURRING_BUFS); /*num_bufs*/

                     AR_MSG(DBG_HIGH_PRIO, "capi_push_lab: ctrl port_id 0x%lx received open. ", ctrl_port_id);
                  }
               }
               else
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi_push_lab: Ctrl port open expects a payload. "
                         "Failing.");
                  return CAPI_EFAILED;
               }
               break;
            }
            case INTF_EXTN_IMCL_PORT_CLOSE:
            {
               if (port_op_ptr->op_payload.data_ptr)
               {
                  intf_extn_imcl_port_close_t *port_close_ptr =
                     (intf_extn_imcl_port_close_t *)port_op_ptr->op_payload.data_ptr;
                  /*Size Validation*/
                  uint32_t num_ports = port_close_ptr->num_ports;

                  if (num_ports > PUSH_LAB_MAX_CTRL_PORT)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "capi_push_lab: Push Lab module only supports a max of "
                            "%lu control ports. Trying to close "
                            "%lu",
                            PUSH_LAB_MAX_CTRL_PORT,
                            num_ports);
                     return CAPI_EBADPARAM;
                  }

                  uint32_t valid_size = sizeof(intf_extn_imcl_port_close_t) + (num_ports * sizeof(uint32_t));
                  if (port_op_ptr->op_payload.actual_data_len < valid_size)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "capi_push_lab: Invalid payload size for ctrl port CLOSE %d",
                            params_ptr->actual_data_len);
                     return CAPI_ENEEDMORE;
                  }

                  // for each port id in the list that follows...
                  for (uint32_t iter = 0; iter < num_ports; iter++)
                  {
                     // Get the arr index for the control port ID.
                     uint32_t cp_arr_idx =
                        capi_push_lab_get_ctrl_port_arr_idx_from_ctrl_port_id(me_ptr,
                                                                              port_close_ptr->port_id_arr[iter]);
                     if (IS_INVALID_PORT_INDEX(cp_arr_idx))
                     {
                        AR_MSG(DBG_ERROR_PRIO,
                               "capi_push_lab: Ctrl port 0x%lx mapping not found.",
                               port_close_ptr->port_id_arr[iter]);
                        continue;
                        CAPI_EBADPARAM;
                     }

                     me_ptr->imcl_port_info_arr[cp_arr_idx].state       = CTRL_PORT_CLOSE;
                     me_ptr->imcl_port_info_arr[cp_arr_idx].port_id     = 0;
                     me_ptr->imcl_port_info_arr[cp_arr_idx].num_intents = 0;
                     // When port close comes, is_gate_opened should be set to FALSE to
                     // stop data flow
                     me_ptr->push_lab_info.is_gate_opened = FALSE;
					 me_ptr->push_lab_info.acc_data = 0;
                  }
               }
               else
               {
                  AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Ctrl port close expects a payload. Failing.");
                  return CAPI_EFAILED;
               }
               break;
            }
            case INTF_EXTN_IMCL_PORT_PEER_CONNECTED:
            {
               if (port_op_ptr->op_payload.data_ptr)
               {
                  intf_extn_imcl_port_start_t *port_start_ptr =
                     (intf_extn_imcl_port_start_t *)port_op_ptr->op_payload.data_ptr;
                  /*Size Validation*/
                  uint32_t num_ports = port_start_ptr->num_ports;

                  if (num_ports > PUSH_LAB_MAX_CTRL_PORT) // me_ptr->max_output_ports)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "capi_push_lab: Push lab module only supports a max of "
                            "%lu control ports. Trying to start "
                            "%lu",
                            PUSH_LAB_MAX_CTRL_PORT,
                            num_ports);
                     return CAPI_EBADPARAM;
                  }

                  uint32_t valid_size = sizeof(intf_extn_imcl_port_start_t) + (num_ports * sizeof(uint32_t));
                  if (port_op_ptr->op_payload.actual_data_len < valid_size)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "capi_push_lab: Invalid payload size for ctrl port Start %d",
                            params_ptr->actual_data_len);
                     return CAPI_ENEEDMORE;
                  }

                  // for each port id in the list that follows...
                  for (uint32_t iter = 0; iter < num_ports; iter++)
                  {
                     // Get the arr index for the control port ID.
                     uint32_t cp_arr_idx =
                        capi_push_lab_get_ctrl_port_arr_idx_from_ctrl_port_id(me_ptr,
                                                                              port_start_ptr->port_id_arr[iter]);
                     if (IS_INVALID_PORT_INDEX(cp_arr_idx))
                     {
                        AR_MSG(DBG_ERROR_PRIO,
                               "capi_push_lab: Ctrl port 0x%lx mapping not found.",
                               port_start_ptr->port_id_arr[iter]);
                        continue;
                        capi_result = CAPI_EBADPARAM;
                     }

                     me_ptr->imcl_port_info_arr[cp_arr_idx].state = CTRL_PORT_PEER_CONNECTED;

                     AR_MSG(DBG_HIGH_PRIO,
                            "capi_push_lab: ctrl port_id 0x%lx received start. ",
                            port_start_ptr->port_id_arr[iter]);
                  }
               }
               else
               {
                  AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Ctrl port start expects a payload. Failing.");
                  return CAPI_EFAILED;
               }
               break;
            }
            case INTF_EXTN_IMCL_PORT_PEER_DISCONNECTED:
            {
               if (port_op_ptr->op_payload.data_ptr)
               {
                  intf_extn_imcl_port_stop_t *port_stop_ptr =
                     (intf_extn_imcl_port_stop_t *)port_op_ptr->op_payload.data_ptr;
                  /*Size Validation*/
                  uint32_t num_ports = port_stop_ptr->num_ports;

                  if (num_ports > PUSH_LAB_MAX_CTRL_PORT)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "capi_push_lab: Push Lab module only supports a max of "
                            "%lu control ports. Trying to STOP "
                            "%lu",
                            PUSH_LAB_MAX_CTRL_PORT,
                            num_ports);
                     return CAPI_EBADPARAM;
                  }

                  uint32_t valid_size = sizeof(intf_extn_imcl_port_stop_t) + (num_ports * sizeof(uint32_t));
                  if (port_op_ptr->op_payload.actual_data_len < valid_size)
                  {
                     AR_MSG(DBG_ERROR_PRIO,
                            "capi_push_lab: Invalid payload size for ctrl port STOP %d",
                            params_ptr->actual_data_len);
                     return CAPI_ENEEDMORE;
                  }

                  // for each port id in the list that follows...
                  for (uint32_t iter = 0; iter < num_ports; iter++)
                  {
                     // Get the arr index for the control port ID.
                     uint32_t cp_arr_idx =
                        capi_push_lab_get_ctrl_port_arr_idx_from_ctrl_port_id(me_ptr, port_stop_ptr->port_id_arr[iter]);
                     if (IS_INVALID_PORT_INDEX(cp_arr_idx))
                     {
                        AR_MSG(DBG_ERROR_PRIO,
                               "capi_push_lab: Ctrl port 0x%lx mapping not found.",
                               port_stop_ptr->port_id_arr[iter]);
                        continue;
                        CAPI_EBADPARAM;
                     }

                     AR_MSG(DBG_HIGH_PRIO,
                            "capi_push_lab: ctrl port_id 0x%lx received STOP. ",
                            port_stop_ptr->port_id_arr[iter]);

                     me_ptr->imcl_port_info_arr[cp_arr_idx].state = CTRL_PORT_PEER_DISCONNECTED;
                     me_ptr->push_lab_info.is_gate_opened         = FALSE;
					 me_ptr->push_lab_info.acc_data = 0;
                  }
               }
               else
               {
                  AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Ctrl port STOP expects a payload. Failing.");
                  return CAPI_EFAILED;
               }
               break;
            }
            default:
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Received unsupported ctrl port opcode %lu", port_op_ptr->opcode);
               return CAPI_EUNSUPPORTED;
            }
         }

         break;
      }

      case INTF_EXTN_PARAM_ID_IMCL_INCOMING_DATA:
      {
         /*TBD: Currently port ID is in the port info. Can be changed later */
         if (NULL == params_ptr->data_ptr)
         {
            AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Set param id 0x%lx, received null buffer", param_id);
            capi_result |= CAPI_EBADPARAM;
            break;
         }

         // Level 1 check
         if (params_ptr->actual_data_len < MIN_INCOMING_IMCL_PARAM_SIZE_SVA_DAM)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "capi_push_lab: Invalid payload size for incoming data %d",
                   params_ptr->actual_data_len);
            return CAPI_ENEEDMORE;
         }

         intf_extn_param_id_imcl_incoming_data_t *payload_ptr =
            (intf_extn_param_id_imcl_incoming_data_t *)params_ptr->data_ptr;

         uint32_t ctrl_port_id = payload_ptr->port_id;

         capi_result = capi_sh_mem_push_lab_imc_set_param_handler(me_ptr, ctrl_port_id, params_ptr);
         if (CAPI_EOK != capi_result)
         {
            AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: IMC set param handler failed 0x%x \n", param_id);
         }
         break;
      }
      default:
      {
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);

         break;
      }
   }
   return capi_result;
}

static capi_err_t capi_push_lab_get_param(capi_t *                _pif,
                                          uint32_t                param_id,
                                          const capi_port_info_t *port_info_ptr,
                                          capi_buf_t *            params_ptr)
{
   if (NULL == _pif || NULL == params_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi_push_lab: Get param received bad pointer, 0x%lx, 0x%lx",
             (uint32_t)_pif,
             (uint32_t)params_ptr);

      return CAPI_EOK;
   }

   capi_err_t       capi_result = CAPI_EOK;
   capi_push_lab_t *me_ptr      = (capi_push_lab_t *)_pif;

   void *param_payload_ptr = (void *)params_ptr->data_ptr;

   if (NULL == param_payload_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Set param received NULL payload pointer");

      return CAPI_EBADPARAM;
   }

   switch (param_id)
   {
      case FWK_EXTN_PCM_PARAM_ID_MEDIA_FORMAT_EXTN:
      {
         if ((params_ptr->actual_data_len >= sizeof(fwk_extn_pcm_param_id_media_fmt_extn_t)))
         {
            fwk_extn_pcm_param_id_media_fmt_extn_t *extn_ptr =
               (fwk_extn_pcm_param_id_media_fmt_extn_t *)(params_ptr->data_ptr);

            AR_MSG(DBG_HIGH_PRIO,
                   "capi_push_lab : GET Param "
                   "FWK_EXTN_PCM_PARAM_ID_MEDIA_FORMAT_EXTN, "
                   "entry");
            extn_ptr->alignment  = me_ptr->push_lab_info.media_fmt.alignment;
            extn_ptr->bit_width  = me_ptr->push_lab_info.media_fmt.bit_width;
            extn_ptr->endianness = me_ptr->push_lab_info.media_fmt.endianness;
            AR_MSG(DBG_HIGH_PRIO,
                   "capi_push_lab: GET Param FWK_EXTN_PCM_PARAM_ID_MEDIA_FORMAT_EXTN, "
                   "exit");
         }
         else
         {
            CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            AR_MSG(DBG_ERROR_PRIO,
                   "capi_push_lab: get param failed because of length issues, 0x%p, "
                   "0x%p, in_len = %d, needed_len = "
                   "%d",
                   _pif,
                   param_id,
                   params_ptr->actual_data_len,
                   sizeof(fwk_extn_pcm_param_id_media_fmt_extn_t));
         }
         break;
      }

      default:
      {
         CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);

         break;
      }
   }
   return capi_result;
}

static capi_err_t capi_push_lab_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == props_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Set property received null property array");
      return CAPI_EBADPARAM;
   }

   capi_push_lab_t *me_ptr = (capi_push_lab_t *)_pif;
   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Set property received null property array");
      return CAPI_EBADPARAM;
   }

   capi_result = capi_cmn_set_basic_properties(props_ptr, &me_ptr->heap_mem, &me_ptr->cb_info, NULL);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Set basic properties failed with result %lu", capi_result);
      return capi_result;
   }

   capi_prop_t *prop_ptr = props_ptr->prop_ptr;

   for (uint32_t i = 0; i < props_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;

      switch (prop_ptr[i].id)
      {
         case CAPI_PORT_NUM_INFO:
         {
            if (NULL == payload_ptr->data_ptr)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Set property id 0x%lx, received null buffer", prop_ptr[i].id);
               capi_result |= CAPI_EBADPARAM;
               break;
            }
            if (payload_ptr->max_data_len >= sizeof(capi_port_num_info_t))
            {
               capi_port_num_info_t *data_ptr = (capi_port_num_info_t *)payload_ptr->data_ptr;
               if ((CAPI_PUSH_LAB_MAX_PORTS != data_ptr->num_input_ports) &&
                   (CAPI_PUSH_LAB_MAX_PORTS < data_ptr->num_output_ports))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi_push_lab: Error: Num of ports supported = %lu, trying "
                         "to set %lu input ports "
                         "and "
                         "%lu "
                         "output ports",
                         CAPI_PUSH_LAB_MAX_PORTS,
                         data_ptr->num_input_ports,
                         data_ptr->num_output_ports);
                  return CAPI_EBADPARAM;
               }
               me_ptr->port_info.num_input_ports  = data_ptr->num_input_ports;
               me_ptr->port_info.num_output_ports = data_ptr->num_output_ports;
               me_ptr->max_input_ports            = data_ptr->num_input_ports;
               me_ptr->max_output_ports           = data_ptr->num_output_ports;
               payload_ptr->actual_data_len       = sizeof(capi_port_num_info_t);

               // Create port related structures based on the port info.
               capi_result |= capi_create_port_structures(me_ptr);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_push_lab :FAILED Set Property id 0x%lx Bad param size %lu",
                      (uint32_t)prop_ptr[i].id,
                      payload_ptr->max_data_len);
               return CAPI_ENEEDMORE;
            }
            break;
         }

         case CAPI_INPUT_MEDIA_FORMAT_V2:
         {
            AR_MSG(DBG_HIGH_PRIO, "capi_push_lab: Set property received for input media fmt");
            me_ptr->push_lab_info.is_disabled            = FALSE;
            me_ptr->push_lab_info.is_media_fmt_populated = 0;

            /* If the query happens for module output port */
            if (!prop_ptr[i].port_info.is_input_port)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: incorrect port info, output port[%d]", prop_ptr[i].id);

               capi_result |= CAPI_EBADPARAM;
               break;
            }

            /* Validate the MF payload */
            if (payload_ptr->max_data_len < sizeof(capi_push_lab_media_fmt_t))
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Not valid media format size %d", payload_ptr->actual_data_len);

               capi_result |= CAPI_ENEEDMORE;
               break;
            }

            /* Return error if circular buffer is not configured */
            if (0 == me_ptr->push_lab_info.shared_circ_buf_size)
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: circular buffer is not configured");
               me_ptr->push_lab_info.is_disabled = TRUE;
               capi_result                       = CAPI_EFAILED;
            }

            capi_media_fmt_v2_t *media_fmt_ptr = (capi_media_fmt_v2_t *)(payload_ptr->data_ptr);

            typedef struct pm_media_format_t
            {
               media_format_t          fmt;
               payload_media_fmt_pcm_t pcm;
               uint8_t                 channel_mapping[PUSH_LAB_MAX_CHANNELS];
            } pm_media_format_t;
            pm_media_format_t media_fmt;

            media_fmt.fmt.data_format     = 0;
            media_fmt.fmt.payload_size    = sizeof(payload_media_fmt_pcm_t);
            media_fmt.fmt.fmt_id          = me_ptr->push_lab_info.media_fmt.fmt_id;
            media_fmt.pcm.num_channels    = media_fmt_ptr->format.num_channels;
            media_fmt.pcm.bit_width       = QFORMAT_TO_BIT_WIDTH(media_fmt_ptr->format.q_factor);
            media_fmt.pcm.q_factor        = media_fmt_ptr->format.q_factor;
            media_fmt.pcm.sample_rate     = media_fmt_ptr->format.sampling_rate;
            media_fmt.pcm.bits_per_sample = media_fmt_ptr->format.bits_per_sample;
            media_fmt.pcm.endianness      = me_ptr->push_lab_info.media_fmt.endianness;
            media_fmt.pcm.alignment       = me_ptr->push_lab_info.media_fmt.alignment;

            for (int i = 0; i < media_fmt_ptr->format.num_channels; i++)
            {
               media_fmt.channel_mapping[i]                   = media_fmt_ptr->channel_type[i];
               me_ptr->push_lab_info.media_fmt.channel_map[i] = media_fmt_ptr->channel_type[i];
            }

            // if media fmt param has already arrived before, then check if the
            // framework media fmt matches the already configured media fmt
            if (0 != me_ptr->push_lab_info.media_fmt.num_channels)
            {
               if ((me_ptr->push_lab_info.media_fmt.num_channels != media_fmt_ptr->format.num_channels) ||
                   (me_ptr->push_lab_info.media_fmt.sample_rate != media_fmt_ptr->format.sampling_rate) ||
                   (me_ptr->push_lab_info.media_fmt.bits_per_sample != media_fmt_ptr->format.bits_per_sample) ||
                   (me_ptr->push_lab_info.media_fmt.Q_format != media_fmt_ptr->format.q_factor))
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "capi_push_lab: Push lab: Media format from framework must "
                         "match media format configured by client.");
                  capi_result = CAPI_EBADPARAM;
               }
            }

            capi_result =
               push_lab_set_inp_media_fmt(&(me_ptr->push_lab_info), &media_fmt.fmt, &me_ptr->push_lab_info.media_fmt);

            me_ptr->push_lab_info.media_fmt.data_interleaving = media_fmt_ptr->format.data_interleaving;

            if ((CAPI_EOK == capi_result) && (TRUE == push_lab_check_media_fmt_validity(&(me_ptr->push_lab_info))))
            {
               capi_result |= capi_cmn_update_kpps_event(&me_ptr->cb_info, CAPI_PUSH_LAB_KPPS);
               capi_result |= capi_cmn_update_bandwidth_event(&me_ptr->cb_info, 0, 0);
               capi_result |= capi_cmn_update_process_check_event(&me_ptr->cb_info, 1);
               capi_result |= capi_cmn_update_algo_delay_event(&me_ptr->cb_info, 0);
               me_ptr->push_lab_info.is_media_fmt_populated = 1; // updating the flag on successful media fmt set

               // media format received from the input is to be used for updating the output channel mapping
               push_lab_update_dam_output_ch_cfg(me_ptr);


               if ((me_ptr->push_lab_info.watermark_interval_in_us != 0) &&
                   (me_ptr->push_lab_info.watermark_interval_in_bytes == 0))
               {
                  me_ptr->push_lab_info.watermark_interval_in_bytes =
                     sh_mem_push_lab_us_to_bytes(me_ptr, me_ptr->push_lab_info.watermark_interval_in_us);
                  AR_MSG(DBG_HIGH_PRIO,
                         "capi_push_lab: watermark_interval_in_us %d, watermark_interval_in_bytes %d",
                         me_ptr->push_lab_info.watermark_interval_in_us,
                         me_ptr->push_lab_info.watermark_interval_in_bytes);
               }
               capi_push_lab_raise_output_media_fmt_event(me_ptr);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_push_lab: Set prop failed for prop id 0x%lx, result %d",
                      prop_ptr[i].id,
                      capi_result);
            }

            break;
         }
         case CAPI_ALGORITHMIC_RESET:
         {
            AR_MSG(DBG_HIGH_PRIO, "capi_push_lab: Set property received for algorithmic reset");
            me_ptr->push_lab_info.is_gate_opened      = FALSE;
			me_ptr->push_lab_info.acc_data = 0;
            me_ptr->push_lab_info.pos_buf_write_index = 0;
            break;
         }
         case CAPI_REGISTER_EVENT_DATA_TO_DSP_CLIENT_V2:
         {
            AR_MSG(DBG_HIGH_PRIO,
                   "capi_push_lab: Set property received for registering "
                   "event data to dsp client v2");

            /* Validate the payload */
            if (payload_ptr->actual_data_len < sizeof(capi_register_event_to_dsp_client_v2_t))
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Invalid payload size %d", payload_ptr->actual_data_len);

               capi_result |= CAPI_ENEEDMORE;
               break;
            }

            capi_register_event_to_dsp_client_v2_t *reg_event_ptr =
               (capi_register_event_to_dsp_client_v2_t *)(payload_ptr->data_ptr);
            bool_t is_found = false, is_already_registered = false;

            if (EVENT_ID_SH_MEM_PUSH_LAB_WATERMARK == reg_event_ptr->event_id)
            {
               AR_MSG(DBG_HIGH_PRIO,
                      "capi_push_lab: Received for registering/deregistering "
                      "event data for watermark event, "
                      "is_register: %d",
                      reg_event_ptr->is_register);
               if (1 == reg_event_ptr->is_register)
               {
                  if (0 == me_ptr->push_lab_info.watermark_event_client_info.watermark_event_dest_address &&
                      0 == me_ptr->push_lab_info.watermark_event_client_info.watermark_event_token && !is_found)
                  {
                     is_found = TRUE;
                  }
                  /* checking if the client is already registered*/
                  else if (reg_event_ptr->dest_address ==
                              me_ptr->push_lab_info.watermark_event_client_info.watermark_event_dest_address &&
                           reg_event_ptr->token ==
                              me_ptr->push_lab_info.watermark_event_client_info.watermark_event_token)
                  {
                     is_already_registered = TRUE;
                     break;
                  }

                  if (is_already_registered)
                  {
                     capi_result |= CAPI_EFAILED;
                     break;
                  }
                  if (is_found)
                  {
                     me_ptr->push_lab_info.watermark_event_client_info.watermark_event_dest_address =
                        reg_event_ptr->dest_address;
                     me_ptr->push_lab_info.watermark_event_client_info.watermark_event_token = reg_event_ptr->token;
                     capi_result                                                             = CAPI_EOK;
                  }
               }
               else if (0 == reg_event_ptr->is_register)
               {
                  if (reg_event_ptr->dest_address ==
                         me_ptr->push_lab_info.watermark_event_client_info.watermark_event_dest_address &&
                      reg_event_ptr->token == me_ptr->push_lab_info.watermark_event_client_info.watermark_event_token)
                  {
                     /* reset destination address and token, decrement num of clients
                      * registered*/
                     me_ptr->push_lab_info.watermark_event_client_info.watermark_event_dest_address = 0;
                     me_ptr->push_lab_info.watermark_event_client_info.watermark_event_token        = 0;
                     is_found                                                                       = TRUE;
                     break;
                  }
                  if (!is_found)
                  {
                     capi_result |= CAPI_EFAILED;
                     AR_MSG(DBG_ERROR_PRIO,
                            "capi_push_lab: client requested for "
                            "de-registration not found");
                  }
               }
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Unsupported event ID[%d]", reg_event_ptr->event_id);

               capi_result |= CAPI_EUNSUPPORTED;
            }
            break;
         }
         default:
         {
            AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Unknown Prop[0x%lX]", prop_ptr[i].id);

            capi_result |= CAPI_EUNSUPPORTED;
            break;
         }
      } /* Outer switch - Generic CAPI Properties */

   } /* Loop all properties */
   return capi_result;
}

static capi_err_t capi_push_lab_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t capi_result = CAPI_EOK;

   if (NULL == props_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Get property received null property array");
      return CAPI_EBADPARAM;
   }

   capi_push_lab_t *me_ptr = (capi_push_lab_t *)_pif;
   if (NULL == me_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Get property received null property array");
      return CAPI_EBADPARAM;
   }

   capi_result = capi_push_lab_process_get_properties(me_ptr, props_ptr);

   return capi_result;
}

static capi_err_t capi_push_lab_process_get_properties(capi_push_lab_t *me_ptr, capi_proplist_t *proplist_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL == proplist_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_push_lab: Get property received null property array");
      return CAPI_EBADPARAM;
   }

   uint32_t          fwk_extn_ids_arr[1] = { FWK_EXTN_PCM }; // TODO: chekc if it is used
   capi_basic_prop_t mod_prop;
   mod_prop.init_memory_req    = align_to_8_byte(sizeof(capi_push_lab_t));
   mod_prop.stack_size         = CAPI_PUSH_LAB_STACK_SIZE;
   mod_prop.num_fwk_extns      = PM_NUM_FRAMEWORK_EXTENSIONS;
   mod_prop.fwk_extn_ids_arr   = fwk_extn_ids_arr;
   mod_prop.is_inplace         = FALSE;
   mod_prop.req_data_buffering = FALSE;
   mod_prop.max_metadata_size  = 0;

   capi_result = capi_cmn_get_basic_properties(proplist_ptr, &mod_prop);
   if (CAPI_EOK != capi_result)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "capi_push_lab: Get common basic properties failed with capi_result "
             "%lu",
             capi_result);
      return capi_result;
   }

   capi_prop_t *prop_ptr = proplist_ptr->prop_ptr;

   for (uint32_t i = 0; i < proplist_ptr->props_num; i++)
   {
      switch (prop_ptr[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_STACK_SIZE:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         case CAPI_MAX_METADATA_SIZE:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_NEEDED_FRAMEWORK_EXTENSIONS:
         {
            break;
         }
		 case CAPI_MIN_PORT_NUM_INFO:
         {
            capi_buf_t *payload_ptr = &prop_ptr[i].payload;
            if (payload_ptr->max_data_len >= sizeof(capi_min_port_num_info_t))
            {
               capi_min_port_num_info_t *data_ptr = (capi_min_port_num_info_t *)payload_ptr->data_ptr;
               data_ptr->num_min_input_ports      = 1; // always needs input
               data_ptr->num_min_output_ports     = 0; // can act as sink
               payload_ptr->actual_data_len       = sizeof(capi_min_port_num_info_t);
            }
            else
            {
               capi_result |= CAPI_ENEEDMORE;
            }
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_V2:
         {
            if (NULL == me_ptr || ((prop_ptr[i].port_info.is_valid) && (0 != prop_ptr[i].port_info.port_index)))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "capi_push_lab: Get property id 0x%lx failed due to "
                      "invalid/unexpected values",
                      (uint32_t)prop_ptr[i].id);
               CAPI_SET_ERROR(capi_result, CAPI_EFAILED);
               break;
            }

            capi_media_fmt_v2_t media_fmt;
            media_fmt.header.format_header.data_format = CAPI_FIXED_POINT;
            media_fmt.format.bitstream_format          = me_ptr->push_lab_info.media_fmt.fmt_id;
            media_fmt.format.bits_per_sample           = me_ptr->push_lab_info.media_fmt.bits_per_sample;
            media_fmt.format.num_channels              = me_ptr->push_lab_info.media_fmt.num_channels;
            media_fmt.format.q_factor                  = me_ptr->push_lab_info.media_fmt.Q_format;
            media_fmt.format.data_is_signed            = me_ptr->push_lab_info.media_fmt.is_signed;
            media_fmt.format.sampling_rate             = me_ptr->push_lab_info.media_fmt.sample_rate;
            media_fmt.format.data_interleaving         = CAPI_DEINTERLEAVED_UNPACKED;

            for (int i = 0; i < media_fmt.format.num_channels; i++)
            {
               media_fmt.channel_type[i] = me_ptr->push_lab_info.media_fmt.channel_map[i];
            }

            capi_result |= capi_cmn_handle_get_output_media_fmt_v2(&prop_ptr[i], &media_fmt);
            break;
         }
         case CAPI_PORT_DATA_THRESHOLD:
         {
            uint32_t threshold = 1;

            capi_result |= capi_cmn_handle_get_port_threshold(&prop_ptr[i], threshold);
            break;
         }

         default:
         {
            AR_MSG(DBG_HIGH_PRIO, "capi_push_lab: Skipped Get Property for 0x%x. Not supported.", prop_ptr[i].id);
            CAPI_SET_ERROR(capi_result, CAPI_EUNSUPPORTED);
            continue;
         }
      }
   }
   return capi_result;
}
