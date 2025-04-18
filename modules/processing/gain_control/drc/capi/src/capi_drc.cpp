/**
@file capi_drc.cpp
@brief CAPI V2 API wrapper for DRC algorithm

 */

/*------------------------------------------------------------------------------
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause
-------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 * Include Files
 * ----------------------------------------------------------------------------*/
#include "capi_drc.h"

#include "capi_drc_utils.h"

/*------------------------------------------------------------------------
 * Static declarations
 * -----------------------------------------------------------------------*/
static capi_err_t capi_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[]);
static capi_err_t capi_end(capi_t *_pif);
static capi_err_t capi_set_param(capi_t *                _pif,
                                 uint32_t                param_id,
                                 const capi_port_info_t *port_info_ptr,
                                 capi_buf_t *            params_ptr);
static capi_err_t capi_get_param(capi_t *                _pif,
                                 uint32_t                param_id,
                                 const capi_port_info_t *port_info_ptr,
                                 capi_buf_t *            params_ptr);
static capi_err_t capi_set_properties(capi_t *_pif, capi_proplist_t *props_ptr);
static capi_err_t capi_get_properties(capi_t *_pif, capi_proplist_t *props_ptr);

static capi_vtbl_t vtbl = { capi_process,        capi_end,           capi_set_param, capi_get_param,
                            capi_set_properties, capi_get_properties };

/*----------------------------------------------------------------------------
 * Function Definitions
 * -------------------------------------------------------------------------*/
capi_err_t capi_drc_get_static_properties(capi_proplist_t *init_props_ptr, capi_proplist_t *out_props_ptr)
{
   return capi_get_properties(NULL, out_props_ptr);
}

capi_err_t capi_drc_init(capi_t *_pif, capi_proplist_t *init_props_ptr)
{
   capi_err_t  result = CAPI_EOK;
   capi_drc_t *me_ptr = (capi_drc_t *)_pif;

   result |= (NULL == _pif) ? CAPI_EFAILED : result;
   CHECK_THROW_ERROR(MIID_UNKNOWN, result, "Init received null pointer.");

   memset(me_ptr, 0, sizeof(capi_drc_t));

   drc_lib_set_default_config(me_ptr);

   if (init_props_ptr)
   {
      result = capi_set_properties(_pif, init_props_ptr);

      // Ignoring non-fatal error code.
      result ^= (result & CAPI_EUNSUPPORTED);
      CHECK_THROW_ERROR(MIID_UNKNOWN, result, "Init set properties failed.");
   }

   me_ptr->vtbl = &vtbl;

   capi_cmn_update_process_check_event(&me_ptr->cb_info, FALSE);

   capi_cmn_ctrl_port_list_init(&me_ptr->ctrl_port_info);

   DRC_MSG(me_ptr->miid, DBG_LOW_PRIO, "Init done, status 0x%x.", result);

   return result;
}

static capi_err_t capi_set_param(capi_t *                _pif,
                                 uint32_t                param_id,
                                 const capi_port_info_t *port_info_ptr,
                                 capi_buf_t *            params_ptr)
{
   capi_err_t  result = CAPI_EOK;
   capi_drc_t *me_ptr = (capi_drc_t *)_pif;

   result |= (NULL == _pif || NULL == params_ptr || NULL == params_ptr->data_ptr) ? CAPI_EFAILED : result;
   CHECK_THROW_ERROR(MIID_UNKNOWN, result, "setparam received bad pointer.");

   switch (param_id)
   {
      case INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION: {
         uint32_t supported_intent[1] = { INTENT_ID_DRC_CONFIG };
         result                       = capi_cmn_ctrl_port_operation_handler(&me_ptr->ctrl_port_info,
                                                       params_ptr,
                                                       (POSAL_HEAP_ID)me_ptr->heap_id,
                                                       0,
                                                       1,
                                                       supported_intent);
         break;
      }
      case PARAM_ID_MODULE_ENABLE: {
         result |= (params_ptr->actual_data_len < sizeof(param_id_module_enable_t)) ? CAPI_ENEEDMORE : result;
         CHECK_THROW_ERROR(me_ptr->miid,
                           result,
                           "Insufficient size for enable/disable parameter. Received %lu bytes",
                           params_ptr->actual_data_len);
         param_id_module_enable_t *en_dis_ptr = (param_id_module_enable_t *)params_ptr->data_ptr;

         if (me_ptr->b_enable != en_dis_ptr->enable)
         {
            me_ptr->b_enable = en_dis_ptr->enable;

            if (me_ptr->b_enable)
            {
               if (NULL != me_ptr->lib_handle.lib_mem_ptr)
               {
                  drc_set_param(&me_ptr->lib_handle, DRC_PARAM_SET_RESET, NULL, 0);
               }
            }

            raise_kpps_delay_process_events(me_ptr);
         }
         break;
      }
      case PARAM_ID_DRC_CONFIG: {
         result |= (params_ptr->actual_data_len < sizeof(param_id_drc_config_t)) ? CAPI_ENEEDMORE : result;
         CHECK_THROW_ERROR(me_ptr->miid,
                           result,
                           "Insufficient size for drc config parameter. Received %lu bytes",
                           params_ptr->actual_data_len);
         param_id_drc_config_t *drc_config_ptr = (param_id_drc_config_t *)params_ptr->data_ptr;

         // copy drc configuration data to wrapper structure
         me_ptr->lib_cfg.channelLinked           = drc_config_ptr->channel_linked_mode;
         me_ptr->lib_cfg.downSampleLevel         = drc_config_ptr->down_sample_level;
         me_ptr->lib_cfg.dnCompAttackUL32Q31     = drc_config_ptr->down_cmpsr_attack;
         me_ptr->lib_cfg.dnCompHysterisisUL16Q14 = drc_config_ptr->down_cmpsr_hysteresis;
         me_ptr->lib_cfg.dnCompReleaseUL32Q31    = drc_config_ptr->down_cmpsr_release;
         me_ptr->lib_cfg.dnCompSlopeUL16Q16      = drc_config_ptr->down_cmpsr_slope;
         me_ptr->lib_cfg.dnCompThresholdL16Q7    = drc_config_ptr->down_cmpsr_threshold;
         me_ptr->lib_cfg.dnExpaAttackUL32Q31     = drc_config_ptr->down_expdr_attack;
         me_ptr->lib_cfg.dnExpaHysterisisUL16Q14 = drc_config_ptr->down_expdr_hysteresis;
         me_ptr->lib_cfg.dnExpaMinGainDBL32Q23   = drc_config_ptr->down_expdr_min_gain_db;
         me_ptr->lib_cfg.dnExpaReleaseUL32Q31    = drc_config_ptr->down_expdr_release;
         me_ptr->lib_cfg.dnExpaSlopeL16Q8        = drc_config_ptr->down_expdr_slope;
         me_ptr->lib_cfg.dnExpaThresholdL16Q7    = drc_config_ptr->down_expdr_threshold;
         me_ptr->lib_cfg.makeupGainUL16Q12       = drc_config_ptr->makeup_gain;
         me_ptr->lib_cfg.rmsTavUL16Q16           = drc_config_ptr->rms_time_avg_const;
         me_ptr->lib_cfg.upCompAttackUL32Q31     = drc_config_ptr->up_cmpsr_attack;
         me_ptr->lib_cfg.upCompHysterisisUL16Q14 = drc_config_ptr->up_cmpsr_hysteresis;
         me_ptr->lib_cfg.upCompReleaseUL32Q31    = drc_config_ptr->up_cmpsr_release;
         me_ptr->lib_cfg.upCompSlopeUL16Q16      = drc_config_ptr->up_cmpsr_slope;
         me_ptr->lib_cfg.upCompThresholdL16Q7    = drc_config_ptr->up_cmpsr_threshold;

         me_ptr->mode = (drc_config_ptr->mode) ? DRC_ENABLED : DRC_BYPASSED;

         // check if delay is changed
         if (me_ptr->delay_us != drc_config_ptr->delay_us)
         {
            me_ptr->delay_us = drc_config_ptr->delay_us;

            result = drc_lib_alloc_init(me_ptr);
            CHECK_THROW_ERROR(me_ptr->miid, result, "Lib allocation failed.");
         }

         if (me_ptr->lib_handle.lib_mem_ptr)
         {
            result = drc_lib_set_calib(me_ptr);
            CHECK_THROW_ERROR(me_ptr->miid, result, "Lib setparam failed.");

            raise_kpps_delay_process_events(me_ptr);
         }

         break;
      }
      default: {
         DRC_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Invalid setparam received 0x%lx", param_id);
         return CAPI_EUNSUPPORTED;
      }
   }

   drc_lib_send_config_imcl(me_ptr);

   return result;
}

static capi_err_t capi_get_param(capi_t *                _pif,
                                 uint32_t                param_id,
                                 const capi_port_info_t *port_info_ptr,
                                 capi_buf_t *            params_ptr)
{
   capi_err_t  result = CAPI_EOK;
   capi_drc_t *me_ptr = (capi_drc_t *)_pif;

   result |= (NULL == _pif || NULL == params_ptr || NULL == params_ptr->data_ptr) ? CAPI_EFAILED : result;
   CHECK_THROW_ERROR(MIID_UNKNOWN, result, "getparam received bad pointer.");

   params_ptr->actual_data_len = 0;
   switch (param_id)
   {
      case PARAM_ID_MODULE_ENABLE: {
         result |= (params_ptr->max_data_len < sizeof(param_id_module_enable_t)) ? CAPI_ENEEDMORE : result;
         CHECK_THROW_ERROR(me_ptr->miid,
                           result,
                           "Insufficient size for enable/disable parameter. Received %lu bytes",
                           params_ptr->max_data_len);
         param_id_module_enable_t *en_dis_ptr = (param_id_module_enable_t *)params_ptr->data_ptr;
         memset(en_dis_ptr, 0, sizeof(param_id_module_enable_t));
         en_dis_ptr->enable = me_ptr->b_enable;

         params_ptr->actual_data_len = sizeof(param_id_module_enable_t);

         break;
      }
      case PARAM_ID_DRC_CONFIG: {
         result |= (params_ptr->max_data_len < sizeof(param_id_drc_config_t)) ? CAPI_ENEEDMORE : result;
         CHECK_THROW_ERROR(me_ptr->miid,
                           result,
                           "Insufficient size for drc config parameter. Received %lu bytes",
                           params_ptr->max_data_len);

         param_id_drc_config_t *drc_params_ptr = (param_id_drc_config_t *)params_ptr->data_ptr;
         drc_config_t           drc_config     = { 0 };
         drc_mode_t             drc_mode       = DRC_BYPASSED;
         memset(drc_params_ptr, 0, sizeof(param_id_drc_config_t));

         if (me_ptr->lib_handle.lib_mem_ptr)
         {
            uint32_t   temp = 0;
            DRC_RESULT lib_result =
               drc_get_param(&me_ptr->lib_handle, DRC_PARAM_CONFIG, (int8_t *)&drc_config, sizeof(drc_config), &temp);
            if (lib_result != DRC_SUCCESS)
            {
               DRC_MSG(me_ptr->miid, DBG_ERROR_PRIO, "drc config get param failed with error %lu", lib_result);
               return CAPI_EFAILED;
            }

            lib_result =
               drc_get_param(&me_ptr->lib_handle, DRC_PARAM_FEATURE_MODE, (int8_t *)&drc_mode, sizeof(drc_mode), &temp);
            if (lib_result != DRC_SUCCESS)
            {
               DRC_MSG(me_ptr->miid, DBG_ERROR_PRIO, "drc mode get param failed with error %lu", lib_result);
               return CAPI_EFAILED;
            }
         }
         else
         {
            memscpy(&drc_config, sizeof(drc_config), &me_ptr->lib_cfg, sizeof(me_ptr->lib_cfg));
            drc_mode = me_ptr->mode;
         }

         drc_params_ptr->delay_us               = me_ptr->delay_us;
         drc_params_ptr->mode                   = drc_mode;
         drc_params_ptr->channel_linked_mode    = drc_config.channelLinked;
         drc_params_ptr->down_sample_level      = drc_config.downSampleLevel;
         drc_params_ptr->down_cmpsr_attack      = drc_config.dnCompAttackUL32Q31;
         drc_params_ptr->down_cmpsr_hysteresis  = drc_config.dnCompHysterisisUL16Q14;
         drc_params_ptr->down_cmpsr_release     = drc_config.dnCompReleaseUL32Q31;
         drc_params_ptr->down_cmpsr_slope       = drc_config.dnCompSlopeUL16Q16;
         drc_params_ptr->down_cmpsr_threshold   = drc_config.dnCompThresholdL16Q7;
         drc_params_ptr->down_expdr_attack      = drc_config.dnExpaAttackUL32Q31;
         drc_params_ptr->down_expdr_hysteresis  = drc_config.dnExpaHysterisisUL16Q14;
         drc_params_ptr->down_expdr_min_gain_db = drc_config.dnExpaMinGainDBL32Q23;
         drc_params_ptr->down_expdr_release     = drc_config.dnExpaReleaseUL32Q31;
         drc_params_ptr->down_expdr_slope       = drc_config.dnExpaSlopeL16Q8;
         drc_params_ptr->down_expdr_threshold   = drc_config.dnExpaThresholdL16Q7;
         drc_params_ptr->makeup_gain            = drc_config.makeupGainUL16Q12;
         drc_params_ptr->rms_time_avg_const     = drc_config.rmsTavUL16Q16;
         drc_params_ptr->up_cmpsr_attack        = drc_config.upCompAttackUL32Q31;
         drc_params_ptr->up_cmpsr_hysteresis    = drc_config.upCompHysterisisUL16Q14;
         drc_params_ptr->up_cmpsr_release       = drc_config.upCompReleaseUL32Q31;
         drc_params_ptr->up_cmpsr_slope         = drc_config.upCompSlopeUL16Q16;
         drc_params_ptr->up_cmpsr_threshold     = drc_config.upCompThresholdL16Q7;

         params_ptr->actual_data_len = sizeof(param_id_drc_config_t);

         break;
      }
      default: {
         DRC_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Invalid getparam received 0x%lx", param_id);
         return CAPI_EUNSUPPORTED;
      }
   }
   return result;
}

static capi_err_t capi_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t result  = CAPI_EOK;
   capi_err_t result2 = CAPI_EOK;

   result |= (!_pif || !props_ptr) ? CAPI_EFAILED : result;
   CHECK_THROW_ERROR(MIID_UNKNOWN, result, "Error! set properties received null pointer.");

   capi_drc_t *me_ptr = (capi_drc_t *)_pif;

   result |= capi_cmn_set_basic_properties(props_ptr, (capi_heap_id_t *)&me_ptr->heap_id, &me_ptr->cb_info, TRUE);

   for (uint32_t i = 0; i < props_ptr->props_num; i++)
   {
      capi_prop_t *current_prop_ptr = &(props_ptr->prop_ptr[i]);
      capi_buf_t * payload_ptr      = &(current_prop_ptr->payload);
      result2                       = CAPI_EOK;
      switch (current_prop_ptr->id)
      {
         case CAPI_EVENT_CALLBACK_INFO:
         case CAPI_HEAP_ID:
         case CAPI_PORT_NUM_INFO: {
            continue;
         }
         case CAPI_ALGORITHMIC_RESET: {
            // Call drc algorithm reset here
            if (me_ptr->lib_handle.lib_mem_ptr)
            {
               drc_set_param(&me_ptr->lib_handle, DRC_PARAM_SET_RESET, NULL, 0);
            }
         }
         break;
         case CAPI_INPUT_MEDIA_FORMAT_V2: {
            // Payload can be no smaller than the header for media format
            if (payload_ptr->actual_data_len < CAPI_MF_V2_MIN_SIZE)
            {
               DRC_MSG(me_ptr->miid, DBG_ERROR_PRIO, "wrong size for input media format property.");
               CAPI_SET_ERROR(result2, CAPI_EBADPARAM);
               break;
            }

            capi_media_fmt_v2_t *fmt_ptr = (capi_media_fmt_v2_t *)payload_ptr->data_ptr;

            DRC_MSG(me_ptr->miid,
                    DBG_LOW_PRIO,
                    "Received input media format, sampling_rate(%ld), "
                    "num_channels(%ld), bit_width(%ld)",
                    fmt_ptr->format.sampling_rate,
                    fmt_ptr->format.num_channels,
                    fmt_ptr->format.bits_per_sample);

            result2 =
               (CAPI_FIXED_POINT != fmt_ptr->header.format_header.data_format) ||
                     ((0 == fmt_ptr->format.num_channels) || (CAPI_MAX_CHANNELS_V2 < fmt_ptr->format.num_channels)) ||
                     ((16 != fmt_ptr->format.bits_per_sample) && (32 != fmt_ptr->format.bits_per_sample)) ||
                     ((16 == fmt_ptr->format.bits_per_sample) && (PCM_Q_FACTOR_15 != fmt_ptr->format.q_factor)) ||
                     ((32 == fmt_ptr->format.bits_per_sample) && (PCM_Q_FACTOR_27 != fmt_ptr->format.q_factor))
                  ? CAPI_EBADPARAM
                  : CAPI_EOK;

            if (CAPI_FAILED(result2))
            {
               DRC_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Invalid input media format received.");
               break;
            }

            memscpy(&me_ptr->input_media_fmt, sizeof(me_ptr->input_media_fmt), fmt_ptr, payload_ptr->actual_data_len);

            // allocate drc memory based on the new static parameters
            result2 = drc_lib_alloc_init(me_ptr);

            raise_kpps_delay_process_events(me_ptr);

            // raise output media fmt
            capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info, fmt_ptr, FALSE, 0);
            break;
         }
         case CAPI_MODULE_INSTANCE_ID: {
            if (payload_ptr->actual_data_len >= sizeof(capi_module_instance_id_t))
            {
               capi_module_instance_id_t *data_ptr = (capi_module_instance_id_t *)payload_ptr->data_ptr;
               me_ptr->miid                        = data_ptr->module_instance_id;
#if 0
               DRC_MSG(me_ptr->miid,
                       DBG_LOW_PRIO,
                       "CAPI DRC: This module-id 0x%08lX, instance-id 0x%08lX",
                       data_ptr->module_id,
                       me_ptr->miid);
#endif
            }
            else
            {
               DRC_MSG(MIID_UNKNOWN,
                       DBG_ERROR_PRIO,
                       "CAPI DRC: Set, Param id 0x%lx Bad param size %lu",
                       (uint32_t)current_prop_ptr->id,
                       payload_ptr->actual_data_len);
               CAPI_SET_ERROR(result2, CAPI_ENEEDMORE);
            }
            break;
         }
         default: {
            DRC_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Invalid set property 0x%x.", (int)current_prop_ptr->id);
            CAPI_SET_ERROR(result2, CAPI_EUNSUPPORTED);
            break;
         }
      }

      if (CAPI_FAILED(result2))
      {
         CAPI_SET_ERROR(result, result2);
      }
   }
   return result;
}

static capi_err_t capi_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_err_t  result  = CAPI_EOK;
   capi_err_t  result2 = CAPI_EOK;
   capi_drc_t *me_ptr  = (capi_drc_t *)_pif;
   uint32_t    miid    = me_ptr ? me_ptr->miid : MIID_UNKNOWN;
   result |= (props_ptr == NULL || props_ptr->prop_ptr == NULL) ? CAPI_EBADPARAM : result;
   CHECK_THROW_ERROR(miid, result, "Bad Pointer received, Get property failed.");

   capi_prop_t *     prop_ptr = props_ptr->prop_ptr;
   uint32_t          i;
   capi_basic_prop_t mod_prop_ptr = { .init_memory_req    = sizeof(capi_drc_t),
                                      .stack_size         = 1024,
                                      .num_fwk_extns      = 0,
                                      .fwk_extn_ids_arr   = NULL,
                                      .is_inplace         = TRUE,
                                      .req_data_buffering = FALSE,
                                      .max_metadata_size  = 0 };

   result |= capi_cmn_get_basic_properties(props_ptr, &mod_prop_ptr);

   // iterating over the properties
   for (i = 0; i < props_ptr->props_num; i++)
   {
      capi_buf_t *payload_ptr = &prop_ptr[i].payload;

      switch (prop_ptr[i].id)
      {
         case CAPI_INIT_MEMORY_REQUIREMENT:
         case CAPI_STACK_SIZE:
         case CAPI_IS_INPLACE:
         case CAPI_REQUIRES_DATA_BUFFERING:
         case CAPI_NUM_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_NEEDED_FRAMEWORK_EXTENSIONS:
         case CAPI_MAX_METADATA_SIZE: {
            // handled in capi common utils.
            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_SIZE: {
            if (NULL == me_ptr)
            {
               DRC_MSG(miid, DBG_ERROR_PRIO, "Get property id 0x%lx, module is not allocated", prop_ptr[i].id);
               CAPI_SET_ERROR(result, CAPI_EBADPARAM);
               break;
            }

            result2 |=
               (payload_ptr->max_data_len < sizeof(capi_output_media_format_size_t)) ? CAPI_ENEEDMORE : CAPI_EOK;
            if (CAPI_FAILED(result2))
            {
               payload_ptr->actual_data_len = 0;
               DRC_MSG(miid, DBG_ERROR_PRIO, "Insufficient get property size.");
               CAPI_SET_ERROR(result, result2);
               break;
            }

            capi_output_media_format_size_t *data_ptr = (capi_output_media_format_size_t *)payload_ptr->data_ptr;

            // One channel
            data_ptr->size_in_bytes = sizeof(capi_standard_data_format_v2_t) +
                                      me_ptr->input_media_fmt.format.num_channels * sizeof(capi_channel_type_t);

            payload_ptr->actual_data_len = sizeof(capi_output_media_format_size_t);

            break;
         }
         case CAPI_OUTPUT_MEDIA_FORMAT_V2: {
            if (NULL == me_ptr)
            {
               DRC_MSG(miid, DBG_ERROR_PRIO, "Get property id 0x%lx, module is not allocated", prop_ptr[i].id);
               CAPI_SET_ERROR(result, CAPI_EBADPARAM);
               break;
            }

            // One Channel
            uint32_t required_size = sizeof(capi_set_get_media_format_t) + sizeof(capi_standard_data_format_v2_t) +
                                     me_ptr->input_media_fmt.format.num_channels * sizeof(capi_channel_type_t);

            result2 |= (payload_ptr->max_data_len < required_size) ? CAPI_ENEEDMORE : CAPI_EOK;
            if (CAPI_FAILED(result2))
            {
               payload_ptr->actual_data_len = 0;
               DRC_MSG(miid, DBG_ERROR_PRIO, "Insufficient get property size.");
               CAPI_SET_ERROR(result, result2);
               break;
            }

            capi_media_fmt_v2_t *data_ptr = (capi_media_fmt_v2_t *)payload_ptr->data_ptr;

            payload_ptr->actual_data_len =
               memscpy(data_ptr, required_size, &me_ptr->input_media_fmt, sizeof(me_ptr->input_media_fmt));

            break;
         }

         case CAPI_INTERFACE_EXTENSIONS: {
            capi_interface_extns_list_t *intf_ext_list = (capi_interface_extns_list_t *)payload_ptr->data_ptr;
            result2 |=
               ((payload_ptr->max_data_len < sizeof(capi_interface_extns_list_t)) ||
                (payload_ptr->max_data_len < (sizeof(capi_interface_extns_list_t) +
                                              (intf_ext_list->num_extensions * sizeof(capi_interface_extn_desc_t)))))
                  ? CAPI_ENEEDMORE
                  : result;

            if (CAPI_FAILED(result2))
            {
               payload_ptr->actual_data_len = 0;
               DRC_MSG(miid, DBG_ERROR_PRIO, "Insufficient get property size.");
               CAPI_SET_ERROR(result, result2);
               break;
            }

            capi_interface_extn_desc_t *curr_intf_extn_desc_ptr =
               (capi_interface_extn_desc_t *)(payload_ptr->data_ptr + sizeof(capi_interface_extns_list_t));

            for (uint32_t j = 0; j < intf_ext_list->num_extensions; j++)
            {
               switch (curr_intf_extn_desc_ptr->id)
               {
                  case INTF_EXTN_IMCL:
                     curr_intf_extn_desc_ptr->is_supported = TRUE;
                     break;
                  default: {
                     curr_intf_extn_desc_ptr->is_supported = FALSE;
                     break;
                  }
               }
               curr_intf_extn_desc_ptr++;
            }

            break;
         }

         default: {
            payload_ptr->actual_data_len = 0;
            DRC_MSG(miid, DBG_ERROR_PRIO, "Unsupported getproperty 0x%x.", prop_ptr[i].id);
            result |= CAPI_EUNSUPPORTED;
            break;
         }
      } // switch
   }
   return result;
}

static capi_err_t capi_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t result                        = CAPI_EOK;
   DRC_RESULT lib_result                    = DRC_SUCCESS;
   int8_t *   in_ptr[CAPI_MAX_CHANNELS_V2]  = { NULL };
   int8_t *   out_ptr[CAPI_MAX_CHANNELS_V2] = { NULL };

   uint32_t samples_to_process = 0, bytes_to_process = 0, shift_factor = 0;

   if ((NULL == _pif) || (NULL == input) || (NULL == output))
   {
      AR_MSG(DBG_ERROR_PRIO, "VCP:ACP: drc_process: received bad pointer");
      return CAPI_EFAILED;
   }
   capi_drc_t *me_ptr = (capi_drc_t *)_pif;

   // populate input buffers, output buffers and framesize
   for (uint32_t i = 0; i < me_ptr->lib_static_cfg.num_channel && i < CAPI_MAX_CHANNELS_V2; i++)
   {
      in_ptr[i]  = (int8_t *)(input[0]->buf_ptr[i].data_ptr);
      out_ptr[i] = (int8_t *)(output[0]->buf_ptr[i].data_ptr);
   }
   bytes_to_process   = (input[0]->buf_ptr[0].actual_data_len > output[0]->buf_ptr[0].max_data_len)
                           ? output[0]->buf_ptr[0].max_data_len
                           : input[0]->buf_ptr[0].actual_data_len;
   shift_factor       = (me_ptr->lib_static_cfg.data_width == BITS_16) ? 1 : 2;
   samples_to_process = (bytes_to_process >> shift_factor);

   if (0 < samples_to_process)
   {
      lib_result = drc_process(&me_ptr->lib_handle, out_ptr, in_ptr, samples_to_process);
      if (DRC_SUCCESS != lib_result)
      {
         DRC_MSG(me_ptr->miid, DBG_ERROR_PRIO, "process failed with result %lu", lib_result);
         result = CAPI_EFAILED;
      }
   }

   for (uint32_t i = 0; i < me_ptr->lib_static_cfg.num_channel && i < CAPI_MAX_CHANNELS_V2; i++)
   {
      input[0]->buf_ptr[i].actual_data_len  = samples_to_process << shift_factor;
      output[0]->buf_ptr[i].actual_data_len = samples_to_process << shift_factor;
   }

   output[0]->flags = input[0]->flags;
   if (input[0]->flags.is_timestamp_valid)
   {
      output[0]->timestamp = input[0]->timestamp - me_ptr->delay_us;
   }

   return result;
}

static capi_err_t capi_end(capi_t *_pif)
{
   capi_err_t  result = CAPI_EOK;
   capi_drc_t *me_ptr = (capi_drc_t *)_pif;

   result |= (NULL == _pif) ? CAPI_EFAILED : result;
   CHECK_THROW_ERROR(MIID_UNKNOWN, result, "end received null pointer.");

   uint32_t miid = me_ptr->miid;

   capi_cmn_ctrl_port_list_deinit(&me_ptr->ctrl_port_info);

   if (me_ptr->lib_handle.lib_mem_ptr)
   {
      posal_memory_free(me_ptr->lib_handle.lib_mem_ptr);
   }

   me_ptr->vtbl = NULL;

   DRC_MSG(miid, DBG_HIGH_PRIO, "end.");

   return CAPI_EOK;
}
