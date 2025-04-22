/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_soft_vol.cpp
 *
 * Implementation for soft_vol module
 */

#include "capi_soft_vol.h"
#include "capi_soft_vol_utils.h"

//#define SOFT_VOL_DEBUG 1

#ifdef DO_SOFT_VOL_PROFILING
#include <q6sim_timer.h>
#endif

capi_err_t capi_soft_vol_get_static_properties(capi_proplist_t *init_set_properties,
                                                     capi_proplist_t *static_properties)
{
   return capi_soft_vol_process_get_properties((capi_soft_vol_t *)NULL, static_properties);
}

capi_err_t capi_soft_vol_init(capi_t *_pif, capi_proplist_t *init_set_properties)
{
   capi_err_t result = CAPI_EOK;

   if (NULL == _pif)
      return CAPI_EBADPARAM;

   capi_soft_vol_t *me_ptr = (capi_soft_vol_t *)_pif;
   memset(me_ptr, 0, sizeof(capi_soft_vol_t));
   me_ptr->soft_vol_state = SOFT_VOL_WAITING_FOR_MEDIA_FORMAT;
   me_ptr->vtbl_ptr = capi_soft_vol_get_vtbl(); // assigning the vtbl with all function pointers

   result = capi_cmn_init_media_fmt_v2(&me_ptr->input_media_fmt);

   capi_soft_vol_init_lib_memory(me_ptr);

   /* init_set_properties contains OUT_BITS_PER_SAMPLE,
    * EVENT_CALLBACK_INFO and PORT_INFO */
   result = capi_soft_vol_process_set_properties(me_ptr, init_set_properties);

   // Initialize the control port list.
   capi_cmn_ctrl_port_list_init(&me_ptr->ctrl_port_info);

   return result;
}

capi_err_t capi_soft_vol_end(capi_t *_pif)
{
   if (NULL == _pif)
   {
      SOFT_VOL_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "End received bad pointer, 0x%p", _pif);
      return CAPI_EBADPARAM;
   }

   capi_soft_vol_t *me_ptr = (capi_soft_vol_t *)(_pif);

#ifdef DO_SOFT_VOL_PROFILING
   capi_soft_vol_print_kpps(me_ptr);
#endif

   CSoftVolumeControlsLib *pLib = &me_ptr->SoftVolumeControlsLib;
   posal_memory_placement_delete(pLib, CSoftVolumeControlsLib);
   capi_cmn_ctrl_port_list_deinit(&me_ptr->ctrl_port_info);
   me_ptr->vtbl_ptr = NULL;

   return CAPI_EOK;
}

capi_err_t capi_soft_vol_set_param(capi_t *                _pif,
                                                uint32_t                   param_id,
                                                const capi_port_info_t *port_info_ptr,
                                                capi_buf_t *            params_ptr)
{
   if (NULL == _pif || NULL == params_ptr)
   {
      SOFT_VOL_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO, "Set param received bad pointers 0x%p, 0x%p", _pif, params_ptr);
      return CAPI_EBADPARAM;
   }
   capi_err_t       result              = CAPI_EOK;
   capi_soft_vol_t *me_ptr              = (capi_soft_vol_t *)(_pif);
   me_ptr->adjust_volume_based_on_headroom = 0;
   me_ptr->update_gain_over_imcl           = TRUE;

   switch (param_id)
   {
      case PARAM_ID_VOL_CTRL_MASTER_GAIN:
      {
         if (params_ptr->actual_data_len >= sizeof(volume_ctrl_master_gain_t))
         {
            volume_ctrl_master_gain_t *pMasterGainPacket = (volume_ctrl_master_gain_t *)(params_ptr->data_ptr);
            me_ptr->soft_vol_lib.masterGain              = pMasterGainPacket->master_gain;
            for (uint32_t i = MIN_CHANNEL_TYPE; i <= MAX_CHANNEL_TYPE; i++)
            {
               uint32 gainQ28 =
                  capi_soft_vol_calc_gain_q28(me_ptr->soft_vol_lib.masterGain, me_ptr->soft_vol_lib.channelGain[i]);
               me_ptr->SoftVolumeControlsLib.SetVolume(gainQ28, me_ptr->soft_vol_lib.pPerChannelData[i]);
            }
            result = CAPI_EOK;

            SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                   "Set param PARAM_ID_VOL_CTRL_MASTER_GAIN, 0x%x",
                   pMasterGainPacket->master_gain);
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "Set Master Gain, Bad param size %lu",
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }
      case PARAM_ID_VOL_CTRL_MASTER_MUTE:
      {
         if (params_ptr->actual_data_len >= sizeof(volume_ctrl_master_mute_t))

         {
            volume_ctrl_master_mute_t *pMutePacket = (volume_ctrl_master_mute_t *)((params_ptr->data_ptr));
            capi_soft_vol_set_mute_for_all_channels(me_ptr, pMutePacket->mute_flag);

            result = CAPI_EOK;

            SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                   "Set param PARAM_ID_VOL_CTRL_MASTER_MUTE, %lu",
                   pMutePacket->mute_flag);
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set Mute, Bad param size %lu", params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }
      case PARAM_ID_VOL_CTRL_GAIN_RAMP_PARAMETERS:
      {
         if (params_ptr->actual_data_len >= sizeof(volume_ctrl_gain_ramp_params_t))
         {
            volume_ctrl_gain_ramp_params_t *p_soft_stepping_pkt =
               (volume_ctrl_gain_ramp_params_t *)(params_ptr->data_ptr);
            SoftSteppingParams params;
            capi_set_soft_stepping_param(me_ptr,
                                            &params,
                                            p_soft_stepping_pkt->period_ms,
                                            p_soft_stepping_pkt->step_us,
                                            p_soft_stepping_pkt->ramping_curve);
            me_ptr->SoftVolumeControlsLib.SetSoftVolumeParams(params);

            SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                   "Set PARAM_ID_VOL_CTRL_GAIN_RAMP_PARAMETERS: period  %d, step %d, curve %d ",
                   p_soft_stepping_pkt->period_ms,
                   p_soft_stepping_pkt->step_us,
                   p_soft_stepping_pkt->ramping_curve);
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "Set gain ramp params, Bad param size %lu",
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }
      case PARAM_ID_VOL_CTRL_MUTE_RAMP_PARAMETERS:
      {
         if (params_ptr->actual_data_len >= sizeof(volume_ctrl_mute_ramp_params_t))
         {
            volume_ctrl_mute_ramp_params_t *p_soft_stepping_pkt =
               (volume_ctrl_mute_ramp_params_t *)(params_ptr->data_ptr);
            SoftSteppingParams params;
            capi_set_soft_stepping_param(me_ptr,
                                            &params,
                                            p_soft_stepping_pkt->period_ms,
                                            p_soft_stepping_pkt->step_us,
                                            p_soft_stepping_pkt->ramping_curve);
            me_ptr->SoftVolumeControlsLib.SetSoftMuteParams(params);

            SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                   "Set PARAM_ID_VOL_CTRL_MUTE_RAMP_PARAMETERS: period  %d, step %d, curve %d ",
                   p_soft_stepping_pkt->period_ms,
                   p_soft_stepping_pkt->step_us,
                   p_soft_stepping_pkt->ramping_curve);
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "Set mute ramp params, Bad param size %lu",
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }
      case PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN:
      {
         if (params_ptr->actual_data_len >= sizeof(volume_ctrl_multichannel_gain_t))
         {
            volume_ctrl_multichannel_gain_t *pVolumePkt = (volume_ctrl_multichannel_gain_t *)(params_ptr->data_ptr);

            uint32_t param_total_size =
               (pVolumePkt->num_config * sizeof(volume_ctrl_channels_gain_config_t)) + sizeof(uint32_t);
            if (params_ptr->actual_data_len < param_total_size)
            {
               SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Insufficient payload size %d", params_ptr->actual_data_len);
               result = CAPI_ENEEDMORE;
            }
            else
            {
               SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Set PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN ");
               result = capi_soft_vol_set_multichannel_gain(me_ptr, pVolumePkt);
            }
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "Set multichannel volume, Bad param size %lu",
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }
      case PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN_V2:
      {
         if (params_ptr->actual_data_len >= sizeof(volume_ctrl_multichannel_gain_v2_t))
         {
            volume_ctrl_multichannel_gain_v2_t *pVolumePkt = ((volume_ctrl_multichannel_gain_v2_t *)params_ptr->data_ptr);
            int8_t                    *pVolumePayload = params_ptr->data_ptr;

            //  validate received num of cfg
            const uint32_t num_cfg = pVolumePkt->num_config;
            if (num_cfg < 1 || num_cfg > PCM_MAX_CHANNEL_MAP_V2)
            {
               SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Received incorrect num_config parameter - %lu", num_cfg);
               return CAPI_EBADPARAM;
            }
            // validate received payload
            uint32_t required_size = 0;
            result = capi_soft_vol_validate_multichannel_payload(me_ptr, num_cfg, pVolumePayload, params_ptr->actual_data_len,
                                                                 &required_size, param_id);
            if (CAPI_FAILED(result))
            {
               SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Multichannel gain SetParam 0x%lx, invalid param size %lu ,required_size %lu",
                               param_id,
                               params_ptr->actual_data_len,
                               required_size);
               return result;
            }
            if (params_ptr->actual_data_len < required_size)
            {
               SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Insufficient payload size %d. Req size %lu", params_ptr->actual_data_len, required_size);
               result = CAPI_ENEEDMORE;
            }
            else
            {
               SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Set PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN_V2 ");
               result = capi_soft_vol_set_multichannel_gain_v2(me_ptr, pVolumePkt);
            }
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "Set multichannel volume, Bad param size %lu",
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }

      case PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE:
      {
         if (params_ptr->actual_data_len >= sizeof(volume_ctrl_multichannel_mute_t))
         {
            volume_ctrl_multichannel_mute_t *pMutePkt = (volume_ctrl_multichannel_mute_t *)(params_ptr->data_ptr);
            uint32_t                         param_total_size =
               (pMutePkt->num_config * sizeof(volume_ctrl_channels_mute_config_t)) + sizeof(uint32_t);
            if (params_ptr->actual_data_len < param_total_size)
            {
               SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Insufficient payload size %d", params_ptr->actual_data_len);
               result = CAPI_ENEEDMORE;
            }
            else
            {
               SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Set PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE ");
               result = capi_soft_vol_set_multichannel_mute(me_ptr, pMutePkt);
            }
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "Set multichannel mute, Bad param size %lu",
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }
      case PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE_V2:
      {
         if (params_ptr->actual_data_len >= sizeof(volume_ctrl_multichannel_mute_v2_t))
         {
            volume_ctrl_multichannel_mute_v2_t *pMutePkt = ((volume_ctrl_multichannel_mute_v2_t *)params_ptr->data_ptr);
            int8_t                    *pMutePayload = params_ptr->data_ptr;
            //  validate received num of cfg
            const uint32_t num_cfg = pMutePkt->num_config;
            if (num_cfg < 1 || num_cfg > PCM_MAX_CHANNEL_MAP_V2)
            {
               SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Received incorrect num_config parameter - %lu", num_cfg);
               return CAPI_EBADPARAM;
            }
            // validate received payload size
            uint32_t required_size = 0;
            result = capi_soft_vol_validate_multichannel_payload(me_ptr, num_cfg, pMutePayload, params_ptr->actual_data_len,
                                                                 &required_size, param_id);
            if (CAPI_FAILED(result))
            {
               SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Multichannel mute SetParam 0x%lx, invalid param size %lu ,required_size %lu",
                               param_id,
                               params_ptr->actual_data_len,
                               required_size);
               return result;
            }
            else
            {
               SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Set PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE_V2 ");
               result = capi_soft_vol_set_multichannel_mute_v2(me_ptr, pMutePkt);
            }
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "Set multichannel mute, Bad param size %lu",
                   params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }

      case PARAM_ID_ALGORITHMIC_HEADROOM:
      {
         if (params_ptr->actual_data_len >= sizeof(int32_t))
         {
            int32_t headroom_mB = int32_t(*((int32_t *)(params_ptr->data_ptr)));
            if (me_ptr->soft_vol_lib.headroom_mB != headroom_mB)
            {
               me_ptr->soft_vol_lib.headroom_mB = headroom_mB;
            }
            else
            {
               me_ptr->adjust_volume_based_on_headroom = 0;
            }
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Headroom, Bad param size %lu", params_ptr->actual_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }
      case INTF_EXTN_PARAM_ID_IMCL_PORT_OPERATION:
      {
         uint32_t supported_intent[3] = { INTENT_ID_GAIN_INFO, INTENT_ID_P_EQ_VOL_HEADROOM, INTENT_ID_MUTE };
         result                       = capi_cmn_ctrl_port_operation_handler(&me_ptr->ctrl_port_info,
                                                          params_ptr,
                                                          (POSAL_HEAP_ID)me_ptr->heap_id,
                                                          0,
                                                          3,
                                                          supported_intent);
         break;
      }
      case INTF_EXTN_PARAM_ID_IMCL_INCOMING_DATA:
      {
         result = capi_vol_imc_set_param_handler(me_ptr, params_ptr);
         if(CAPI_EOK != result)
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "IMC set param handler failed 0x%x \n", param_id);
         }
         break;
      }
      default:
      {
         SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Set, unsupported param ID %#x", param_id);
         result = CAPI_EUNSUPPORTED;
         break;
      }
   } /* switch (param_id) */

   if (me_ptr->adjust_volume_based_on_headroom)
      result |= capi_soft_vol_headroom_gain(me_ptr, me_ptr->soft_vol_lib.headroom_mB);

   if (SOFT_VOL_ENABLE == me_ptr->soft_vol_state)
   {
      uint32_t process_check = 0;
      for (uint32_t i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
      {
         if (!me_ptr->SoftVolumeControlsLib.isUnityGain(
                me_ptr->soft_vol_lib.pPerChannelData[me_ptr->soft_vol_lib.channelMapping[i]]))
         {
            process_check = 1;
            break;
         }
      }
      result |= capi_cmn_update_process_check_event(&me_ptr->cb_info, process_check);
   }
   return result;
}

capi_err_t capi_soft_vol_get_param(capi_t *                _pif,
                                                uint32_t                   param_id,
                                                const capi_port_info_t *port_info_ptr,
                                                capi_buf_t *            params_ptr)

{
   if (NULL == _pif || NULL == params_ptr)
   {
      SOFT_VOL_MSG(MIID_UNKNOWN, DBG_ERROR_PRIO,
             "Get param received bad pointer,"
             " 0x%p, 0x%p",
             _pif,
             params_ptr);
      return CAPI_EBADPARAM;
   }

   capi_err_t       result = CAPI_EOK;
   capi_soft_vol_t *me_ptr = (capi_soft_vol_t *)(_pif);

   if ((TRUE == me_ptr->higher_channel_map_present) && ((PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN == param_id) || (PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE == param_id)))
   {
      SOFT_VOL_MSG(me_ptr->miid,DBG_ERROR_PRIO, "GetParam 0x%x failed as higher than 63 channel map present in IMF (0(No)/1(Yes)): %lu."
                   "V1 API is not sufficient to have higher channel maps.",
                   (int)param_id,
                   me_ptr->higher_channel_map_present);
      return CAPI_EBADPARAM;
   }
   switch (param_id)
   {
      case PARAM_ID_VOL_CTRL_MASTER_GAIN:
      {
         if (params_ptr->max_data_len >= sizeof(volume_ctrl_master_gain_t))
         {
            volume_ctrl_master_gain_t *pMasterGainPacket = (volume_ctrl_master_gain_t *)(params_ptr->data_ptr);

            pMasterGainPacket->master_gain = me_ptr->soft_vol_lib.masterGain;
            pMasterGainPacket->reserved    = 0;
            params_ptr->actual_data_len    = sizeof(volume_ctrl_master_gain_t);
#ifdef CAPI_SOFT_VOLUME_DEBUG_MSG
            SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Get Master Gain, %d", pMasterGainPacket->master_gain);
#endif
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Get Master Gain, Bad param size %lu", params_ptr->max_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }
      case PARAM_ID_VOL_CTRL_MASTER_MUTE:
      {
         if (params_ptr->max_data_len >= sizeof(volume_ctrl_master_mute_t))
         {
            volume_ctrl_master_mute_t *pMutePacket = (volume_ctrl_master_mute_t *)(params_ptr->data_ptr);
            pMutePacket->mute_flag                 = 1;
            for (uint32_t i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
            {
               if (!me_ptr->SoftVolumeControlsLib.IsMuted(
                      me_ptr->soft_vol_lib.pPerChannelData[me_ptr->soft_vol_lib.channelMapping[i]]))
               {
                  pMutePacket->mute_flag = 0;
                  break;
               }
            }
            params_ptr->actual_data_len = sizeof(volume_ctrl_master_mute_t);
            result                      = AR_EOK;
#ifdef CAPI_SOFT_VOLUME_DEBUG_MSG
            SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Get Mute, %lu", pMutePacket->mute_flag);
#endif
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Get Mute, Bad param size %lu", params_ptr->max_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }
      case PARAM_ID_VOL_CTRL_GAIN_RAMP_PARAMETERS:
      {
         if (params_ptr->max_data_len >= sizeof(volume_ctrl_gain_ramp_params_t))
         {
            volume_ctrl_gain_ramp_params_t *p_soft_stepping_pkt =
               (volume_ctrl_gain_ramp_params_t *)(params_ptr->data_ptr);
            SoftSteppingParams params;
            me_ptr->SoftVolumeControlsLib.GetSoftVolumeParams(&params);
            capi_get_soft_stepping_param(me_ptr,
                                            &params,
                                            &p_soft_stepping_pkt->period_ms,
                                            &p_soft_stepping_pkt->step_us,
                                            &p_soft_stepping_pkt->ramping_curve);
            params_ptr->actual_data_len = sizeof(volume_ctrl_gain_ramp_params_t);
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "Get gain ramp params, Bad param size %lu",
                   params_ptr->max_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }
      case PARAM_ID_VOL_CTRL_MUTE_RAMP_PARAMETERS:
      {
         if (params_ptr->max_data_len >= sizeof(volume_ctrl_mute_ramp_params_t))
         {
            volume_ctrl_mute_ramp_params_t *p_soft_stepping_pkt =
               (volume_ctrl_mute_ramp_params_t *)(params_ptr->data_ptr);
            SoftSteppingParams params;
            me_ptr->SoftVolumeControlsLib.GetSoftMuteParams(&params);
            capi_get_soft_stepping_param(me_ptr,
                                            &params,
                                            &p_soft_stepping_pkt->period_ms,
                                            &p_soft_stepping_pkt->step_us,
                                            &p_soft_stepping_pkt->ramping_curve);
            params_ptr->actual_data_len = sizeof(volume_ctrl_mute_ramp_params_t);
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "Get Mute ramp params, Bad param size %lu",
                   params_ptr->max_data_len);
            result = CAPI_ENEEDMORE;
         }
         break;
      }
      case PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN:
      {
         if(me_ptr->higher_channel_map_present)
         {
             SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,"Get Multichannel Gain, not supported with V1 param, as higher channel maps are present in input media format.");
             return CAPI_EBADPARAM;
         }
         uint32_t num_configs = capi_soft_vol_get_multichannel_gain(me_ptr, NULL);
         uint32_t req_size =
            sizeof(volume_ctrl_multichannel_gain_t) + (num_configs * sizeof(volume_ctrl_channels_gain_config_t));

         if (params_ptr->max_data_len >= req_size)
         {
            volume_ctrl_multichannel_gain_t *multi_gain_ptr = (volume_ctrl_multichannel_gain_t *)(params_ptr->data_ptr);
            capi_soft_vol_get_multichannel_gain(me_ptr, multi_gain_ptr);
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "multichannel volume, Bad param size %lu",
                   params_ptr->max_data_len);
            result                      = CAPI_ENEEDMORE;
         }
         params_ptr->actual_data_len = req_size;
         break;
      }
      case PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE:
      {
         if(me_ptr->higher_channel_map_present)
         {
             SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,"Get Multichannel Mute, not supported with V1 param, as higher channel maps are present in input media format.");
             return CAPI_EBADPARAM;
         }
         //num configs is 1
         uint32_t req_size = params_ptr->actual_data_len =
            sizeof(volume_ctrl_multichannel_mute_t) + (sizeof(volume_ctrl_channels_mute_config_t));

         if (params_ptr->max_data_len >= sizeof(volume_ctrl_multichannel_mute_t))
         {
            volume_ctrl_multichannel_mute_t *multi_mute_ptr = (volume_ctrl_multichannel_mute_t *)(params_ptr->data_ptr);
            capi_soft_vol_get_multichannel_mute(me_ptr, multi_mute_ptr);
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "Get Multichannel Mute, Bad param size %lu",
                   params_ptr->max_data_len);
            result = CAPI_ENEEDMORE;
         }
         params_ptr->actual_data_len = req_size;
         break;
      }
      case PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN_V2:
      {
         capi_soft_vol_multich_gain_info gain_list[VOLUME_CONTROL_MAX_CHANNELS] = { { 0 } }; // list to maintain all unique gains
         uint32_t req_size = 0;
         uint32_t num_unique_gains = 0;

         if (params_ptr->max_data_len >= sizeof(volume_ctrl_multichannel_gain_v2_t))
         {
             num_unique_gains = capi_soft_vol_get_multichannel_gain_v2_payload_size(me_ptr, gain_list, &req_size);
#ifdef SOFT_VOL_DEBUG
             SOFT_VOL_MSG(me_ptr->miid, DBG_MED_PRIO,"num_gains: %lu, req size for the payload is %lu.",num_unique_gains, req_size);
#endif
             if (params_ptr->max_data_len >= req_size)
             {
                result = capi_soft_vol_get_multichannel_gain_v2(me_ptr, params_ptr->data_ptr, gain_list, req_size, num_unique_gains);
             }
             else
             {
                SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                       "multichannel volume, Bad param size %lu",
                       params_ptr->max_data_len);
                result                  = CAPI_ENEEDMORE;
             }
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                   "multichannel volume, Bad param size %lu",
                   params_ptr->max_data_len);
            result                      = CAPI_ENEEDMORE;
         }
         params_ptr->actual_data_len = req_size;
         break;
      }
      case PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE_V2:
      {
         uint32_t mute_chmask_list[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS] = { 0 };
         uint32_t req_size = 0;
         if(params_ptr->max_data_len >= sizeof(volume_ctrl_multichannel_mute_v2_t))
         {
            result = capi_soft_vol_get_multichannel_mute_v2_payload_size(me_ptr, mute_chmask_list, &req_size);
            if (params_ptr->max_data_len >= req_size)
            {
               capi_soft_vol_get_multichannel_mute_v2(me_ptr, params_ptr->data_ptr, mute_chmask_list);
            }
            else
            {
               SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                      "Get Multichannel Mute, Bad param size %lu",
                      params_ptr->max_data_len);
               params_ptr->actual_data_len = req_size;
               result = CAPI_ENEEDMORE;
            }
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Get Multichannel Mute, Bad param size %lu",
                         params_ptr->max_data_len);
            return CAPI_ENEEDMORE;
         }
         params_ptr->actual_data_len = req_size;
         break;
      }
      default:
      {
         SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "get, unsupported param ID %#x", param_id);
         result |= CAPI_EUNSUPPORTED;
         break;
      }
   }
   return result;
}

capi_err_t capi_soft_vol_set_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_soft_vol_t *me_ptr = (capi_soft_vol_t *)_pif;
   return capi_soft_vol_process_set_properties(me_ptr, props_ptr);
}

capi_err_t capi_soft_vol_get_properties(capi_t *_pif, capi_proplist_t *props_ptr)
{
   capi_soft_vol_t *me_ptr = (capi_soft_vol_t *)_pif;
   return capi_soft_vol_process_get_properties(me_ptr, props_ptr);
}
