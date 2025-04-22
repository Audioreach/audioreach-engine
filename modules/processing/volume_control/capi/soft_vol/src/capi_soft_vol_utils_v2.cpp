/* =========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_soft_vol_utils_v2.cpp
 *
 * Utility functions for soft_vol module.
 */

#include "ar_defs.h"
#include "audpp_util.h"
#include "capi_soft_vol_utils.h"
#include "audio_basic_op_ext.h"
#include "audio_exp10.h"

capi_err_t capi_soft_vol_validate_multichannel_payload(capi_soft_vol_t *me_ptr,
                                                       uint32_t         num_cfg,
                                                       int8_t *         payload_cfg_ptr,
                                                       uint32_t         param_size,
                                                       uint32_t *       required_size_ptr,
                                                       uint32_t         param_id)
{
   capi_err_t capi_result             = CAPI_EOK;
   int8_t * base_payload_ptr          = payload_cfg_ptr;
   uint32_t per_cfg_base_payload_size = 0;
   uint32_t config_size               = 0;
   switch (param_id)
   {
      case PARAM_ID_VOL_CTRL_MULTICHANNEL_GAIN_V2: {
         *required_size_ptr += sizeof(volume_ctrl_multichannel_gain_v2_t);
         per_cfg_base_payload_size = MULT_CH_GAIN_BASE_PYLD_SIZE;
         break;
      }
      case PARAM_ID_VOL_CTRL_MULTICHANNEL_MUTE_V2: {
         *required_size_ptr += sizeof(volume_ctrl_multichannel_mute_v2_t);
         per_cfg_base_payload_size = MULT_CH_MUTE_BASE_PYLD_SIZE;
      }
   }
   base_payload_ptr += *required_size_ptr; // 4
   // configuration payload
   // _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
   //| num_ | channel_type_group_mask | dynamic channel_type_mask_list[0] | payload1(mute/gain) |_ _ _
   //|config| _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _  |_ _ _ _ _ _ _ _ _ _  |
   //
#ifdef SOFT_VOL_DEBUG
   SOFT_VOL_MSG(me_ptr->miid, DBG_LOW_PRIO, "Validating received payload for %#x.", param_id);
#endif
   for (uint32_t count = 0; count < num_cfg; count++)
   {
      if ((*required_size_ptr + per_cfg_base_payload_size) > param_size)
      {
         SOFT_VOL_MSG(me_ptr->miid,
                      DBG_ERROR_PRIO,
                      "Failed while validating required size for config #%lu. Total config: %lu",
                      count + 1,
                      num_cfg);
         return CAPI_ENEEDMORE;
      }
      else
      {
         uint32_t ch_type_group_mask = 0;
         base_payload_ptr += config_size;
         ch_type_group_mask = *((uint32_t *)base_payload_ptr);

#ifdef SOFT_VOL_DEBUG
         SOFT_VOL_MSG(me_ptr->miid,
                      DBG_MED_PRIO,
                      "channel group mask %lu received for config %lu.",
                      ch_type_group_mask,
					  count + 1);
#endif
         // check for group mask payload if only desired bits are set or not
         if (0 == (ch_type_group_mask >> CAPI_CMN_MAX_CHANNEL_MAP_GROUPS))
         {
             capi_result = capi_cmn_check_payload_validation(me_ptr->miid,
            		                              ch_type_group_mask,
												  per_cfg_base_payload_size,
            		                              count + 1,
												  param_size,
												  &config_size,
												  required_size_ptr);
             if(CAPI_FAILED(capi_result))
             {
                return capi_result;
             }
         }
         else
         {
            SOFT_VOL_MSG(me_ptr->miid,
                         DBG_ERROR_PRIO,
                         "Invalid configuration of channel group mask: %lu. More than maximum valid channel groups are "
                         "set, maximum valid channel groups: %lu.",
                         ch_type_group_mask,
                         CAPI_CMN_MAX_CHANNEL_MAP_GROUPS);
            return CAPI_EBADPARAM;
         }
      }
   }
#ifdef SOFT_VOL_DEBUG
   SOFT_VOL_MSG(me_ptr->miid, DBG_MED_PRIO, "Total calculated size for param is %lu", *required_size_ptr);
#endif

   // validate the configs for duplication
   if (!capi_soft_vol_check_multi_ch_channel_mask_v2_param(me_ptr->miid,
				                                     num_cfg,
													 param_id,
		                                             (int8_t *)payload_cfg_ptr,
													 sizeof(volume_ctrl_multichannel_gain_v2_t),
													 per_cfg_base_payload_size))
   {
      SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "Trying to set different gain values to same channel, returning.");
      return CAPI_EBADPARAM;
   }
   return CAPI_EOK;
}
/* Returns FALSE if same channel is set to multiple times in different configs */
bool_t capi_soft_vol_check_multi_ch_channel_mask_v2_param(uint32_t miid,
		                                    uint32_t    num_config,
                                            uint32_t    param_id,
                                            int8_t *    param_ptr,
                                            uint32_t    base_payload_size,
											uint32_t per_cfg_base_payload_size)
{
   uint32_t *temp_mask_list_ptr                               = NULL;
   int8_t *  data_ptr                                         = param_ptr; // points to the start of the payload
   uint32_t  check_channel_mask_arr[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS]   = { 0 };
   uint32_t  current_channel_mask_arr[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS] = { 0 };
   bool_t    check                                            = TRUE;
   uint32_t  offset                                           = base_payload_size;

   for (uint32_t cnfg_cntr = 0; cnfg_cntr < num_config; cnfg_cntr++)
   {
      uint32_t channel_group_mask = 0;
      // Increment data ptr by calculated offset to point at next payload's group_mask
      data_ptr += offset;
      channel_group_mask          = *((uint32_t *)data_ptr);
#ifdef SOFT_VOL_DEBUG
         SOFT_VOL_MSG(miid,
                      DBG_MED_PRIO,
                      "channel group mask %#lx received for config %lu.",
					  channel_group_mask,
					  cnfg_cntr + 1);
#endif
      temp_mask_list_ptr = (uint32_t *)(data_ptr + CAPI_CMN_INT32_SIZE_IN_BYTES);
      if(!capi_cmn_check_v2_channel_mask_duplication(miid,
    		                                         cnfg_cntr,
    		                                         channel_group_mask,
    		                                         temp_mask_list_ptr,
													 current_channel_mask_arr,
													 check_channel_mask_arr,
													 &offset,
													 per_cfg_base_payload_size))
      {
    	  return FALSE;
      }
   }
   return check;
}

capi_err_t capi_soft_vol_set_multichannel_gain_v2(capi_soft_vol_t *                   me_ptr,
                                                  volume_ctrl_multichannel_gain_v2_t *vol_payload_ptr)
{

   uint32_t *temp_cfg_ptr = NULL;
   uint32_t *gain_ref_ptr = NULL;
   int8_t *  data_ptr     = (int8_t *)vol_payload_ptr;
   uint32_t  offset       = sizeof(volume_ctrl_multichannel_gain_v2_t);

   // loop through all the configs
   for (uint32_t cnfg_cntr = 0; cnfg_cntr < vol_payload_ptr->num_config; cnfg_cntr++)
   {
      uint32_t ch_mask_arr_index  = 0;
      uint32_t channel_group_mask = 0;
      uint32_t ch_mask_list_size  = 0;

      // Increment data ptr by calculated offset to point at next payload's group_mask
      data_ptr += offset;
      volume_ctrl_channels_gain_config_v2_t *temp_multich_gain_cfg = (volume_ctrl_channels_gain_config_v2_t *)data_ptr;
      channel_group_mask                                           = temp_multich_gain_cfg->channel_type_group_mask;
      temp_cfg_ptr      = (uint32_t *)(data_ptr + CAPI_CMN_INT32_SIZE_IN_BYTES); // points to mask array
      ch_mask_list_size = capi_cmn_count_set_bits(channel_group_mask);
      gain_ref_ptr      = temp_cfg_ptr + ch_mask_list_size; // points to gain payload
#ifdef SOFT_VOL_DEBUG
      SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Gain for config #%lu is %#lx", cnfg_cntr + 1, *gain_ref_ptr);
#endif
      // setting the gain data channel wise
      for (uint32_t group_no = 0; group_no < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; group_no++)
      {
         // check if a group is configured. If yes, update ch_index_mask_to_log_ptr
         if (CAPI_CMN_IS_BIT_SET_AT_POS_IN_32B_VAL(channel_group_mask, group_no))
         {
            uint32_t start_ch_map = group_no * CAPI_CMN_CHANNELS_PER_MASK;
            uint32_t end_ch_map   = (group_no + 1) * CAPI_CMN_CHANNELS_PER_MASK;
            uint32_t j            = 0;
            if(0 == group_no)
            {
               start_ch_map = 1; //ignoring reserved bit 0
               j = 1;
            }
            if(PCM_MAX_CHANNEL_MAP_V2 < end_ch_map)
            {
               end_ch_map = PCM_MAX_CHANNEL_MAP_V2 + 1;
            }
            for (uint32_t i = start_ch_map; i < end_ch_map; i++, j++)
            {
               uint32_t ch_mask = (1 << j);
               if (ch_mask & temp_cfg_ptr[ch_mask_arr_index]) // check if this channel is set anywhere in this group
               {
                  me_ptr->soft_vol_lib.channelGain[i] = *gain_ref_ptr; // copy the respective payload
                  uint32 gainQ28 =
                     capi_soft_vol_calc_gain_q28(me_ptr->soft_vol_lib.masterGain, me_ptr->soft_vol_lib.channelGain[i]);
                  me_ptr->SoftVolumeControlsLib.SetVolume(gainQ28, me_ptr->soft_vol_lib.pPerChannelData[i]);
               }
            }
#ifdef SOFT_VOL_DEBUG
      SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
    		      "Set volume ctrl: config: #%lu, channel mask: %#lx, group no: %lu,"
    		      "includes channels from %lu to %lu. Gain: %#lx",
				  cnfg_cntr + 1,
				  temp_cfg_ptr[ch_mask_arr_index],
				  group_no,
				  start_ch_map,
				  end_ch_map - 1,
				  *gain_ref_ptr);
#endif
            ch_mask_arr_index++;
         }
      }
      offset = sizeof(volume_ctrl_channels_gain_config_v2_t) + (ch_mask_arr_index << 2); //sizeof(uint32_t)
   }
   return CAPI_EOK;
}

capi_err_t capi_soft_vol_set_multichannel_mute_v2(capi_soft_vol_t *                   me_ptr,
                                                  volume_ctrl_multichannel_mute_v2_t *mute_payload_ptr)
{
   uint32_t *temp_cfg_ptr = NULL;
   uint32_t *mute_ref_ptr = NULL;
   int8_t *  data_ptr     = (int8_t *)mute_payload_ptr;
   uint32_t  offset       = sizeof(volume_ctrl_multichannel_mute_v2_t);

   // loop through all the configs
   for (uint32_t cnfg_cntr = 0; cnfg_cntr < mute_payload_ptr->num_config; cnfg_cntr++)
   {
      uint32_t ch_mask_arr_index  = 0;
      uint32_t channel_group_mask = 0;
      uint32_t ch_mask_list_size  = 0;

      // Increment data ptr by calculated offset to point at next payload's group_mask
      data_ptr += offset;
      volume_ctrl_channels_mute_config_v2_t *temp_multich_mute_cfg = (volume_ctrl_channels_mute_config_v2_t *)data_ptr;
      channel_group_mask                                           = temp_multich_mute_cfg->channel_type_group_mask;

      temp_cfg_ptr      = (uint32_t *)(data_ptr + CAPI_CMN_INT32_SIZE_IN_BYTES); // points to the mask array
      ch_mask_list_size = capi_cmn_count_set_bits(channel_group_mask);
      mute_ref_ptr      = temp_cfg_ptr + ch_mask_list_size; // points to the mute payload

      for (uint32_t group_no = 0; group_no < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; group_no++)
      {
         // check if a group is configured.
         if (CAPI_CMN_IS_BIT_SET_AT_POS_IN_32B_VAL(channel_group_mask, group_no))
         {
             uint32_t start_ch_map = group_no * CAPI_CMN_CHANNELS_PER_MASK;
             uint32_t end_ch_map   = (group_no + 1) * CAPI_CMN_CHANNELS_PER_MASK;
             uint32_t j            = 0;
             if(0 == group_no)
             {
                start_ch_map = 1; //ignoring reserved bit 0
                j = 1;
             }
             if(PCM_MAX_CHANNEL_MAP_V2 < end_ch_map)
             {
                end_ch_map = PCM_MAX_CHANNEL_MAP_V2 + 1;
             }
            // iterate over the set of channel maps for this group
            for (uint32_t i = start_ch_map; i < end_ch_map; i++, j++)
            {
               uint32_t ch_mask = (1 << j);
               if (ch_mask & temp_cfg_ptr[ch_mask_arr_index]) // check if this channel is set anywhere in this group
               {
                  if (0 == *mute_ref_ptr)
                  {
                     me_ptr->SoftVolumeControlsLib.Unmute(me_ptr->soft_vol_lib.pPerChannelData[i]);
                  }
                  else
                  {
                     me_ptr->SoftVolumeControlsLib.Mute(me_ptr->soft_vol_lib.pPerChannelData[i]);
                  }

#ifdef SOFT_VOL_DEBUG
                  SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Set volume ctrl: channel: %hu Mute: 0x%lx", i, *mute_ref_ptr);
#endif
               }
            }
#ifdef SOFT_VOL_DEBUG
      SOFT_VOL_MSG(me_ptr->miid, DBG_HIGH_PRIO,
    		      "Set volume ctrl: config: #%lu, channel mask: %#lx, group no: %lu,"
    		      "includes channels from %lu to %lu. Gain: %#lx",
				  cnfg_cntr + 1,
				  temp_cfg_ptr[ch_mask_arr_index],
				  group_no,
				  start_ch_map,
				  end_ch_map - 1,
				  *mute_ref_ptr);
#endif
            ch_mask_arr_index++;
         }
      }
      offset = sizeof(volume_ctrl_channels_mute_config_v2_t) + (ch_mask_arr_index << 2); //sizeof(uint32_t)
   }
   return CAPI_EOK;
}

capi_err_t capi_soft_vol_get_multichannel_mute_v2_payload_size(capi_soft_vol_t *me_ptr,
                                                               uint32_t *       mute_chmask_list,
                                                               uint32_t *       req_size_ptr)
{
   uint32_t mute               = 0;
   uint32_t channel_group_mask = 0;
   uint16_t channel_type       = 0;

   for (int i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
   {
      channel_type = me_ptr->soft_vol_lib.channelMapping[i];
      mute         = (me_ptr->SoftVolumeControlsLib.IsMuted(
                me_ptr->soft_vol_lib.pPerChannelData[me_ptr->soft_vol_lib.channelMapping[i]]))
                        ? 1
                        : 0;
      if (mute == 1)
      {
         // get the group index in which this particular chan type should be stored
         uint32_t group_index = CAPI_CMN_DIVIDE_WITH_32(channel_type);
         // get the bit position in which this particular chan type should be stored
         // in mute_chmask_list[group_index]
         uint32_t ch_bit_position = CAPI_CMN_MOD_WITH_32(channel_type);
         // add this channel type to mask list
         mute_chmask_list[group_index] |= CAPI_CMN_CONVERT_TO_32B_MASK(ch_bit_position);
      }
   }
   //check for the set groups, then update ch group mask and required size for group mask
   for (uint32_t group_index = 0; group_index < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; group_index++)
   {
      if (mute_chmask_list[group_index])
      {
         channel_group_mask |= CAPI_CMN_CONVERT_TO_32B_MASK(group_index);
         *req_size_ptr += CAPI_CMN_INT32_SIZE_IN_BYTES;
      }
   }
   if (channel_group_mask) // if there is a valid group mask (>0), update the required size
   {
      *req_size_ptr += (sizeof(volume_ctrl_multichannel_mute_v2_t) + sizeof(volume_ctrl_channels_mute_config_v2_t));
   }
   else
   {
      SOFT_VOL_MSG(me_ptr->miid, DBG_ERROR_PRIO, "No mute config present! Returning.");
      return CAPI_EBADPARAM;
   }
   return CAPI_EOK;
}

void capi_soft_vol_get_multichannel_mute_v2(capi_soft_vol_t *me_ptr,
                                            int8_t *         get_payload_ptr,
                                            uint32_t *       mute_chamask_list_ptr)
{
   uint32_t *temp_payload_ptr   = NULL;
   uint32_t  mute               = 1;
   uint32_t  channel_group_mask = 0;
   temp_payload_ptr             = (uint32_t *)get_payload_ptr;

   // group mask creation
   for (uint32_t group_index = 0; group_index < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; group_index++)
   {
      if (mute_chamask_list_ptr[group_index])
      {
         channel_group_mask |= CAPI_CMN_CONVERT_TO_32B_MASK(group_index);
      }
   }
   if (channel_group_mask)
   {
      uint32_t chmask_arr_index = 0;
      *temp_payload_ptr         = 1; // num_config is 1 (only mute config)
      temp_payload_ptr++;
      *temp_payload_ptr = channel_group_mask;
      temp_payload_ptr++;
      for (uint32_t group_index = 0; group_index < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; group_index++)
      {
         if (mute_chamask_list_ptr[group_index])
         {
            temp_payload_ptr[chmask_arr_index] = mute_chamask_list_ptr[group_index];
            chmask_arr_index++;
         }
      }
      temp_payload_ptr += chmask_arr_index; // pointing to the mute payload
      *temp_payload_ptr = mute;             // mute
   }
}

uint32_t capi_soft_vol_get_multichannel_gain_v2_payload_size(capi_soft_vol_t *                me_ptr,
                                                             capi_soft_vol_multich_gain_info *gain_list,
                                                             uint32_t *                       req_size_ptr)
{
   uint32_t position         = 0;
   uint32_t num_unique_gains = 0; // num_unique_gains is equivalent to num_configs
   uint32_t gain             = 0;
   uint16_t channel_type     = 0;

   *req_size_ptr += sizeof(volume_ctrl_multichannel_gain_v2_t);

   for (uint32_t i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
   {
      gain         = me_ptr->soft_vol_lib.channelGain[me_ptr->soft_vol_lib.channelMapping[i]];
      channel_type = me_ptr->soft_vol_lib.channelMapping[i];

      capi_soft_vol_update_mult_ch_gain_info(me_ptr, gain_list, gain, &num_unique_gains, &position, channel_type, req_size_ptr);
   }
#ifdef SOFT_VOL_DEBUG
      SOFT_VOL_MSG(me_ptr->miid, DBG_LOW_PRIO, "Get multichannel volume, num_unique_gains(num_cfg): %lu"
    		                         ". Required size of payload: %lu",
									 num_unique_gains,
									 *req_size_ptr);
#endif
   return num_unique_gains;
}

/* checks if the gain is already present in the previous configs added and updates the number of unique gains
 * (equivalent to number of configs), updates the corresponding channel mask info*/
void capi_soft_vol_update_mult_ch_gain_info(capi_soft_vol_t *                me_ptr,
		                                    capi_soft_vol_multich_gain_info *gain_list,
                                            uint32_t                         gain,
                                            uint32_t *                       num_unique_gains_ptr,
                                            uint32_t *                       position_ptr,
                                            uint32_t                         channel_type,
                                            uint32_t *                       req_size_ptr)
{
   bool_t unique = TRUE;
   // loop through all the unique gains(num_cfg till now) in gain_list, check if gain is not unique,
   // then update the gain_list without incrementing position.
   for (uint32_t i = 0; i < *num_unique_gains_ptr; i++)
   {
      if (gain == gain_list[i].gain)
      {
         unique        = FALSE;
         *position_ptr = i;
         capi_soft_vol_update_gain_ch_mask_list(me_ptr, &gain_list[0], position_ptr, channel_type, req_size_ptr);
         break;
      }
   }
   //if gain is unique, then add the gain to gain list, increment the req size with per cfg size and
   //update the gain_list with incrementing position.
   if (unique)
   {
      gain_list[*num_unique_gains_ptr].gain = gain;
      *req_size_ptr += MULT_CH_GAIN_BASE_PYLD_SIZE; // group_mask + gain = 8B
      *position_ptr = *num_unique_gains_ptr;
      capi_soft_vol_update_gain_ch_mask_list(me_ptr, &gain_list[0], position_ptr, channel_type, req_size_ptr);
      *num_unique_gains_ptr += 1;
   }
}
void capi_soft_vol_update_gain_ch_mask_list(capi_soft_vol_t *                me_ptr,
		                                    capi_soft_vol_multich_gain_info *gain_list,
                                            uint32_t *                       position_ptr,
                                            uint32_t                         channel_type,
                                            uint32_t *                       req_size_ptr)
{
   // get the group index in which this particular ch should be stored
   uint32_t group_index = CAPI_CMN_DIVIDE_WITH_32(channel_type);
   if (!CAPI_CMN_IS_BIT_SET_AT_POS_IN_32B_VAL(gain_list[*position_ptr].group_ch_mask, group_index))
   {
      gain_list[*position_ptr].group_ch_mask |= CAPI_CMN_CONVERT_TO_32B_MASK(group_index);
      *req_size_ptr += CAPI_CMN_INT32_SIZE_IN_BYTES; // size of channel mask list size would be increased by 1
#ifdef SOFT_VOL_DEBUG
      SOFT_VOL_MSG(me_ptr->miid,
    		  DBG_LOW_PRIO,
                   "gain_list[%lu].group_ch_mask = %#lx",
                   *position_ptr,
                   gain_list[*position_ptr].group_ch_mask);
#endif
   }
   // get the bit position in which this particular channel should be stored
   // in channel mask list
   uint32_t ch_bit_position = CAPI_CMN_MOD_WITH_32(channel_type);
   // update this channel_type in the list
   gain_list[*position_ptr].multch_gain_mask_list[group_index] |= CAPI_CMN_CONVERT_TO_32B_MASK(ch_bit_position);
#ifdef SOFT_VOL_DEBUG
   SOFT_VOL_MSG(me_ptr->miid,
		        DBG_LOW_PRIO,
                "gain_list[%lu].multch_gain_mask_list[%lu] = %#lx",
                *position_ptr,
                group_index,
                gain_list[*position_ptr].multch_gain_mask_list[group_index]);
#endif
}

capi_err_t capi_soft_vol_get_multichannel_gain_v2(capi_soft_vol_t *                me_ptr,
                                                  int8_t *                         get_payload_ptr,
                                                  capi_soft_vol_multich_gain_info *gain_list,
                                                  uint32_t                         payload_size,
                                                  uint32_t                         num_unique_gains)
{
   uint32_t dest_size = 0;
   dest_size += sizeof(volume_ctrl_multichannel_gain_v2_t);
   uint32 *base_payload_ptr = (uint32_t *)get_payload_ptr;
   *base_payload_ptr        = num_unique_gains; // copying the num_config

   for (uint32_t i = 0; i < num_unique_gains; i++)
   {
      base_payload_ptr++;
      *base_payload_ptr = gain_list[i].group_ch_mask;
      dest_size += CAPI_CMN_INT32_SIZE_IN_BYTES; // group mask
      base_payload_ptr++;
      for (uint32_t j = 0; j < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; j++)
      {
         if (gain_list[i].multch_gain_mask_list[j])
         {
            *base_payload_ptr = gain_list[i].multch_gain_mask_list[j];
            dest_size += CAPI_CMN_INT32_SIZE_IN_BYTES;
            base_payload_ptr++;
         }
      }
      *base_payload_ptr = gain_list[i].gain;
      dest_size += CAPI_CMN_INT32_SIZE_IN_BYTES; // gain
   }
   if (payload_size != dest_size)
   {
      SOFT_VOL_MSG(me_ptr->miid,
                   DBG_ERROR_PRIO,
                   "Get multichannel volume, Bad param size %lu. Required param_size %lu",
                   dest_size,
                   payload_size);
      return CAPI_EFAILED;
   }
   return CAPI_EOK;
}
