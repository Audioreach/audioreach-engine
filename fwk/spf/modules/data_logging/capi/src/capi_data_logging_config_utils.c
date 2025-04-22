/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_data_logging_utils.c
 */

/*==============================================================================
   Include files and Macro definitions
==============================================================================*/
#include "capi_data_logging_i.h"
/*==============================================================================
   Function definitions
==============================================================================*/

/*utility to calculate mask array size dependent on group mask*/
uint32_t calculate_size_for_ch_mask_array(uint32_t num)
{
   return (capi_cmn_count_set_bits(num) * sizeof(uint32_t));
}

/*return the number of channels to log.*/
uint32_t get_number_of_channels_to_log(capi_data_logging_t *me_ptr)
{
   // default if number of channels are invalid (raw-compressed or media format not yet set)
   uint32_t enabled_num_chs = 1;

   if (CAPI_DATA_FORMAT_INVALID_VAL != me_ptr->media_format.format.num_channels)
   {
      uint32_t *enabled_ch_mask_ptr = &me_ptr->nlpi_me_ptr->enabled_channel_mask_array[0];
      enabled_num_chs               = 0;
      for (uint32_t i = 0; i < CAPI_CMN_MAX_CHANNEL_INDEX_GROUPS; i++)
      {
         enabled_num_chs += capi_cmn_count_set_bits(enabled_ch_mask_ptr[i]);
      }
   }
   return enabled_num_chs;
}

/*utility to check if payload is valid and calculate payload size*/
capi_err_t capi_data_logging_check_and_calculate_mask_payload_size(capi_data_logging_t *me_ptr,
                                                                   uint32_t *           array_size_ptr,
                                                                   uint32_t             max_valid_channel_group,
                                                                   uint32_t             channel_group_mask)
{
   DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid, DBG_HIGH_PRIO, "Channel group mask is :%lu ", channel_group_mask);
   // check for group mask payload if only desired bits are set or not
   if (0 == (channel_group_mask >> max_valid_channel_group))
   {
      *array_size_ptr = calculate_size_for_ch_mask_array(channel_group_mask);
   }
   else
   {
      DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "Invalid configuration of channel group mask: %lu. More than maximum valid channel groups are "
                       "set, maximum valid channel groups: %lu.",
                       channel_group_mask,
                       max_valid_channel_group);
      return CAPI_EBADPARAM;
   }
   return CAPI_EOK;
}
capi_err_t capi_data_logging_validate_ch_mask_payload_size(capi_data_logging_t *              me_ptr,
                                                           data_logging_select_channels_v2_t *chan_mask_cfg_ptr,
                                                           uint32_t                           param_size,
                                                           uint32_t *                         required_size_ptr)
{
   capi_err_t capi_result           = CAPI_EOK;
   uint32_t   ch_type_group_mask    = 0;
   uint32_t   chan_index_array_size = 0;
   uint32_t   chan_type_array_size  = 0;
   uint32_t   ch_index_group_mask   = 0;
   uint32_t   max_valid_channel_index_group = CAPI_CMN_MAX_CHANNEL_INDEX_GROUPS;
   uint32_t   max_valid_channel_type_group  = CAPI_CMN_MAX_CHANNEL_MAP_GROUPS;
   uint32_t   *base_payload_ptr = (uint32_t *)chan_mask_cfg_ptr;
   *required_size_ptr           = sizeof(data_logging_select_channels_v2_t); // 4Bytes

   // configuration payload
   // _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
   //| mode | channel_index_group_mask | dynamic channel_index_mask_list[0] | channel_type_group_mask | dynamic channel_type_mask_list[0] |
   //|_ _ _ | _ _ _ _ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ |
   //
   //
   if ((DATA_LOGGING_SELECT_CHANNEL_TYPE_MASK == chan_mask_cfg_ptr->mode) ||
		   (DATA_LOGGING_SELECT_CHANNEL_INDEX_MASK == chan_mask_cfg_ptr->mode))
   {
      DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid, DBG_HIGH_PRIO, "Mode selected is :%d ", chan_mask_cfg_ptr->mode);
      base_payload_ptr += (*required_size_ptr / 4);
      ch_index_group_mask = *base_payload_ptr; //incrementing base_payload_ptr by required_size_ptr will give ch index group mask
      //calculate index mask payload size based on index group mask
      capi_result = capi_data_logging_check_and_calculate_mask_payload_size(me_ptr,
                                                                            &chan_index_array_size,
                                                                            max_valid_channel_index_group,
                                                                            ch_index_group_mask);
      if (!capi_result)
      {
         *required_size_ptr +=
    	          		 sizeof(uint32_t) + chan_index_array_size;// size of channel_index_group_mask + channel index array size
      }
      else
      {
         return CAPI_EBADPARAM;
      }

      //check if param size is sufficient to hold channel_type_group_mask
      //and whether channel type mask payload is present in the configuration.
      //if yes, then increment the base payload pointer to point to channel type group mask
      if (param_size <= *required_size_ptr)
      {
         DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                          DBG_ERROR_PRIO,
                          "Failed while validating "
                          "required size for configuration in mode Channel Type mask. Param size not sufficient to "
                          "hold channel type mask payload. Required size calculated "
                          "till channel_index_mask_list: %lu, Param size recieved: %lu",
                          *required_size_ptr,
                          param_size);
         return CAPI_ENEEDMORE;
      }

      //increment base_payload_ptr with the channel_index_group_mask plus size of ch index array to point to ch type group mask
      base_payload_ptr += (1 + (chan_index_array_size / 4)); // it is 4 byte word
      ch_type_group_mask = *base_payload_ptr; // first element of channel_type_config

      //calculate type mask payload size based on type group mask
      capi_result = capi_data_logging_check_and_calculate_mask_payload_size(me_ptr,
                                                                            &chan_type_array_size,
                                                                            max_valid_channel_type_group,
                                                                            ch_type_group_mask);
      if (!capi_result)
      {
         *required_size_ptr +=
            sizeof(uint32_t) + chan_type_array_size; // size of channel_type_group_mask + channel type array size
         if (*required_size_ptr != param_size)
         {
            DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                             DBG_ERROR_PRIO,
                             "Failed while validating "
                             "required size for config. Required size: %lu, Param size received: %lu",
                             *required_size_ptr,
                             param_size);
            return CAPI_EBADPARAM;
         }
      }
      else
      {
         return CAPI_EBADPARAM;
      }
   }
   else
   {
      DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "Invalid configuration of ch_mask mode %lu",
                       chan_mask_cfg_ptr->mode);
      return CAPI_EBADPARAM;
   }
   me_ptr->nlpi_me_ptr->channel_logging_cfg.ch_index_mask_list_size_in_bytes = (uint16_t)chan_index_array_size;
   me_ptr->nlpi_me_ptr->channel_logging_cfg.ch_type_mask_list_size_in_bytes  = (uint16_t)chan_type_array_size;
   DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,DBG_HIGH_PRIO, "Received chan_index_array_size = %lu",
		   chan_index_array_size);
   DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,DBG_HIGH_PRIO, "Received chan_type_array_size = %lu",
		   chan_type_array_size);
   return CAPI_EOK;
}

// utility function to return the channel index mask for the channels which are selected for logging.
capi_err_t capi_data_logging_get_channel_index_mask_to_log_v2(capi_data_logging_t *              me_ptr,
                                                              data_logging_select_channels_v2_t *ch_mask_cfg_ptr)
{
   uint32_t  ch_index_group_mask   = 0;
   uint32_t  ch_type_group_mask    = 0;
   uint32_t  num_channels          = me_ptr->media_format.format.num_channels;
   uint32_t  max_ch_index_grp      = (num_channels + (CAPI_CMN_CHANNELS_PER_MASK - 1)) / CAPI_CMN_CHANNELS_PER_MASK;
   uint32_t *ch_type_mask_arr      = NULL;
   uint32_t *ch_index_mask_arr     = NULL;
   uint32_t *base_payload_ptr      = (uint32_t *)ch_mask_cfg_ptr;
   uint32_t *ch_index_mask_to_log_ptr  = &me_ptr->nlpi_me_ptr->channel_logging_cfg.channel_mask_index_to_log_arr[0];

   base_payload_ptr += sizeof(data_logging_select_channels_v2_t) / 4; //elements are 4 byte word

   for (uint32_t i = 0; i < CAPI_CMN_MAX_CHANNEL_INDEX_GROUPS; i++)
   {
      ch_index_mask_to_log_ptr[i] = DATA_LOGGING_ALL_CH_LOGGING_MASK;
   }

   if(CAPI_MAX_CHANNELS_V2 < num_channels) //added this to resolve kw issues
   {
      DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,DBG_ERROR_PRIO,"unsupported number of channels");
      return CAPI_EFAILED;
   }
   if (CAPI_DEINTERLEAVED_RAW_COMPRESSED == me_ptr->media_format.header.format_header.data_format)
   {
      // For raw compressed, log all channels, ignoring configuration.
      // iterate through maximum valid group
      for (uint32_t index_group = 0; index_group < max_ch_index_grp; index_group++)
      {
         // Calculate the number of bits to store in ch_index_mask_to_log_ptr[group] depending on num_channels
         // if group is not the last group: OR if num_bits is 0: set all the bits in bitmask.
         // if it is the last group: set only required bits as per num_channels and rest of the bits will be 0
         uint32_t num_bits = (index_group < (max_ch_index_grp - 1)) ? 0 : CAPI_CMN_MOD_WITH_32(num_channels);
         // Calculate the bitmask for the group
         uint32_t bitmask = (0 == num_bits) ? CAPI_CMN_SET_MASK_32B : (CAPI_CMN_CONVERT_TO_32B_MASK(num_bits) - 1);
         me_ptr->nlpi_me_ptr->enabled_channel_mask_array[index_group] = bitmask;
      }
   }
   else if (CAPI_DATA_FORMAT_INVALID_VAL != num_channels)
   {
      // Configuration is based on channel type mask.
      if (DATA_LOGGING_SELECT_CHANNEL_TYPE_MASK == ch_mask_cfg_ptr->mode)
      {
    	 uint32_t  ch_type_arr_idx       = 0;
         base_payload_ptr += (1 + ((me_ptr->nlpi_me_ptr->channel_logging_cfg.ch_index_mask_list_size_in_bytes) / 4)); //increment by position of ch index group mask + ch index mask array
         ch_type_group_mask = *base_payload_ptr; //first element of channel_type_config
         ch_type_mask_arr  = base_payload_ptr + 1; //incrementing offset by 1 gives the position of channel type config array
#ifdef DATA_LOGGING_DBG
         DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,DBG_HIGH_PRIO, "Received ch_index_mask_list_size_in_bytes = %lu",
        		 me_ptr->nlpi_me_ptr->channel_logging_cfg.ch_index_mask_list_size_in_bytes);
         DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,DBG_HIGH_PRIO, "Received ch_type_group_mask = %lu",
        		 ch_type_group_mask);
#endif
         // initialize the channel index mask array to zero.
         for (uint32_t i = 0; i < CAPI_CMN_MAX_CHANNEL_INDEX_GROUPS; i++)
         {
            ch_index_mask_to_log_ptr[i] = 0;
         }

         // Iterate over maximum valid channel map group
         // 1. check if a channel group is configured.
         // 2. if yes, iterate over all channels in input media fmt.
         // 3. check if any channel type falls under this particular group
         // 4. find ideal bit position to store in ch_index_mask_to_log_ptr, of that channel type in channel_type_mask_list
         // 5. check if that bit is set in channel_type_mask_list
         // 6. if yes, then store in ch_index_mask_to_log_ptr array.
         for (uint32_t group = 0; group < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; group++)
         {
            // check if a group is configured
            // if yes, update ch_index_mask_to_log_ptr
            if (CAPI_CMN_IS_BIT_SET_AT_POS_IN_32B_VAL(ch_type_group_mask, group))
            {
               // Iterate through each channel and check if a channel type is configured
               for (uint32_t chan_no = 0; chan_no < num_channels; chan_no++)
               {
                  // check if a channel type falls under this group
                  if (group == (me_ptr->media_format.format.channel_type[chan_no] / CAPI_CMN_CHANNELS_PER_MASK))
                  {
                     //find the ideal bit position of the channel_type in channel type config array
                     uint32_t bit_position = (uint32_t)CAPI_CMN_MOD_WITH_32(me_ptr->media_format.format.channel_type[chan_no]);
                     if (group == 0 && bit_position == 0)
                     {
                        // Skip the reserved bit 0 in group 0 channel_type_mask_list
                        continue;
                     }
                     // check if a channel-type in imf is configured
                     if (CAPI_CMN_IS_BIT_SET_AT_POS_IN_32B_VAL(ch_type_mask_arr[ch_type_arr_idx], bit_position))
                     {
                        //get the group index in which this particular ch should be stored
                        uint32_t group_mask_index   = (chan_no / CAPI_CMN_CHANNELS_PER_MASK);
                        //get the bit position in which this particular ch should be stored
                        //in ch_index_mask_to_log_ptr[group_mask_index]
                        uint32_t index_bit_position = CAPI_CMN_MOD_WITH_32(chan_no);
                        //update this chan_no to log
                        ch_index_mask_to_log_ptr[group_mask_index] |= CAPI_CMN_CONVERT_TO_32B_MASK(index_bit_position);
                     }
                  }
               }
               ch_type_arr_idx++;
            }
         }
		 
		 for (uint32_t group = 0; group < CAPI_CMN_MAX_CHANNEL_INDEX_GROUPS; group++)
		 {
            DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,DBG_HIGH_PRIO, "Received ch_index_mask_to_log_ptr[%lu] = %lu",
            		   group, ch_index_mask_to_log_ptr[group]);
		 }
      }

      // Configuration is based on channel index mask.
      else if (DATA_LOGGING_SELECT_CHANNEL_INDEX_MASK == ch_mask_cfg_ptr->mode)
      {
    	 uint32_t  ch_index_mask_arr_idx = 0;
         ch_index_group_mask = *base_payload_ptr;
		 base_payload_ptr++; // incrementing by 1 to point to ch_index mask array
         ch_index_mask_arr   = base_payload_ptr; 

         // initialize the channel index mask to zero.
         for (uint32_t i = 0; i < CAPI_CMN_MAX_CHANNEL_INDEX_GROUPS; i++)
         {
            ch_index_mask_to_log_ptr[i] = 0;
         }
         // iterate through maximum valid group
         for (uint32_t group = 0; group < max_ch_index_grp; group++)
         {
            // check if a group is configured
            // if yes, update ch_index_mask_to_log_ptr
            if (CAPI_CMN_IS_BIT_SET_AT_POS_IN_32B_VAL(ch_index_group_mask, group))
            {
               // Calculate the number of bits to store in ch_index_mask_to_log_ptr[group] depending on num_channels
               // if group is not the last group OR if num_bits is 0: set all the bits in bitmask.
               // if it is the last group: set only required bits as per num_channels and rest of the bits will be 0
               uint32_t num_bits = (group < (max_ch_index_grp - 1)) ? 0 : CAPI_CMN_MOD_WITH_32(num_channels);
               // Calculate the bitmask for the group
               uint32_t bitmask = (0 == num_bits) ? CAPI_CMN_SET_MASK_32B : (CAPI_CMN_CONVERT_TO_32B_MASK(num_bits) - 1);

               ch_index_mask_to_log_ptr[group] = ch_index_mask_arr[ch_index_mask_arr_idx] & bitmask;
               ch_index_mask_arr_idx++;

               DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,DBG_HIGH_PRIO, "Received ch_index_mask_to_log_ptr[%lu] = %lu",
            		   group, ch_index_mask_to_log_ptr[group]);

            }
         }
      }
      else
      {
         // Invalid configuration mode.
         for (uint32_t i = 0; i < CAPI_CMN_MAX_CHANNEL_INDEX_GROUPS; i++)
         {
            ch_index_mask_to_log_ptr[i] = 0;
         }

         DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                          DBG_ERROR_PRIO,
                          "Invalid configuration of ch_mask mode %lu",
                          ch_mask_cfg_ptr->mode);
         return CAPI_EBADPARAM;
      }
   }
   return CAPI_EOK;
}

// utility function to cache the recieved payload
capi_err_t capi_data_logging_cache_selective_channel_payload(capi_data_logging_t *me_ptr,
                                                             int8_t *const        payload_ptr,
                                                             uint32_t             payload_actual_data_len,
                                                             uint32_t             required_cache_size)
{
   uint32_t *temp_ptr = NULL;
   // allocate memory required if size is different
   if (required_cache_size != me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_size)
   {
      if (NULL != me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_ptr)
      {
         posal_memory_aligned_free(me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_ptr);
         me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_ptr  = NULL;
         me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_size = 0;
      }

      temp_ptr = (uint32_t *)posal_memory_aligned_malloc(required_cache_size,
                                                         LOG_BUFFER_ALIGNMENT,
                                                         (POSAL_HEAP_ID)me_ptr->nlpi_me_ptr->nlpi_heap_id.heap_id);
      if (NULL == temp_ptr)
      {
         DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                          DBG_ERROR_PRIO,
                          "No memory to cache the selective channel cfg of %lu bytes",
                          required_cache_size);
         return CAPI_ENOMEMORY;
      }
      me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_ptr =
         (data_logging_select_channels_v2_t *)temp_ptr;
      me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_size = required_cache_size;
   }

   // copy the configuration
   memscpy(me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_ptr,
           me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_size,
           payload_ptr,
           payload_actual_data_len);
   DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                    DBG_HIGH_PRIO,
                    "Cached selective channel logging cfg of size %lu bytes",
                    me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_size);
   return CAPI_EOK;
}

// utility function to update the enabled channels to log
capi_err_t capi_data_logging_update_enabled_channels_to_log(
   capi_data_logging_t *                           me_ptr,
   capi_data_logging_selective_channel_cfg_state_t selective_ch_logging_cfg_state)
{
   uint32_t   num_channels                   = me_ptr->media_format.format.num_channels;
   uint32_t   max_ch_index_grp               = (num_channels + (CAPI_CMN_CHANNELS_PER_MASK - 1)) / CAPI_CMN_CHANNELS_PER_MASK;
   uint32_t * enabled_channel_index_mask_ptr = NULL;
   uint32_t * ch_index_mask_to_log_ptr       = NULL;
   capi_err_t capi_result                    = CAPI_EOK;

   //if V1(old) config is configured
   if (SELECTVE_CH_LOGGING_CFG_V1 == selective_ch_logging_cfg_state)
   {
       if(MAX_CHANNEL_V1_CFG < num_channels)
      {
         DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                    DBG_HIGH_PRIO,
                    "Unsupported number of channels in input media format to configure with V1 selective channel API." 
                    "Max supported channel is %lu. Returning",
                    MAX_CHANNEL_V1_CFG);
         return CAPI_EFAILED;
      }
      me_ptr->nlpi_me_ptr->enabled_channel_mask_array[0] =
         capi_data_logging_get_channel_index_mask_to_log(me_ptr, &me_ptr->nlpi_me_ptr->selective_ch_logging_cfg);
   }
   //if V2(new) config is configured
   else if (SELECTVE_CH_LOGGING_CFG_V2 == selective_ch_logging_cfg_state)
   {

      capi_result = capi_data_logging_get_channel_index_mask_to_log_v2(me_ptr,
                                                                       me_ptr->nlpi_me_ptr->channel_logging_cfg
                                                                          .cache_channel_logging_cfg_ptr);
      if (CAPI_FAILED(capi_result))
      {
         return capi_result;
      }
      enabled_channel_index_mask_ptr = &me_ptr->nlpi_me_ptr->enabled_channel_mask_array[0];
      ch_index_mask_to_log_ptr       = &me_ptr->nlpi_me_ptr->channel_logging_cfg.channel_mask_index_to_log_arr[0];

      for (uint32_t i = 0; i < CAPI_CMN_MAX_CHANNEL_INDEX_GROUPS; i++)
      {
         if (ch_index_mask_to_log_ptr[i] != enabled_channel_index_mask_ptr[i])
         {
            // Array elements are not equal
            enabled_channel_index_mask_ptr[i] = ch_index_mask_to_log_ptr[i];
            DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                             DBG_HIGH_PRIO,
                             "Enabled channel mask for group %lu is %lu",
                             i,
                             enabled_channel_index_mask_ptr[i]);
         }
      }
   }
   //if selective channel logging configuration hasn't been configured then enable all the channels for logging
   else
   {
      // iterate through maximum valid group
      for (uint32_t index_group = 0; index_group < max_ch_index_grp; index_group++)
      {
         // Calculate the number of bits to store in ch_index_mask_to_log_ptr[group] depending on num_channels
         // if group is not the last group: OR if num_bits is 0: set all the bits in bitmask.
         // if it is the last group: set only required bits as per num_channels and rest of the bits will be 0
         uint32_t num_bits = (index_group < (max_ch_index_grp - 1)) ? 0 : CAPI_CMN_MOD_WITH_32(num_channels);
         // Calculate the bitmask for the group
         uint32_t bitmask = (0 == num_bits) ? CAPI_CMN_SET_MASK_32B : (CAPI_CMN_CONVERT_TO_32B_MASK(num_bits) - 1);
         me_ptr->nlpi_me_ptr->enabled_channel_mask_array[index_group] = bitmask;
      }
   }
   return CAPI_EOK;
}

// utility function to set the PARAM_ID_DATA_LOGGING_SELECT_CHANNELS_V2 param ID
capi_err_t capi_data_logging_set_selective_channels_v2_pid(capi_data_logging_t *me_ptr, capi_buf_t *params_ptr)
{
   capi_err_t result                         = CAPI_EOK;
   uint32_t   required_size                  = 0;
   uint32_t   is_ch_mask_changed             = 0;
   uint32_t * enabled_channel_index_mask_ptr = NULL;
   uint32_t * ch_index_mask_to_log_ptr       = NULL;

   if (params_ptr->actual_data_len < sizeof(data_logging_select_channels_v2_t))
   {
      DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                       DBG_ERROR_PRIO,
                       " Set "
                       "PARAM_ID_DATA_LOGGING_SELECT_CHANNELS_V2 fail: Invalid "
                       "payload size: "
                       "%u",
                       params_ptr->actual_data_len);
      return (CAPI_EBADPARAM);
   }

   data_logging_select_channels_v2_t *ch_mask_cfg_ptr = (data_logging_select_channels_v2_t *)params_ptr->data_ptr;

   // validate received payload size
   result = capi_data_logging_validate_ch_mask_payload_size(me_ptr,
                                                            ch_mask_cfg_ptr,
                                                            params_ptr->actual_data_len,
                                                            &required_size);
   if (CAPI_FAILED(result))
   {
      return result;
   }

   // caching the configuration recieved
   result = capi_data_logging_cache_selective_channel_payload(me_ptr,
                                                              params_ptr->data_ptr,
                                                              params_ptr->actual_data_len,
                                                              required_size);

   if (CAPI_FAILED(result))
   {
      return result;
   }

   /* If the media format is valid and enable channel mask is changed then
    * 1. log the existing data
    * 2. increment the counter
    * 3. realloc the log buffer.*/
   if (CAPI_DATA_FORMAT_INVALID_VAL != me_ptr->media_format.format.num_channels)
   {
      result = capi_data_logging_get_channel_index_mask_to_log_v2(me_ptr, ch_mask_cfg_ptr);
      if (CAPI_FAILED(result))
      {
         return result;
      }

      enabled_channel_index_mask_ptr = &me_ptr->nlpi_me_ptr->enabled_channel_mask_array[0];
      ch_index_mask_to_log_ptr       = &me_ptr->nlpi_me_ptr->channel_logging_cfg.channel_mask_index_to_log_arr[0];

      for (uint32_t i = 0; i < CAPI_CMN_MAX_CHANNEL_INDEX_GROUPS; i++)
      {
         if (ch_index_mask_to_log_ptr[i] != enabled_channel_index_mask_ptr[i])
         {
            // Array elements are not equal
            is_ch_mask_changed                = 1;
            enabled_channel_index_mask_ptr[i] = ch_index_mask_to_log_ptr[i];
            DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                             DBG_HIGH_PRIO,
                             "Enabled channel mask for group %lu is %lu",
                             i,
                             enabled_channel_index_mask_ptr[i]);
         }
      }
      if (is_ch_mask_changed)
      {
         data_logging(me_ptr, me_ptr->nlpi_me_ptr->log_buf_ptr, FALSE);
         incr_log_id(me_ptr);
         DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid, DBG_HIGH_PRIO, "data logging channel mask changed.");
         me_ptr->nlpi_me_ptr->number_of_channels_to_log = get_number_of_channels_to_log(me_ptr);
         result                                         = check_alloc_log_buf(me_ptr);
         if (CAPI_FAILED(result))
         {
            return result;
         }
         // update votes because number of channels to log are changed
         capi_data_logging_raise_kpps_bw_event(me_ptr);
         capi_data_logging_raise_process_state_event(me_ptr);
      }
   }
   DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid, DBG_HIGH_PRIO, "Set Data Logging Select Channels Param id is set");
   me_ptr->nlpi_me_ptr->selective_ch_logging_cfg_state = SELECTVE_CH_LOGGING_CFG_V2;

   return CAPI_EOK;
}

// utility function to get the selective channels V2 param ID.
capi_err_t capi_data_logging_get_selective_channels_v2_pid(capi_data_logging_t *me_ptr, uint32_t  param_id, capi_buf_t *params_ptr)
{
   if (me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_ptr)
   {
      if (params_ptr->max_data_len < me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_size)
      {
         params_ptr->actual_data_len = me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_size;
         DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                          DBG_ERROR_PRIO,
                          " Get PARAM_ID_DATA_LOGGING_SELECT_CHANNELS_V2, size received %ld, size expected %ld",
                          params_ptr->max_data_len,
                          me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_size);
         return (CAPI_EBADPARAM);
      }

      params_ptr->actual_data_len =
         memscpy(params_ptr->data_ptr,
                 params_ptr->max_data_len,
                 me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_ptr,
                 me_ptr->nlpi_me_ptr->channel_logging_cfg.cache_channel_logging_cfg_size);
   }
   else
   {
      DATA_LOGGING_MSG(me_ptr->nlpi_me_ptr->iid,
                       DBG_ERROR_PRIO,
                       "Did not receive any set param for parameter 0x%lx",
                       param_id);
      return (CAPI_EFAILED);
      params_ptr->actual_data_len = 0;
   }
   return CAPI_EOK;
}
