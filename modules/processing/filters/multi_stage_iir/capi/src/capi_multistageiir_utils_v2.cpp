/* ======================================================================== */
/*
@file capi_multistageiir_utils_v2.cpp

   Source file to implement the Audio Post Processor Interface for Multi-Stage
   IIR filters
*/

/* =========================================================================
   * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   * SPDX-License-Identifier: BSD-3-Clause-Clear
  ========================================================================= */

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "capi_multistageiir_utils.h"
/*------------------------------------------------------------------------
 * Static declarations
 * -----------------------------------------------------------------------*/
/*------------------------------------------------------------------------
 * Function declarations
 * -----------------------------------------------------------------------*/
capi_err_t capi_msiir_validate_multichannel_v2_payload(capi_multistageiir_t *me_ptr,
                                                     int8_t *    data_ptr,
                                                     uint32_t    param_id,
                                                     uint32_t    param_size,
                                                     uint32_t *  req_payload_size,
                                                     uint32_t    base_payload_size,
                                                     uint32_t    per_cfg_base_payload_size)
{
   int8_t * temp_cfg_ptr = data_ptr;
   uint32_t capi_result      = CAPI_EOK;
   uint32_t num_cfg          = *((uint32_t *)data_ptr);

#ifdef CAPI_MSIIR_DEBUG_MSG
   MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO, "Received num_config = %lu for PID = 0x%lx", num_cfg, param_id);
#endif

   if (num_cfg < 1 || num_cfg > PCM_MAX_CHANNEL_MAP_V2)
   {
      MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Received incorrect num_config parameter - %lu", num_cfg);
      return CAPI_EBADPARAM;
   }

   capi_result = capi_msiir_validate_per_channel_v2_payload(me_ptr,
                                                          num_cfg,
                                                          temp_cfg_ptr,
                                                          param_size,
                                                          req_payload_size,
                                                          param_id,
                                                          base_payload_size,
                                                          per_cfg_base_payload_size);
   if (CAPI_FAILED(capi_result))
   {
      MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Configuartion SetParam 0x%lx failed.", param_id);
      return capi_result;
   }

   return CAPI_EOK;
}

capi_err_t capi_msiir_validate_per_channel_v2_payload(capi_multistageiir_t *me_ptr,
                                                    uint32_t    num_cfg,
                                                    int8_t *    payload_cfg_ptr,
                                                    uint32_t    param_size,
                                                    uint32_t *  required_size_ptr,
                                                    uint32_t    param_id,
                                                    uint32_t    base_payload_size,
                                                    uint32_t    per_cfg_base_payload_size)
{
   capi_err_t capi_result          = CAPI_EOK;
   int8_t *   base_payload_ptr     = payload_cfg_ptr; // points to the start of the payload
   uint32_t   config_size          = 0;
   uint32_t   per_cfg_payload_size = 0;
   *required_size_ptr += base_payload_size; // 4B
   base_payload_ptr += *required_size_ptr;  // 4B
   // configuration payload
   // _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
   //| num_ | channel_type_group_mask | dynamic channel_type_mask_list[0] | payload1 |  _ _ _ _
   //|config| _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _  |_ _ _ _ _ |_ _ _ _ _ _
   //

   for (uint32_t count = 0; count < num_cfg; count++)
   {
      if ((*required_size_ptr + per_cfg_base_payload_size) > param_size)
      {
         MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                 "CAPI MSIIR : Failed while validating required size for config #%lu. Total config: %lu",
                 count,
                 num_cfg);
         return CAPI_ENEEDMORE;
      }
      else
      {
         uint32_t ch_type_group_mask = 0;
         base_payload_ptr += config_size;
         ch_type_group_mask = *((uint32_t *)base_payload_ptr);

         if (PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2 == param_id)
         {
            uint16_t num_biquad_stages = *((uint16_t*)(base_payload_ptr +
                                 (CAPI_CMN_INT32_SIZE_IN_BYTES + (capi_cmn_count_set_bits(ch_type_group_mask) * CAPI_CMN_INT32_SIZE_IN_BYTES) +
                                 sizeof(uint16_t)))); // accessing num_biquad_stages

            uint32_t incr_size = num_biquad_stages * MSIIR_COEFF_LENGTH * sizeof(int32_t); //filter_coeffs_payload_size
            incr_size          += num_biquad_stages * sizeof(int16_t);                     //num_shift_factor
            incr_size          += (num_biquad_stages & 0x1) ? sizeof(int16_t): 0;

            per_cfg_payload_size = per_cfg_base_payload_size + incr_size; // per cfg size excluding size for channel mask list
#ifdef CAPI_MSIIR_DEBUG_MSG
            MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI MSIIR : config: %lu, num_biquad_stages is %lu, incr_size: %lu, per_cfg_base_payload_size: %lu",
                    count,
                    num_biquad_stages,
                    incr_size,
                    per_cfg_base_payload_size);
#endif
         }
         else
         {
            per_cfg_payload_size = per_cfg_base_payload_size; // per cfg size excluding size for channel mask list
         }
#ifdef CAPI_MSIIR_DEBUG_MSG
         MSIIR_MSG(me_ptr->miid, DBG_MED_PRIO,
                 "CAPI MSIIR : Channel group mask %lu received for param 0x%#x. base_payload_ptr = %xlx",
                 ch_type_group_mask,
                 param_id,
                 base_payload_ptr);
#endif
         // check for group mask payload if only desired bits are set or not
         if (0 == (ch_type_group_mask >> CAPI_CMN_MAX_CHANNEL_MAP_GROUPS))
         {
            capi_result = capi_cmn_check_payload_validation(me_ptr->miid, ch_type_group_mask, per_cfg_payload_size,
                    count, param_size, &config_size, required_size_ptr);
            if(CAPI_FAILED(capi_result))
            {
               return capi_result;
            }
         }
         else
         {
             MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                    "CAPI MSIIR : Invalid configuration of channel group mask: %lu. More than maximum valid channel groups are "
                    "set, maximum valid channel groups: %lu.",
                    ch_type_group_mask,
                    CAPI_CMN_MAX_CHANNEL_MAP_GROUPS);
            return CAPI_EBADPARAM;
         }
      }
   }
#ifdef CAPI_MSIIR_DEBUG_MSG
   MSIIR_MSG(me_ptr->miid, DBG_MED_PRIO, "CAPI MSIIR : calculated size for param is %lu", *required_size_ptr);
#endif
   // validate the configs for duplication
   if (!capi_msiir_check_multi_ch_channel_mask_v2_param(me_ptr->miid,
                                                        num_cfg,
                                                        param_id,
                                                        (int8_t *)payload_cfg_ptr,
                                                        base_payload_size,
                                                        per_cfg_base_payload_size))
   {
      MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Trying to set different configuartion to same channel, returning");
      return CAPI_EBADPARAM;
   }
   return CAPI_EOK;
}

/* Returns FALSE if same channel is set to multiple times in different configs */
bool_t capi_msiir_check_multi_ch_channel_mask_v2_param(uint32_t miid,
                                                       uint32_t num_config,
                                                       uint32_t param_id,
                                                       int8_t * param_ptr,
                                                       uint32_t base_payload_size,
                                                       uint32_t per_cfg_base_payload_size)
{
   uint32_t *temp_mask_list_ptr                                        = NULL;
   int8_t *  data_ptr                                                  = param_ptr; // points to the start of the payload
   uint32_t  check_channel_mask_arr[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS]   = { 0 };
   uint32_t  current_channel_mask_arr[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS] = { 0 };
   bool_t    check                                                     = TRUE;
   uint32_t  offset                                                    = CAPI_CMN_INT32_SIZE_IN_BYTES;
   uint32_t  per_cfg_payload_size                                      = 0;

   for (uint32_t cnfg_cntr = 0; cnfg_cntr < num_config; cnfg_cntr++)
   {
      uint32_t channel_group_mask = 0;
      // Increment data ptr by calculated offset to point at next payload's group_mask
      data_ptr            += offset;
      channel_group_mask  = *((uint32_t *)data_ptr);
      if (PARAM_ID_MSIIR_TUNING_FILTER_CONFIG_PARAMS_V2 == param_id)
      {
         uint16_t num_biquad_stages = *((uint16_t*)(data_ptr +
                              (CAPI_CMN_INT32_SIZE_IN_BYTES + (capi_cmn_count_set_bits(channel_group_mask) * CAPI_CMN_INT32_SIZE_IN_BYTES) +
                              sizeof(uint16_t)))); // accessing num_biquad_stages

         uint32_t incr_size = num_biquad_stages * MSIIR_COEFF_LENGTH * sizeof(int32_t); //filter_coeffs_payload_size
         incr_size          += num_biquad_stages * sizeof(int16_t);                     //num_shift_factor
         incr_size          += (num_biquad_stages & 0x1) ? sizeof(int16_t): 0;

         per_cfg_payload_size = per_cfg_base_payload_size + incr_size; // per cfg size excluding size for channel mask list
#ifdef CAPI_MSIIR_DEBUG_MSG
         MSIIR_MSG(miid, DBG_HIGH_PRIO, "CAPI MSIIR : config: %lu, num_biquad_stages is %lu, incr_size: %lu, per_cfg_base_payload_size: %lu",
                 cnfg_cntr,
                 num_biquad_stages,
                 incr_size,
                 per_cfg_base_payload_size);
#endif
      }
      else
      {
         per_cfg_payload_size = per_cfg_base_payload_size; // per cfg size excluding size for channel mask list
      }
#ifdef CAPI_MSIIR_DEBUG_MSG
      MSIIR_MSG(miid, DBG_HIGH_PRIO, "CAPI MSIIR : channel_group_mask %lu, offset %lu", channel_group_mask, offset);
#endif
      temp_mask_list_ptr = (uint32_t *)(data_ptr + CAPI_CMN_INT32_SIZE_IN_BYTES);
      if(!capi_cmn_check_v2_channel_mask_duplication(miid,
                                                     cnfg_cntr,
                                                     channel_group_mask,
                                                     temp_mask_list_ptr,
                                                     current_channel_mask_arr,
                                                     check_channel_mask_arr,
                                                     &offset,
                                                     per_cfg_payload_size))
      {
          return FALSE;
      }
   }
   return check;
}

capi_err_t capi_msiir_set_enable_disable_per_channel_v2(capi_multistageiir_t *me_ptr,
                                                     const capi_buf_t *params_ptr,
                                                     uint32_t    param_id)
{
   int8_t *data_ptr = params_ptr->data_ptr;
   capi_err_t capi_result = CAPI_EOK;
   uint32_t param_size    = params_ptr->actual_data_len;
   if (param_size < sizeof(param_id_msiir_enable_v2_t))
   {
      MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
              "CAPI MSIIR : Enable SetParam 0x%lx, invalid param size %lx ",
              param_id,
              param_size);
      capi_result = CAPI_ENEEDMORE;
      return capi_result;
   }
   uint32_t req_payload_size          = 0;
   uint32_t base_payload_size         = sizeof(param_id_msiir_enable_v2_t);
   uint32_t per_cfg_base_payload_size = sizeof(param_id_msiir_ch_enable_v2_t);
   capi_result                        = capi_msiir_validate_multichannel_v2_payload(me_ptr,
                                                           data_ptr,
                                                           param_id,
                                                           param_size,
                                                           &req_payload_size,
                                                           base_payload_size,
                                                           per_cfg_base_payload_size);
   if (CAPI_FAILED(capi_result))
   {
      MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
              "CAPI MSIIR : Enable V2 SetParam 0x%lx, invalid param size %lu ,required_size %lu",
              param_id,
              param_size,
              req_payload_size);
      return capi_result;
   }
   if (param_size < req_payload_size)
   {
      MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Insufficient payload size %d. Req size %lu", param_size, req_payload_size);
      return CAPI_ENEEDMORE;
   }
   else
   {
      // set the payload
      capi_result = capi_msiir_set_enable_disable_v2_payload(me_ptr, data_ptr, per_cfg_base_payload_size);
      if (CAPI_FAILED(capi_result))
      {
         return capi_result;
      }
      MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI MSIIR : Enable V2 Set Param set Successfully");
   }
   capi_msiir_raise_process_check_event(me_ptr);
   capi_msiir_update_delay_event(me_ptr);
   return CAPI_EOK;
}

capi_err_t capi_msiir_set_enable_disable_v2_payload(capi_multistageiir_t *me_ptr,
                                                   int8_t* data_ptr,
                                                   uint32_t per_cfg_base_payload_size)
{
   uint32_t * ch_mask_list_ptr = NULL;
   uint32_t * enable_ref_ptr   = NULL;
   uint32_t num_cfg            = *((uint32_t*)data_ptr);
   uint32_t offset             = sizeof(param_id_msiir_enable_v2_t);

   // loop through all the configs
   for (uint32_t count = 0; count < num_cfg; count++)
   {
      uint32_t ch_mask_arr_index  = 0;
      uint32_t channel_group_mask = 0;
      uint32_t ch_mask_list_size  = 0;

      // Increment data ptr by calculated offset to point at next payload's group_mask
      data_ptr += offset;
      param_id_msiir_ch_enable_v2_t *temp_en_cfg_ptr = (param_id_msiir_ch_enable_v2_t *)data_ptr;
      channel_group_mask                             = temp_en_cfg_ptr->channel_type_group_mask;

      ch_mask_list_ptr  = (uint32_t *)(data_ptr + CAPI_CMN_INT32_SIZE_IN_BYTES); // points to the mask array
      ch_mask_list_size = capi_cmn_count_set_bits(channel_group_mask);
      enable_ref_ptr    = ch_mask_list_ptr + ch_mask_list_size; // points to the enable payload

      MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI MSIIR :Enable for config #%lu is %lu", count + 1, *enable_ref_ptr);

      for (uint32_t group_no = 0; group_no < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; group_no++)
      {
         // check if the group is configured.
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
               if (ch_mask & ch_mask_list_ptr[ch_mask_arr_index]) // check if this channel is set anywhere in this group
               {
                   // convert channel_type to channel index, channel index has range 0 ~ (PCM_MAX_CHANNEL_MAP_V2-1)
                   int32_t ch_idx = me_ptr->channel_map_to_index[i];
                   if ((ch_idx < 0) || (ch_idx >= (int32_t)me_ptr->media_fmt[0].format.num_channels))
                   {
                      continue;
                   }
                   me_ptr->enable_flag[ch_idx] = *enable_ref_ptr;
                   //AR_MSG(DBG_MED_PRIO, "CAPI MSIIR : Set enable: channel: %lu enable: %lu", i, me_ptr->enable_flag[ch_idx]);
               }
            }
            //MOVE TO HERE //print the range as well
#ifdef CAPI_MSIIR_DEBUG_MSG
      MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                  "CAPI MSIIR : config: #%lu, channel mask: %#lx, group no: %lu,"
                  "includes channels from %lu to %lu. Enable: %#lx",
                  count,
                  ch_mask_list_ptr[ch_mask_arr_index],
                  group_no,
                  start_ch_map,
                  end_ch_map - 1,
                  *enable_ref_ptr);
#endif
            ch_mask_arr_index++;
         }
      }
      offset = per_cfg_base_payload_size + (ch_mask_arr_index << 2); //sizeof(uint32_t)
   }
   return CAPI_EOK;
}

capi_err_t capi_msiir_set_pregain_per_channel_v2(capi_multistageiir_t *me_ptr,
                                                     const capi_buf_t *params_ptr,
                                                     uint32_t          param_id)
{
   int8_t *data_ptr = params_ptr->data_ptr;
   capi_err_t capi_result = CAPI_EOK;
   int32_t param_size = params_ptr->actual_data_len;

   if (param_size < sizeof(param_id_msiir_pregain_v2_t))
   {
      MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
              "CAPI MSIIR : Pregain SetParam 0x%lx, invalid param size %lx ",
              param_id,
              param_size);
      capi_result = CAPI_ENEEDMORE;
      return capi_result;
   }
   uint32_t req_payload_size          = 0;
   uint32_t base_payload_size         = sizeof(param_id_msiir_pregain_v2_t);
   uint32_t per_cfg_base_payload_size = sizeof(param_id_msiir_ch_pregain_v2_t);
   capi_result                        = capi_msiir_validate_multichannel_v2_payload(me_ptr,
                                                           data_ptr,
                                                           param_id,
                                                           param_size,
                                                           &req_payload_size,
                                                           base_payload_size,
                                                           per_cfg_base_payload_size);
   if (CAPI_FAILED(capi_result))
   {
      MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
              "CAPI MSIIR : Pregain V2 SetParam 0x%lx, invalid param size %lu ,required_size %lu",
              param_id,
              param_size,
              req_payload_size);
      return capi_result;
   }
   if (param_size < req_payload_size)
   {
      MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Insufficient payload size %d. Req size %lu", param_size, req_payload_size);
      return CAPI_ENEEDMORE;
   }
   else
   {
      // set the payload
      capi_result = capi_msiir_set_pregain_v2_payload(me_ptr, data_ptr, per_cfg_base_payload_size);
      if (CAPI_FAILED(capi_result))
      {
         return capi_result;
      }
      MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI MSIIR : Pregain V2 Set Param set Successfully");
   }
   return CAPI_EOK;
}

capi_err_t capi_msiir_set_pregain_v2_payload(capi_multistageiir_t *me_ptr,
                                                 int8_t* data_ptr,
                                                 uint32_t per_cfg_base_payload_size)
{
   MSIIR_RESULT                          result_lib         = MSIIR_SUCCESS;
   uint32_t                              num_cfg            = *((uint32_t*)data_ptr);
   uint32_t                              offset             = sizeof(param_id_msiir_pregain_v2_t);

   // loop through all the configs
   for (uint32_t count = 0; count < num_cfg; count++)
   {
      uint32_t *ch_mask_list_ptr  = NULL;
      uint32_t *pregain_ref_ptr   = NULL;
      uint32_t ch_mask_arr_index  = 0;
      uint32_t channel_group_mask = 0;
      uint32_t ch_mask_list_size  = 0;

      // Increment data ptr by calculated offset to point at next payload's group_mask
      data_ptr += offset; //4B
      param_id_msiir_ch_pregain_v2_t *temp_pg_cfg_ptr = (param_id_msiir_ch_pregain_v2_t *)data_ptr;
      channel_group_mask                              = temp_pg_cfg_ptr->channel_type_group_mask;

      ch_mask_list_ptr  = (uint32_t *)(data_ptr + CAPI_CMN_INT32_SIZE_IN_BYTES); // points to the mask array
      ch_mask_list_size = capi_cmn_count_set_bits(channel_group_mask);
      pregain_ref_ptr   = ch_mask_list_ptr + ch_mask_list_size; // points to the en payload

      MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI MSIIR :Pregain for config #%lu is %lu", count, *pregain_ref_ptr);

      for (uint32_t group_no = 0; group_no < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; group_no++)
      {
         // check if the group is configured.
         if (CAPI_CMN_IS_BIT_SET_AT_POS_IN_32B_VAL(channel_group_mask, group_no))
         {
            msiir_pregain_t pregain = 0;
            uint32_t start_ch_map   = group_no * CAPI_CMN_CHANNELS_PER_MASK;
            uint32_t end_ch_map     = (group_no + 1) * CAPI_CMN_CHANNELS_PER_MASK;
            uint32_t j              = 0;
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
               if (ch_mask & ch_mask_list_ptr[ch_mask_arr_index]) // check if this channel is set anywhere in this group
               {
                   // convert channel_type to channel index, channel index has range 0 ~ (PCM_MAX_CHANNEL_MAP_V2-1)
                   int32_t ch_idx = me_ptr->channel_map_to_index[i];
                   if ((ch_idx < 0) || (ch_idx >= (int32_t)me_ptr->media_fmt[0].format.num_channels))
                   {
                      continue;
                   }
                   pregain = (msiir_pregain_t)(*pregain_ref_ptr);
                   if ((me_ptr->is_first_frame) || (me_ptr->per_chan_msiir_pregain[ch_idx] == pregain))
                   {
                      result_lib =
                         msiir_set_param(&(me_ptr->msiir_lib[ch_idx]), MSIIR_PARAM_PREGAIN, (void *)&pregain, sizeof(pregain));
                      if (MSIIR_SUCCESS != result_lib)
                      {
                    	  MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Set pregain error %d", result_lib);
                         return CAPI_EFAILED;
                      }
                   }
                   else
                   {
                      me_ptr->start_cross_fade = TRUE;
                   }
                   // save the new configuration
                   me_ptr->per_chan_msiir_pregain[ch_idx] = pregain;
               }
            }
#ifdef CAPI_MSIIR_DEBUG_MSG
      MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                  "CAPI MSIIR : config: #%lu, channel mask: %#lx, group no: %lu,"
                  "includes channels from %lu to %lu. pregain: %#lx",
                  count,
                  ch_mask_list_ptr[ch_mask_arr_index],
                  group_no,
                  start_ch_map,
                  end_ch_map - 1,
                  pregain);
#endif
            ch_mask_arr_index++;
         }
      }
      offset = per_cfg_base_payload_size + (ch_mask_arr_index << 2); //sizeof(uint32_t)
   }
   return CAPI_EOK;
}

capi_err_t capi_msiir_set_config_per_channel_v2(capi_multistageiir_t *me_ptr,
                                                    const capi_buf_t *params_ptr,
                                                    uint32_t         param_id)
{
   int8_t *data_ptr = params_ptr->data_ptr;
   capi_err_t capi_result = CAPI_EOK;
   uint32_t param_size = params_ptr->actual_data_len;

   if (param_size < sizeof(param_id_msiir_config_v2_t))
   {
      MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
              "CAPI MSIIR : Filter config SetParam 0x%lx, invalid param size %lx ",
              param_id,
              param_size);
      capi_result = CAPI_ENEEDMORE;
      return capi_result;
   }
   uint32_t req_payload_size          = 0;
   uint32_t base_payload_size         = sizeof(param_id_msiir_config_v2_t);
   uint32_t per_cfg_base_payload_size = sizeof(param_id_msiir_ch_filter_config_v2_t);
   capi_result                        = capi_msiir_validate_multichannel_v2_payload(me_ptr,
                                                           data_ptr,
                                                           param_id,
                                                           param_size,
                                                           &req_payload_size,
                                                           base_payload_size,
                                                           per_cfg_base_payload_size);
   if (CAPI_FAILED(capi_result))
   {
      MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
              "CAPI MSIIR : Filter config V2 SetParam 0x%lx, invalid param size %lu ,required_size %lu",
              param_id,
              param_size,
              req_payload_size);
      return capi_result;
   }
   if (param_size < req_payload_size)
   {
      MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Insufficient payload size %d. Req size %lu", param_size, req_payload_size);
      return CAPI_ENEEDMORE;
   }
   else
   {
      // set the payload
      capi_result = capi_msiir_set_config_v2_payload(me_ptr, data_ptr);
      if (CAPI_FAILED(capi_result))
      {
         MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI MSIIR : SET V2 cfg payload failed");
         return capi_result;
      }
      MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO, "CAPI MSIIR : Filter config V2 Set Param set Successfully");
   }
   return CAPI_EOK;
}

capi_err_t capi_msiir_set_config_v2_payload(capi_multistageiir_t *me_ptr,
                                                 int8_t* payload_ptr)
{
   uint32_t     num_cfg            = *((uint32_t*)payload_ptr);

   uint8_t *data_ptr   = (uint8_t *)(((uint8_t *)(payload_ptr)) + sizeof(param_id_msiir_config_v2_t));
   uint8_t *data_ptr_1 = (uint8_t *)(((uint8_t *)(payload_ptr)) + sizeof(param_id_msiir_config_v2_t));
   uint32_t offset     = 0;
   ///
   // loop through all the configs
   for (uint32_t count = 0; count < num_cfg; count++)
   {
      uint32_t *ch_mask_list_ptr          = NULL;
      uint32_t ch_mask_arr_index          = 0;
      uint32_t channel_group_mask         = 0;
      uint32_t ch_mask_list_size          = 0;

      // Increment data ptr by calculated offset to point at next payload's group_mask
      data_ptr_1 += offset;
      param_id_msiir_ch_filter_config_v2_t *this_chan_cfg_inp_ptr = (param_id_msiir_ch_filter_config_v2_t *)data_ptr_1;
      channel_group_mask                                          = this_chan_cfg_inp_ptr->channel_type_group_mask;

      data_ptr   = data_ptr_1;
      ch_mask_list_ptr  = (uint32_t *)(data_ptr + CAPI_CMN_INT32_SIZE_IN_BYTES); // points to the mask array
      ch_mask_list_size = capi_cmn_count_set_bits(channel_group_mask);
      uint32_t ch_mask_list_size_in_bytes = (ch_mask_list_size * CAPI_CMN_INT32_SIZE_IN_BYTES);
      int32_t num_biquad_stages           = *((uint16_t*)(data_ptr + CAPI_CMN_INT32_SIZE_IN_BYTES + ch_mask_list_size_in_bytes +
                                             sizeof(uint16_t)));     // incrementing offset to point to the num_biquad_stages
#ifdef CAPI_MSIIR_DEBUG_MSG
      MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO,
             "CAPI MSIIR : Set config for config %lu, channel group mask %lu, stages %lu",
             count+1,
             channel_group_mask,
             num_biquad_stages);
#endif
      data_ptr += (sizeof(param_id_msiir_ch_filter_config_v2_t) + ch_mask_list_size_in_bytes);
      offset    = (sizeof(param_id_msiir_ch_filter_config_v2_t) + ch_mask_list_size_in_bytes);

      // move pointer to filter coeffs of current config
      int32_t *coeff_ptr = (int32_t *)data_ptr;
      int32_t *coeff_ptr_1 = (int32_t *)data_ptr;

      size_t coeff_size = num_biquad_stages * MSIIR_COEFF_LENGTH * sizeof(int32_t);

      // move pointer to numerator shift factors of current config
      data_ptr += coeff_size;
      offset += coeff_size;

      int16_t *shift_factor_ptr = (int16_t *)data_ptr;
      int16_t *shift_factor_ptr_1 = (int16_t *)data_ptr;
      size_t numerator_shift_fac_size = num_biquad_stages * sizeof(int16_t);

      data_ptr += numerator_shift_fac_size;
      offset += numerator_shift_fac_size;

      // move pointer to skip the padding bit if necessary
      bool_t is_odd_stages = (bool_t)(num_biquad_stages & 0x1);
      if (is_odd_stages)
      {
         data_ptr += sizeof(int16_t); // 16-bits padding
         offset += sizeof(int16_t);
      }
#ifdef CAPI_MSIIR_DEBUG_MSG
   MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO,"CAPI MSIIR : offset for config %lu is %lu.",
         count + 1, offset);
#endif
      for (uint32_t group_no = 0; group_no < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; group_no++)
      {
         // check if the group is configured.
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

               if (ch_mask & ch_mask_list_ptr[ch_mask_arr_index]) // check if this channel is set anywhere in this group
               {
                   // convert channel_type to channel index, channel index has range 0 ~ (PCM_MAX_CHANNEL_MAP_V2-1)
                   int32_t ch_idx           = me_ptr->channel_map_to_index[i];
#ifdef CAPI_MSIIR_DEBUG_MSG
         MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO,"CAPI MSIIR : channel mask : %lu, ch_idx : %ld, ch_mask_list_ptr[%lu] : %#lx.",
                 ch_mask,
                 ch_idx,
                 ch_mask_arr_index,
                 ch_mask_list_ptr[ch_mask_arr_index]);
#endif
                   if ((ch_idx < 0) || (ch_idx >= (int32_t)me_ptr->media_fmt[0].format.num_channels))
                   {
                      /*offset = sizeof(param_id_msiir_ch_filter_config_v2_t) + ch_mask_list_size_in_bytes;
                      offset += num_biquad_stages * MSIIR_COEFF_LENGTH * sizeof(int32_t); // filter_coeffs
                      offset += num_biquad_stages * sizeof(int16_t);                      // num_shift_factor
                      offset += (num_biquad_stages & 0x1) ? sizeof(int16_t): 0;           // zero padding */
                      continue;
                   }
                   //offset = 0;
                   if ((!me_ptr->is_first_frame) && (num_biquad_stages != me_ptr->per_chan_msiir_cfg_max[ch_idx].num_stages))
                   {
                      // do cross fading if num stages changed in the middle of data processing
                      me_ptr->start_cross_fade = TRUE;
                   }

                   me_ptr->per_chan_msiir_cfg_max[ch_idx].num_stages = num_biquad_stages;

                   coeff_ptr = coeff_ptr_1;
                   shift_factor_ptr = shift_factor_ptr_1;

                   for (int32_t stage = 0; stage < num_biquad_stages; stage++)
                   {
                      for (int32_t idx = 0; idx < MSIIR_COEFF_LENGTH; idx++)
                      {
                         int32_t iir_coeff = *coeff_ptr++;

                         if ((!me_ptr->is_first_frame) &&
                             (iir_coeff != me_ptr->per_chan_msiir_cfg_max[ch_idx].coeffs_struct[stage].iir_coeffs[idx]))
                         {
                             me_ptr->start_cross_fade = TRUE;
                         }

                         me_ptr->per_chan_msiir_cfg_max[ch_idx].coeffs_struct[stage].iir_coeffs[idx] = iir_coeff;
                      }
                   }

                   for (int32_t stage = 0; stage < num_biquad_stages; stage++)
                   {
                      int32_t shift_factor = (int32_t)(*shift_factor_ptr++);

                      if ((!me_ptr->is_first_frame) &&
                          (shift_factor != me_ptr->per_chan_msiir_cfg_max[ch_idx].coeffs_struct[stage].shift_factor))
                      {
                          me_ptr->start_cross_fade = TRUE;
                      }

                      me_ptr->per_chan_msiir_cfg_max[ch_idx].coeffs_struct[stage].shift_factor = shift_factor;
                   }

                   if (!me_ptr->start_cross_fade)
                   {
                      uint32_t param_size = sizeof(me_ptr->per_chan_msiir_cfg_max[ch_idx].num_stages) +
                                            num_biquad_stages * MSIIR_COEFF_LENGTH * sizeof(int32_t) +
                                            num_biquad_stages * sizeof(int32_t);

                      msiir_set_param(&(me_ptr->msiir_lib[ch_idx]),
                                      MSIIR_PARAM_CONFIG,
                                      (void *)&(me_ptr->per_chan_msiir_cfg_max[ch_idx]),
                                      param_size);

                      // when we reach here, data_ptr should points to the start of next channel's
                      // config params (param_id_channel_type_msiir_config_pair_t followed by filter coeff and
                      // num_shift_factor
                      capi_msiir_update_delay_event(me_ptr);
                   }
               }
            }
            MSIIR_MSG(me_ptr->miid, DBG_HIGH_PRIO,
                   "CAPI MSIIR : Set config for group %d, includes channels from %lu to %lu, stages %lu.",
                   group_no,
                   start_ch_map,
                   end_ch_map,
                   num_biquad_stages);
            ch_mask_arr_index++;
         }
      }
   }

   return CAPI_EOK;
}

capi_err_t capi_msiir_get_enable_disable_per_channel_v2(capi_multistageiir_t   *me_ptr,
                                                        capi_buf_t             *params_ptr)
{
   if (me_ptr->enable_params.params_ptr.data_ptr)
   {
      if (params_ptr->max_data_len < me_ptr->enable_params.params_ptr.actual_data_len)
      {
         params_ptr->actual_data_len = me_ptr->enable_params.params_ptr.actual_data_len;
         MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : Get Enable/Disable, Bad payload size %lu: actual = %lu",
                params_ptr->max_data_len,
                me_ptr->enable_params.params_ptr.actual_data_len);
             return CAPI_ENEEDMORE;
      }
      params_ptr->actual_data_len = memscpy(params_ptr->data_ptr,
                                            params_ptr->max_data_len,
                                            me_ptr->enable_params.params_ptr.data_ptr,
                                            me_ptr->enable_params.params_ptr.actual_data_len);
   }
   else if (me_ptr->media_fmt_received)
   {
      param_id_msiir_config_v2_t *en_dis_cfg_ptr      = (param_id_msiir_config_v2_t *)params_ptr->data_ptr;
      uint32_t *payload_ptr                           = (uint32_t *)params_ptr->data_ptr;
      uint32_t chan_mask_list[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS] = { 0 };
      uint32_t channel_type_group_mask                = 0;
      uint32_t num_channels                           = me_ptr->media_fmt[0].format.num_channels;
      en_dis_cfg_ptr->num_config                      = 1; //disable

      uint32_t required_payload_size = sizeof(param_id_msiir_config_v2_t) +
                                       sizeof(param_id_msiir_ch_enable_v2_t) +
                                       CAPI_CMN_GET_MAX_CHANNEL_GROUPS_NEEDED(num_channels) * CAPI_CMN_INT32_SIZE_IN_BYTES;

      if (params_ptr->max_data_len < required_payload_size)
      {
    	  MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Get Enable/Disable, Bad payload size %lu", params_ptr->max_data_len);
         params_ptr->actual_data_len = 0;
         return CAPI_ENEEDMORE;
      }

      for(uint32_t ch = 0; ch < num_channels; ch ++)
      {
         uint32_t curr_channel_type    = me_ptr->media_fmt[0].channel_type[ch];
         uint32_t curr_ch_mask         = (1 << CAPI_CMN_MOD_WITH_32(curr_channel_type));
         uint32_t curr_ch_index_grp_no = CAPI_CMN_DIVIDE_WITH_32(curr_channel_type);
         uint32_t curr_group_mask      = (1 << CAPI_CMN_MOD_WITH_32(curr_ch_index_grp_no));

         if(!CAPI_CMN_IS_BIT_SET_AT_POS_IN_32B_VAL(curr_group_mask, curr_ch_index_grp_no)) //if this group is not set, then only set
         {
            channel_type_group_mask |= (1 << CAPI_CMN_MOD_WITH_32(curr_ch_index_grp_no));
         }
         chan_mask_list[curr_ch_index_grp_no] |= curr_ch_mask;
      }
      payload_ptr++;  //ref to group_mask

      param_id_msiir_ch_enable_v2_t *multi_ch_iir_enable_ptr = (param_id_msiir_ch_enable_v2_t*)payload_ptr;
      multi_ch_iir_enable_ptr->channel_type_group_mask       = channel_type_group_mask;

      uint32_t *ch_mask_list_ptr = (payload_ptr++);
      uint32_t ch_mask_array_idx = 0;
      for(uint32_t group_no = 0; group_no < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; group_no ++)
      {
         if(chan_mask_list[group_no])
         {
             ch_mask_list_ptr[ch_mask_array_idx] = chan_mask_list[group_no];
             ch_mask_array_idx++;
         }
      }
      payload_ptr    += ch_mask_array_idx;
      *(payload_ptr) = 0; //disable value

      params_ptr->actual_data_len = required_payload_size;
   }
   else
   {
      params_ptr->actual_data_len = 0;
      return CAPI_EBADPARAM;
   }
   return CAPI_EOK;
}

capi_err_t capi_msiir_get_pregain_per_channel_v2(
        capi_multistageiir_t   *me_ptr,
        capi_buf_t             *params_ptr)
{
   if (me_ptr->pregain_params.params_ptr.data_ptr)
   {
      if (params_ptr->max_data_len < me_ptr->pregain_params.params_ptr.actual_data_len)
      {
         params_ptr->actual_data_len = me_ptr->pregain_params.params_ptr.actual_data_len;
         MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : Get pregain, Bad payload size %lu: actual = %lu",
                params_ptr->max_data_len,
                me_ptr->pregain_params.params_ptr.actual_data_len);

         return CAPI_ENEEDMORE;
      }
      params_ptr->actual_data_len = memscpy(params_ptr->data_ptr,
                                            params_ptr->max_data_len,
                                            me_ptr->pregain_params.params_ptr.data_ptr,
                                            me_ptr->pregain_params.params_ptr.actual_data_len);
   }
   else if (me_ptr->media_fmt_received)
   {
      param_id_msiir_pregain_v2_t *iir_pregain_pkt_ptr = (param_id_msiir_pregain_v2_t *)(params_ptr->data_ptr);
      uint32_t *payload_ptr                            = (uint32_t *)params_ptr->data_ptr;
      uint32_t chan_mask_list[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS]  = { 0 };
      uint32_t channel_type_group_mask                 = 0;
      uint32_t num_channels                            = me_ptr->media_fmt[0].format.num_channels;
      iir_pregain_pkt_ptr->num_config                  = 1; // would return the same default pregain value
                                                            // for all channels

      uint32_t required_payload_size = sizeof(param_id_msiir_pregain_v2_t) +
                                       sizeof(param_id_msiir_ch_pregain_v2_t) +
                                       CAPI_CMN_GET_MAX_CHANNEL_GROUPS_NEEDED(num_channels) * CAPI_CMN_INT32_SIZE_IN_BYTES;

      if (params_ptr->max_data_len < required_payload_size)
      {
         MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO, "CAPI MSIIR : Get pregain, Bad payload size %lu", params_ptr->max_data_len);
         return CAPI_ENEEDMORE;
      }

      for(uint32_t ch = 0; ch < num_channels; ch ++)
      {
         uint32_t curr_channel_type    = me_ptr->media_fmt[0].channel_type[ch];
         uint32_t curr_ch_mask         = (1 << CAPI_CMN_MOD_WITH_32(curr_channel_type));
         uint32_t curr_ch_index_grp_no = CAPI_CMN_DIVIDE_WITH_32(curr_channel_type);
         uint32_t curr_group_mask      = (1 << CAPI_CMN_MOD_WITH_32(curr_ch_index_grp_no));

         if(!CAPI_CMN_IS_BIT_SET_AT_POS_IN_32B_VAL(curr_group_mask, curr_ch_index_grp_no)) //if this group is not set, then only set
         {
            channel_type_group_mask |= (1 << CAPI_CMN_MOD_WITH_32(curr_ch_index_grp_no));
         }
         chan_mask_list[curr_ch_index_grp_no] |= curr_ch_mask;
      }
      // unity gain in Q27
      msiir_pregain_t pregain_lib = (msiir_pregain_t)((int32_t)(1 << MSIIR_Q_PREGAIN));
      uint32_t actual_param_size  = 0;
      MSIIR_RESULT result_lib     = MSIIR_SUCCESS;
      result_lib = msiir_get_param(&(me_ptr->msiir_lib[0]),
                                      MSIIR_PARAM_PREGAIN,
                                      (void *)&pregain_lib,
                                      sizeof(pregain_lib),
                                      &actual_param_size);

      if (actual_param_size > sizeof(pregain_lib) || (MSIIR_SUCCESS != result_lib))
      {
         MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : get pregain error result %d, size %lu",
                result_lib,
                actual_param_size);
         return CAPI_EFAILED;
      }
      int32_t pregain = (int32_t)pregain_lib;

      payload_ptr++;  //ref to group_mask

      param_id_msiir_ch_pregain_v2_t *multi_ch_iir_pregain_ptr = (param_id_msiir_ch_pregain_v2_t*)payload_ptr;
      multi_ch_iir_pregain_ptr->channel_type_group_mask = channel_type_group_mask;

      uint32_t *ch_mask_list_ptr = (payload_ptr++);
      uint32_t ch_mask_array_idx = 0;
      for(uint32_t group_no = 0; group_no < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; group_no ++)
      {
         if(chan_mask_list[group_no])
         {
             ch_mask_list_ptr[ch_mask_array_idx] = chan_mask_list[group_no];
             ch_mask_array_idx++;
         }
      }
      payload_ptr    += ch_mask_array_idx;
      *(payload_ptr) = pregain; //pregain value

      params_ptr->actual_data_len = required_payload_size;
   }
   else
   {
      params_ptr->actual_data_len = 0;
      return CAPI_EBADPARAM;
   }
   return CAPI_EOK;
}

capi_err_t capi_msiir_get_config_per_channel_v2(
        capi_multistageiir_t   *me_ptr,
        capi_buf_t             *params_ptr)
{
   if (me_ptr->config_params.params_ptr.data_ptr)
   {
      if (params_ptr->max_data_len < me_ptr->config_params.params_ptr.actual_data_len)
      {
         params_ptr->actual_data_len = me_ptr->config_params.params_ptr.actual_data_len;
         MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : Get IIR configuration, Bad payload size %lu, Actual = %lu",
                params_ptr->max_data_len,
                params_ptr->actual_data_len);
             return CAPI_ENEEDMORE;
      }
      params_ptr->actual_data_len = memscpy(params_ptr->data_ptr,
                                            params_ptr->max_data_len,
                                            me_ptr->config_params.params_ptr.data_ptr,
                                            me_ptr->config_params.params_ptr.actual_data_len);
   }
   else if (me_ptr->media_fmt_received)
   {
      param_id_msiir_config_v2_t *iir_config_pkt_ptr  = (param_id_msiir_config_v2_t *)(params_ptr->data_ptr);
      uint32_t *payload_ptr                           = (uint32_t *)params_ptr->data_ptr;
      uint32_t chan_mask_list[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS] = { 0 };
      uint32_t channel_type_group_mask                = 0;
      uint32_t num_channels                           = me_ptr->media_fmt[0].format.num_channels;
      uint32_t actual_size                            = sizeof(param_id_msiir_config_v2_t);

      iir_config_pkt_ptr->num_config                  = 1; // would return the same default pregain value
                                                           // for all channels
      uint8_t *data_ptr = (uint8_t *)(((uint8_t *)(params_ptr->data_ptr)) + sizeof(param_id_msiir_config_v2_t));
      payload_ptr = (uint32_t *)data_ptr;

      for(uint32_t ch = 0; ch < num_channels; ch ++)
      {
         uint32_t curr_channel_type    = me_ptr->media_fmt[0].channel_type[ch];
         uint32_t curr_ch_mask         = (1 << CAPI_CMN_MOD_WITH_32(curr_channel_type));
         uint32_t curr_ch_index_grp_no = CAPI_CMN_DIVIDE_WITH_32(curr_channel_type);
         uint32_t curr_group_mask      = (1 << CAPI_CMN_MOD_WITH_32(curr_ch_index_grp_no));

         if(!CAPI_CMN_IS_BIT_SET_AT_POS_IN_32B_VAL(curr_group_mask, curr_ch_index_grp_no)) //if this group is not set, then only set
         {
            channel_type_group_mask |= (1 << CAPI_CMN_MOD_WITH_32(curr_ch_index_grp_no));
         }
         chan_mask_list[curr_ch_index_grp_no] |= curr_ch_mask;
      }
      *payload_ptr = channel_type_group_mask;

      uint32_t *ch_mask_list_ptr = (payload_ptr++);
      uint32_t ch_mask_array_idx = 0;
      for(uint32_t group_no = 0; group_no < CAPI_CMN_MAX_CHANNEL_MAP_GROUPS; group_no ++)
      {
         if(chan_mask_list[group_no])
         {
             ch_mask_list_ptr[ch_mask_array_idx] = chan_mask_list[group_no];
             ch_mask_array_idx++;
         }
      }
      payload_ptr += ch_mask_array_idx;
      capi_one_chan_msiir_config_static_param_t *this_chan_cfg_ptr = (capi_one_chan_msiir_config_static_param_t*)payload_ptr;
      this_chan_cfg_ptr->reserved = 0;

      capi_one_chan_msiir_config_max_t *one_chan_msiir_config_max_ptr = &(me_ptr->default_per_chan_msiir_cfg_max);
      // if the filter is disabled, the library uses default 0 stages for copy through
      uint32_t     num_biquad_stages = 0;
      uint32_t     actual_param_size = 0;
      MSIIR_RESULT result_lib        = MSIIR_SUCCESS;

      one_chan_msiir_config_max_ptr = &(me_ptr->per_chan_msiir_cfg_max[0]);
#ifdef CAPI_MSIIR_DEBUG_MSG
      MSIIR_MSG(me_ptr->miid, DBG_LOW_PRIO, "CAPI MSIIR : Config get param from CUR_INST ");
#endif
      result_lib = msiir_get_param(&(me_ptr->msiir_lib[0]),
                                   MSIIR_PARAM_CONFIG,
                                   (void *)one_chan_msiir_config_max_ptr,
                                   sizeof(me_ptr->per_chan_msiir_cfg_max[0]),
                                   &actual_param_size);
      if (actual_param_size > sizeof(me_ptr->per_chan_msiir_cfg_max[0]) || (MSIIR_SUCCESS != result_lib))
      {
         MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : get delay error result %d, size %lu",
                result_lib,
                actual_param_size);
         return CAPI_EFAILED;
      }

      num_biquad_stages = one_chan_msiir_config_max_ptr->num_stages;

      if (num_biquad_stages > (int32_t)MSIIR_MAX_STAGES)
      {
         MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : Get config params, bad internal num_biquad_stages %d",
                (int)num_biquad_stages);
         return CAPI_EFAILED;
      }

      this_chan_cfg_ptr->num_biquad_stages = num_biquad_stages;

      // move pointer to filter coeffs
      data_ptr += sizeof(param_id_msiir_ch_filter_config_v2_t) +
                  ch_mask_array_idx * CAPI_CMN_INT32_SIZE_IN_BYTES;
      actual_size += sizeof(param_id_msiir_ch_filter_config_v2_t) +
                     ch_mask_array_idx * CAPI_CMN_INT32_SIZE_IN_BYTES;

      int32_t *coeff_ptr = (int32_t *)data_ptr;
      for (int32_t stage = 0; stage < num_biquad_stages; stage++)
      {
         for (int32_t idx = 0; idx < MSIIR_COEFF_LENGTH; idx++)
         {
            *coeff_ptr++ = one_chan_msiir_config_max_ptr->coeffs_struct[stage].iir_coeffs[idx];
         }
      }

      size_t coeff_size = num_biquad_stages * MSIIR_COEFF_LENGTH * sizeof(int32_t);

      // move pointer to numerator shift factors
      data_ptr += coeff_size;
      // add size of filter coeffs
      actual_size += coeff_size;

      int16_t *shift_factor_ptr = (int16_t *)data_ptr;

      for (int32_t stage = 0; stage < num_biquad_stages; stage++)
      {
         *shift_factor_ptr++ = (int16_t)(one_chan_msiir_config_max_ptr->coeffs_struct[stage].shift_factor);
      }

      size_t num_shift_fac_size = num_biquad_stages * sizeof(int16_t);

      // add size of numerator shift factors
      actual_size += num_shift_fac_size;

      // add padding if odd stages
      bool_t is_odd_stages = (bool_t)(num_biquad_stages & 0x1);
      if (is_odd_stages)
      {
         data_ptr += num_shift_fac_size + sizeof(int16_t);
         actual_size += sizeof(int16_t); // 16-bits padding
      }

      params_ptr->actual_data_len = actual_size;

      if (params_ptr->max_data_len < actual_size)
      {
         MSIIR_MSG(me_ptr->miid, DBG_ERROR_PRIO,
                "CAPI MSIIR : Get config params, Bad payload size %d, required %d",
                (int)params_ptr->max_data_len,
                (int)actual_size);
         return CAPI_ENEEDMORE;
      }
   }
   else
   {
      params_ptr->actual_data_len = 0;
      return CAPI_EBADPARAM;
   }

   return CAPI_EOK;
}
