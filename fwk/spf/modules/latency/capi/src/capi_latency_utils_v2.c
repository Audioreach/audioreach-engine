/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_latency_utils_v2.c
 *
 * C source file to implement the Audio Post Processor Interface for
 * latency Module
 */

#include "capi_latency_utils.h"

capi_err_t capi_latency_set_config_v2(capi_latency_t *me_ptr, uint32_t param_id, uint32_t param_size, int8_t *data_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (param_size < sizeof(param_id_latency_cfg_v2_t))
   {
      AR_MSG(DBG_ERROR_PRIO, "CAPI Latency Delay CFG, Bad param size %lu", param_size);
      return CAPI_ENEEDMORE;
   }
    param_id_latency_cfg_v2_t *delay_param_ptr = (param_id_latency_cfg_v2_t *)data_ptr;
    me_ptr->cfg_mode                        = delay_param_ptr->cfg_mode;

    AR_MSG(DBG_HIGH_PRIO, "CAPI Latency Delay CFG mode = %lu", me_ptr->cfg_mode);

    if (LATENCY_MODE_GLOBAL == me_ptr->cfg_mode)
    {
       param_id_latency_cfg_v2_t *delay_param_ptr = (param_id_latency_cfg_v2_t *)data_ptr;

       if (delay_param_ptr->global_delay_us > CAPI_LATENCY_MAX_DELAY_US)
       {
          AR_MSG(DBG_ERROR_PRIO,
                 "CAPI Latency Global Delay %lu too high, max is %lu",
                 delay_param_ptr->global_delay_us,
                 CAPI_LATENCY_MAX_DELAY_US);
          return CAPI_EBADPARAM;
       }

       if (NULL != me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr)
       {
          posal_memory_free(me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr);
          me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr             = NULL;
          me_ptr->cache_delay_v2.cache_delay_per_config_v2_size = 0;
       }

       AR_MSG(DBG_HIGH_PRIO, "Capi latency: Configured Global Delay = %lu us", delay_param_ptr->global_delay_us);

       // cache the payload
       me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr =
          (param_id_latency_cfg_v2_t *)posal_memory_malloc(sizeof(param_id_latency_cfg_v2_t),
                                                                  (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
       if (NULL == me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr)
       {
          AR_MSG(DBG_FATAL_PRIO,
                 "CAPI Latency: No memory to store cache_delay_config. Requires %lu bytes",
                 sizeof(param_id_latency_cfg_v2_t));
          return CAPI_ENOMEMORY;
       }
       me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr->num_config = 0;
       memscpy(me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr, sizeof(param_id_latency_cfg_v2_t), data_ptr, sizeof(param_id_latency_cfg_v2_t));
       me_ptr->cache_delay_v2.cache_delay_per_config_v2_size = sizeof(param_id_latency_cfg_v2_t); //per channel delay not included
    }
    else if (LATENCY_MODE_PER_CH == me_ptr->cfg_mode)
    {
       const uint32_t num_config = delay_param_ptr->num_config;
       uint32_t req_payload_size          = 0;
       uint32_t base_payload_size         = sizeof(param_id_latency_cfg_v2_t);
       uint32_t per_cfg_base_payload_size = sizeof(delay_param_per_ch_cfg_v2_t);
       capi_result                        = capi_latency_validate_multichannel_v2_payload(me_ptr,
                                                               data_ptr,
                                                               param_id,
                                                               param_size,
                                                               &req_payload_size,
                                                               base_payload_size,
                                                               per_cfg_base_payload_size);
       if (CAPI_FAILED(capi_result))
       {
          AR_MSG(DBG_ERROR_PRIO, "Latency V2 SetParam 0x%lx failed", param_id);
          return capi_result;
       }
       if (param_size < req_payload_size)
       {
          AR_MSG(DBG_ERROR_PRIO,
                  "Latency SetParam 0x%lx, invalid param size %lu ,required_size %lu",
                  param_id,
                  param_size,
                  req_payload_size);
          return CAPI_ENEEDMORE;
       } //till here edited
       uint8_t *temp_cfg_ptr = (uint8_t *)data_ptr;
       temp_cfg_ptr += sizeof(param_id_latency_cfg_v2_t);

       if (param_size < req_payload_size)
       {
          AR_MSG(DBG_ERROR_PRIO,
                 "CAPI Latency: per ch Delay SetParam 0x%lx, invalid param size %lu ,required_size %lu",
                 param_id,
                 param_size,
                 req_payload_size);
          return CAPI_EBADPARAM;
       }

       uint32_t offset   = 0;
       uint32_t *ch_mask_arr_ptr = NULL;
       uint32_t ch_mask_list_size = 0;
       for (uint32_t count = 0; count < num_config; count++)
       {
           temp_cfg_ptr += offset;

           delay_param_per_ch_cfg_v2_t *delay_cfg_ptr = (delay_param_per_ch_cfg_v2_t *)temp_cfg_ptr;
           uint32_t delay_us = 0;
           uint32_t channel_group_mask = delay_cfg_ptr->channel_type_group_mask;

           ch_mask_arr_ptr     = (uint32_t *)(temp_cfg_ptr + CAPI_CMN_INT32_SIZE_IN_BYTES); // points to mask array
           ch_mask_list_size   = capi_cmn_count_set_bits(channel_group_mask);
           delay_us = *((uint32_t*)(ch_mask_arr_ptr + ch_mask_list_size));
           AR_MSG(DBG_MED_PRIO,
                   "CAPI Latency Set Delay %lu for config %lu.", delay_us, count);
          if (delay_us > CAPI_LATENCY_MAX_DELAY_US)
          {
             AR_MSG(DBG_ERROR_PRIO,
                    "CAPI Latency Set Delay %lu too high, max is %lu",
                    delay_us,
                    CAPI_LATENCY_MAX_DELAY_US);
             return CAPI_EBADPARAM;
          }
          offset = sizeof(delay_param_per_ch_cfg_v2_t) + (ch_mask_list_size << 2);
       }

       if (NULL != me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr)
       {
          posal_memory_free(me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr);
          me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr  = NULL;
          me_ptr->cache_delay_v2.cache_delay_per_config_v2_size = 0;
       }

       // cache the payload
       //me_ptr->cache_delay.num_config             = num_config;
       me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr = (param_id_latency_cfg_v2_t *)
          posal_memory_malloc(req_payload_size,
                              (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);

       if (NULL == me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr)
       {
          AR_MSG(DBG_FATAL_PRIO,
                 "CAPI Latency: No memory to store cache_delay_per_config. Requires %lu bytes",
                 req_payload_size);
          return CAPI_ENOMEMORY;
       }
       memscpy(me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr, req_payload_size, data_ptr, req_payload_size);
       me_ptr->cache_delay_v2.cache_delay_per_config_v2_size = req_payload_size;
       AR_MSG(DBG_HIGH_PRIO, "Capi latency: Received per ch Delay, num_config = %lu, size = %lu", num_config, req_payload_size);
    }
    me_ptr->cfg_version = VERSION_V2;
    if (me_ptr->is_media_fmt_received == FALSE)
    {
       return capi_result;
    }

    capi_delay_delayline_t *old_delay_lines = NULL;
    void *                  old_mem_ptr     = NULL;

    uint32_t *old_delay_in_us =
       (uint32_t *)posal_memory_malloc(me_ptr->media_fmt.format.num_channels * sizeof(uint32_t),
                                       (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
    if (NULL == old_delay_in_us)
    {
       AR_MSG(DBG_ERROR_PRIO,
              "CAPI Latency: No memory to store old delay.Requires %lu bytes",
              me_ptr->media_fmt.format.num_channels * sizeof(uint32_t));
       CAPI_SET_ERROR(capi_result, CAPI_ENOMEMORY);
       goto __bailout;
    }

    for (uint32_t i = 0; i < me_ptr->media_fmt.format.num_channels; i++)
    {
       old_delay_in_us[i]                             = me_ptr->lib_config.mchan_config[i].delay_in_us;
       me_ptr->lib_config.mchan_config[i].delay_in_us = 0;
    }

    capi_delay_set_delay_v2(me_ptr);

    old_delay_lines = (capi_delay_delayline_t *)posal_memory_malloc(me_ptr->media_fmt.format.num_channels *
                                                                       sizeof(capi_delay_delayline_t),
                                                                    (POSAL_HEAP_ID)me_ptr->heap_mem.heap_id);
    if (NULL == old_delay_lines)
    {
       AR_MSG(DBG_ERROR_PRIO, "CAPI Latency failed to allocate memory for old_delay_lines");
       CAPI_SET_ERROR(capi_result, CAPI_ENOMEMORY);
       goto __bailout;
    }
    memset(old_delay_lines, 0, sizeof(capi_delay_delayline_t));

    for (uint32_t i = 0; i < me_ptr->media_fmt.format.num_channels; i++)
    {
       memscpy(&old_delay_lines[i],
               sizeof(capi_delay_delayline_t),
               &me_ptr->lib_config.mchan_config[i].delay_line,
               sizeof(capi_delay_delayline_t));
    }
    old_mem_ptr = me_ptr->lib_config.mem_ptr;

    capi_result |= capi_delay_create_buffer(me_ptr, old_delay_in_us);
    if (CAPI_SUCCEEDED(capi_result))
    {
       for (uint32_t i = 0; i < me_ptr->media_fmt.format.num_channels; i++)
       {
          if (old_delay_in_us[i] != 0)
          {
             capi_delay_delayline_copy(&me_ptr->lib_config.mchan_config[i].delay_line, &old_delay_lines[i]);
          }
       }
    }
 __bailout:
    if (NULL != old_delay_lines)
    {
       posal_memory_free(old_delay_lines);
       old_delay_lines = NULL;
    }

    if (NULL != old_mem_ptr)
    {
       posal_memory_free(old_mem_ptr);
       old_mem_ptr = NULL;
    }

    if (NULL != old_delay_in_us)
    {
       posal_memory_free(old_delay_in_us);
       old_delay_in_us = NULL;
    }

   return CAPI_EOK;
}


capi_err_t capi_latency_validate_multichannel_v2_payload(capi_latency_t *me_ptr,
                                                     int8_t *    data_ptr,
                                                     uint32_t    param_id,
                                                     uint32_t    param_size,
                                                     uint32_t *  req_payload_size,
                                                     uint32_t    base_payload_size,
                                                     uint32_t    per_cfg_base_payload_size)
{
   //int8_t * temp_cfg_ptr = data_ptr;
   uint32_t capi_result      = CAPI_EOK;
   param_id_latency_cfg_v2_t *cfg_ptr = (param_id_latency_cfg_v2_t*)data_ptr;
   uint32_t num_cfg          = cfg_ptr->num_config; //get num_cfg from arguments

   AR_MSG(DBG_MED_PRIO, "Received num_config = %lu for PID = 0x%lx", num_cfg, param_id);

   if (num_cfg < 1 || num_cfg > PCM_MAX_CHANNEL_MAP_V2)
   {
      AR_MSG(DBG_ERROR_PRIO, "Received incorrect num_config parameter - %lu", num_cfg);
      return CAPI_EBADPARAM;
   }

   capi_result = capi_latency_validate_per_channel_v2_payload(me_ptr,
                                                          num_cfg,
                                                          data_ptr,
                                                          param_size,
                                                          req_payload_size,
                                                          param_id,
                                                          base_payload_size,
                                                          per_cfg_base_payload_size);
   if (CAPI_FAILED(capi_result))
   {
      AR_MSG(DBG_ERROR_PRIO, "Configuration SetParam 0x%lx failed.", param_id);
      return capi_result;
   }
   return CAPI_EOK;
}

capi_err_t capi_latency_validate_per_channel_v2_payload(capi_latency_t *me_ptr,
                                                    uint32_t    num_cfg,
                                                    int8_t *    data_ptr,
                                                    uint32_t    param_size,
                                                    uint32_t *  required_size_ptr,
                                                    uint32_t    param_id,
                                                    uint32_t    base_payload_size,
                                                    uint32_t    per_cfg_base_payload_size)
{
   capi_err_t capi_result         = CAPI_EOK;
   int8_t *  base_payload_ptr     = data_ptr; // points to the start of the payload
   uint32_t  config_size          = 0;

   *required_size_ptr += base_payload_size; // size of param_id_latency_cfg_v2_t
   base_payload_ptr += *required_size_ptr;  // size of param_id_latency_cfg_v2_t
   // configuration payload
   // _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _
   //| num_ | channel_type_group_mask | dynamic channel_type_mask_list[0] | payload1 |  _ _ _ _
   //|config| _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _  |_ _ _ _ _ |_ _ _ _ _ _
   //
   uint32_t miid = 1; //for latency//remove after miid addition
   for (uint32_t count = 0; count < num_cfg; count++)
   {
      if ((*required_size_ptr + per_cfg_base_payload_size) > param_size)
      {
         AR_MSG(
                 DBG_ERROR_PRIO,
                 "Failed while validating required size for config #%lu. Total config: %lu. Total size required till this point is %lu. "
                 "Param size received %lu",
                 count,
                 num_cfg,
                 (*required_size_ptr + per_cfg_base_payload_size),
                 param_size);
         return CAPI_ENEEDMORE;
      }
      else
      {
         uint32_t ch_type_group_mask = 0;
         base_payload_ptr            += config_size;
         ch_type_group_mask          = *((uint32_t *)base_payload_ptr);
#ifdef CAPI_LATENCY_DBG_MSG
         AR_MSG(
                 DBG_LOW_PRIO,
                 "Channel group mask %#lx received for param 0x%#x for config %lu.",
                 ch_type_group_mask,
                 param_id,
                 count);
#endif
         // check for group mask payload if only desired bits are set or not
         if (0 == (ch_type_group_mask >> CAPI_CMN_MAX_CHANNEL_MAP_GROUPS))
         {
            capi_result = capi_cmn_check_payload_validation(miid, ch_type_group_mask, per_cfg_base_payload_size,
                       count, param_size, &config_size, required_size_ptr);
            if(CAPI_FAILED(capi_result))
            {
               return capi_result;
            }
         }
         else
         {
            AR_MSG(
                    DBG_ERROR_PRIO,
                    "Invalid configuration of channel group mask: %#lx. More than maximum valid channel groups are "
                    "set, maximum valid channel groups: %lu.",
                    ch_type_group_mask,
                    CAPI_CMN_MAX_CHANNEL_MAP_GROUPS);
            return CAPI_EBADPARAM;
         }
      }
   }
   AR_MSG(DBG_MED_PRIO, "calculated size for param is %lu", *required_size_ptr);

   // validate the configs for duplication
   if (!capi_latency_check_multi_ch_channel_mask_v2_param(miid,
                                                 num_cfg,
                                                 param_id,
                                                 (int8_t *)data_ptr,
                                                 base_payload_size,
                                                 per_cfg_base_payload_size))
   {
      AR_MSG(DBG_ERROR_PRIO, "Trying to set different delay to same channel, returning");
      return CAPI_EBADPARAM;
   }
   return CAPI_EOK;
}

/* Returns FALSE if same channel is set to multiple times in different configs */
bool_t capi_latency_check_multi_ch_channel_mask_v2_param(uint32_t miid,
                                            uint32_t    num_config,
                                            uint32_t    param_id,
                                            int8_t *    param_ptr,
                                            uint32_t    base_payload_size,
                                            uint32_t    per_cfg_base_payload_size)
{
   uint32_t *temp_mask_list_ptr                               = NULL;
   int8_t *  data_ptr                                         = param_ptr; // points to the start of the payload
   uint32_t  check_channel_mask_arr[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS]   = { 0 };
   uint32_t  current_channel_mask_arr[CAPI_CMN_MAX_CHANNEL_MAP_GROUPS] = { 0 };
   bool_t    check                                            = TRUE;
   uint32_t  offset                                           = sizeof(param_id_latency_cfg_v2_t);

   for (uint32_t cnfg_cntr = 0; cnfg_cntr < num_config; cnfg_cntr++)
   {
      uint32_t channel_group_mask = 0;
      // Increment data ptr by calculated offset to point at next payload's group_mask
      data_ptr += offset;
      channel_group_mask          = *((uint32_t *)data_ptr);
#ifdef CAPI_LATENCY_DBG_MSG
      AR_MSG(DBG_LOW_PRIO, "channel_group_mask %#lx, offset %lu for config %lu", channel_group_mask, offset, cnfg_cntr);
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

void capi_delay_set_delay_v2(capi_latency_t *me_ptr)
{
   param_id_latency_cfg_v2_t * cfg_ptr = (param_id_latency_cfg_v2_t *)me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr;
   if(LATENCY_MODE_GLOBAL == cfg_ptr->cfg_mode)
   {
      for (uint32_t chan_num = 0; chan_num < me_ptr->media_fmt.format.num_channels; chan_num++)
      {
          me_ptr->lib_config.mchan_config[chan_num].delay_in_us = cfg_ptr->global_delay_us;
      }
   }
   else
   {
      for (uint32_t chan_num = 0; chan_num < me_ptr->media_fmt.format.num_channels; chan_num++)
      {
         int8_t * delay_cfg_ptr = (int8_t *)me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr;
         uint32_t offset           = sizeof(param_id_latency_cfg_v2_t);
         for (uint32_t count = 0; count < me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr->num_config; count++)
         {
            uint32_t *ch_mask_arr_ptr = NULL;
            uint32_t channel_type = me_ptr->media_fmt.channel_type[chan_num];
            uint32_t delay_us = 0;
            // uint32_t ch_mask_arr_index = 0;
            uint32_t channel_group_mask = 0;
            uint32_t ch_mask_list_size  = 0;
            // Increment data ptr by calculated offset to point at next payload's group_mask
            delay_cfg_ptr += offset;
            delay_param_per_ch_cfg_v2_t *temp_cfg_ptr = (delay_param_per_ch_cfg_v2_t *)delay_cfg_ptr;
            channel_group_mask                        = temp_cfg_ptr->channel_type_group_mask;
            ch_mask_arr_ptr          = (uint32_t *)(delay_cfg_ptr + CAPI_CMN_INT32_SIZE_IN_BYTES); // points to mask array
            ch_mask_list_size        = capi_cmn_count_set_bits(channel_group_mask);
            delay_us = *(ch_mask_arr_ptr + ch_mask_list_size);
#ifdef CAPI_LATENCY_DBG_MSG
            AR_MSG(DBG_LOW_PRIO,
                        "CAPI Latency Set Delay %lu for config %lu, channel group mask %#lx", delay_us, count, channel_group_mask);
#endif
            uint32_t curr_ch_mask    = (1 << CAPI_CMN_MOD_WITH_32(channel_type));
            uint32_t ch_index_grp_no = CAPI_CMN_DIVIDE_WITH_32(channel_type);
            // check if the group is configured. If yes, update ch_index_mask_to_log_ptr
            if (CAPI_CMN_IS_BIT_SET_AT_POS_IN_32B_VAL(channel_group_mask, ch_index_grp_no))
            {
               //get the index of this group amongst the set bits in group mask
               uint32_t set_index = capi_cmn_count_set_bits_in_lower_n_bits(channel_group_mask, ch_index_grp_no);
#ifdef CAPI_LATENCY_DBG_MSG
               AR_MSG(DBG_LOW_PRIO,
                    "curr_ch_mask %lu, temp_cfg_ptr[%lu]: %lu",
                    curr_ch_mask,
                    set_index,
                    ch_mask_arr_ptr[set_index]);
#endif
               if (curr_ch_mask & ch_mask_arr_ptr[set_index])
               {

                  me_ptr->lib_config.mchan_config[chan_num].delay_in_us = delay_us;
#ifdef CAPI_LATENCY_DBG_MSG
                  AR_MSG(DBG_LOW_PRIO,
                       "chan_num %lu,  channel_type: %lu, delay in us: %lu",
                       chan_num,
                       channel_type,
                       delay_us);
#endif
                  break;
               }
            }
            offset = sizeof(delay_param_per_ch_cfg_v2_t) + (ch_mask_list_size << 2);
         }
      }
   }

   capi_delay_calc_delay_in_samples(me_ptr);

   return;
}

capi_err_t capi_latency_get_config_v2(
        capi_latency_t   *me_ptr,
        capi_buf_t       *params_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   if (NULL != me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr)
   {
      if (params_ptr->max_data_len >= sizeof(param_id_latency_cfg_v2_t))
      {
         param_id_latency_cfg_v2_t *delay_param_ptr = (param_id_latency_cfg_v2_t *)(params_ptr->data_ptr);
         delay_param_ptr->cfg_mode                  = me_ptr->cfg_mode;

         if (LATENCY_MODE_GLOBAL == me_ptr->cfg_mode)
         {
            delay_param_ptr->num_config      = 0;
            delay_param_ptr->global_delay_us = me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr->global_delay_us;
            params_ptr->actual_data_len      = sizeof(param_id_latency_cfg_v2_t);
            CAPI_SET_ERROR(capi_result, CAPI_EOK);
         }
         else if (LATENCY_MODE_PER_CH == me_ptr->cfg_mode)
         {
            delay_param_ptr->num_config      = me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr->num_config;
            delay_param_ptr->global_delay_us = 0; // don't care
            uint8_t *temp_cfg_ptr            = (uint8_t *)params_ptr->data_ptr;
            temp_cfg_ptr += sizeof(param_id_latency_cfg_v2_t);

            uint32_t payload_size = me_ptr->cache_delay_v2.cache_delay_per_config_v2_size;

            if (params_ptr->max_data_len >= payload_size)
            {

               params_ptr->actual_data_len = 0;
               param_id_latency_cfg_v2_t *delay_cfg_ptr =
                  (param_id_latency_cfg_v2_t *)(params_ptr->data_ptr);
               param_id_latency_cfg_v2_t *cached_delay_cfg_ptr =
                  (param_id_latency_cfg_v2_t *)me_ptr->cache_delay_v2.cache_delay_per_config_v2_ptr;
               params_ptr->actual_data_len = memscpy(delay_cfg_ptr,
                                                   params_ptr->max_data_len,
                                                   cached_delay_cfg_ptr,
                                                   me_ptr->cache_delay_v2.cache_delay_per_config_v2_size);

               CAPI_SET_ERROR(capi_result, CAPI_EOK);
            }
            else
            {
               AR_MSG(DBG_ERROR_PRIO, "CAPI Latency Get Delay, Bad param size %lu", params_ptr->max_data_len);
               CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
            }
         }
      }
      else
      {
         AR_MSG(DBG_ERROR_PRIO, "CAPI Latency Get Delay, Bad param size %lu", params_ptr->max_data_len);
         CAPI_SET_ERROR(capi_result, CAPI_ENEEDMORE);
      }
   }
   else
   {
      AR_MSG(DBG_ERROR_PRIO, "Did not receive any set param for delay parameter");
      capi_result |= CAPI_EFAILED;
      params_ptr->actual_data_len = 0;
   }
   return CAPI_EOK;
}
