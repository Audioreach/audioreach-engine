/**
@file capi_drc_utils.cpp

@brief CAPI V2 utility for DRC module.

*/

/*-----------------------------------------------------------------------
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
-----------------------------------------------------------------------*/

#include "capi_drc_utils.h"

// drc default parameters
static const drc_config_t drc_cfg_nb_t = {
   CHANNEL_NOT_LINKED,  // int16   stereoLinked
   0x1,                 // int16   downSampleLevel
   0x06F2,              // uint16  rmsTavUL16Q16
   0x1000,              // uint16  makeupGainUL16Q12
   0x0A28,              // int16   dnExpaThresholdL16Q7
   (int16_t)0xFF9A,     // int16   dnExpaSlopeL16Q8
   0x05828860,          // uint32  dnExpaAttackUL32Q31
   0x0D554E9B,          // uint32  dnExpaReleaseUL32Q31
   (int32_t)0xFD000000, // int32   dnExpaMinGainDBL32Q23
   0x4934,              // uint16  dnExpaHysterisisUL16Q14
   0x0A28,              // int16   upCompThresholdL16Q7
   0x02C90623,          // uint32  upCompAttackUL32Q31
   0x02C90623,          // uint32  upCompReleaseUL32Q31
   0x0,                 // uint16  upCompSlopeUL16Q16
   0x4934,              // uint16  upCompHysterisisUL16Q14
   0x1BA8,              // int16   dnCompThresholdL16Q7
   0xF333,              // uint16  dnCompSlopeUL16Q16
   0x19471064,          // uint32  dnCompAttackUL32Q31
   0x02C90623,          // uint32  dnCompReleaseUL32Q31
   0x4934               // uint16  dnCompHysterisisUL16Q14
};

void drc_lib_set_default_config(capi_drc_t *me_ptr)
{
   // Initialize drc static parameters with default values
   me_ptr->lib_static_cfg.data_width  = BITS_16;
   me_ptr->lib_static_cfg.num_channel = 1;
   me_ptr->lib_static_cfg.sample_rate = 8000;
   me_ptr->delay_us                   = 5000; // 5milleseconds.

   // default initializations
   me_ptr->b_enable = 0;           // By default DRC module disabled
   me_ptr->mode     = DRC_ENABLED; // DRC processing enabled; normal DRC processing

   memscpy(&(me_ptr->lib_cfg), sizeof(drc_config_t), &drc_cfg_nb_t, sizeof(drc_cfg_nb_t));
}

capi_err_t drc_lib_set_calib(capi_drc_t *me_ptr)
{
   capi_err_t result     = CAPI_EOK;
   DRC_RESULT lib_result = DRC_SUCCESS;

   result |= ((NULL == me_ptr) || (NULL == me_ptr->lib_handle.lib_mem_ptr)) ? CAPI_EFAILED : result;
   CHECK_THROW_ERROR(MIID_UNKNOWN, result, "lib uninitialized.")

   lib_result =
      drc_set_param(&me_ptr->lib_handle, DRC_PARAM_FEATURE_MODE, (int8_t *)&(me_ptr->mode), sizeof(drc_feature_mode_t));
   if (DRC_SUCCESS != lib_result)
   {
      DRC_MSG(me_ptr->miid, DBG_ERROR_PRIO, "lib feature mode set param failed. result %lu", lib_result);
      return CAPI_EFAILED;
   }

   lib_result =
      drc_set_param(&me_ptr->lib_handle, DRC_PARAM_CONFIG, (int8_t *)&(me_ptr->lib_cfg), sizeof(drc_config_t));
   if (DRC_SUCCESS != lib_result)
   {
      DRC_MSG(me_ptr->miid, DBG_ERROR_PRIO, "lib config set param failed. result %lu", lib_result);
      return CAPI_EFAILED;
   }

   return result;
}

capi_err_t drc_lib_alloc_init(capi_drc_t *me_ptr)
{
   capi_err_t result       = CAPI_EOK;
   DRC_RESULT lib_result   = DRC_SUCCESS;
   uint64_t   sample_delay = 0;

   result |= (NULL == me_ptr) ? CAPI_EFAILED : result;
   CHECK_THROW_ERROR(MIID_UNKNOWN, result, "null pointer.")

   if (me_ptr->input_media_fmt.format.sampling_rate == 0)
   {
      return CAPI_EOK;
   }

   sample_delay = ((uint64_t)me_ptr->delay_us * (uint64_t)me_ptr->input_media_fmt.format.sampling_rate) / 1000000;

   me_ptr->lib_static_cfg.num_channel = me_ptr->input_media_fmt.format.num_channels;
   me_ptr->lib_static_cfg.sample_rate = me_ptr->input_media_fmt.format.sampling_rate;
   me_ptr->lib_static_cfg.data_width  = (16 == me_ptr->input_media_fmt.format.bits_per_sample) ? BITS_16 : BITS_32;
   me_ptr->lib_static_cfg.delay       = sample_delay;

   // cache allocated memory size
   uint32_t old_mem_size = me_ptr->mem_req.lib_mem_size;

   // Query memory requirement from drc library for new static parameters
   lib_result = drc_get_mem_req(&me_ptr->mem_req, &me_ptr->lib_static_cfg);
   if (DRC_SUCCESS != lib_result)
   {
      DRC_MSG(me_ptr->miid, DBG_ERROR_PRIO, "drc_get_mem_req failed, result 0x%lu", lib_result);
      return CAPI_EFAILED;
   }

   if (old_mem_size != me_ptr->mem_req.lib_mem_size)
   {
      // free current memory
      if (NULL != me_ptr->lib_handle.lib_mem_ptr)
      {
         posal_memory_free(me_ptr->lib_handle.lib_mem_ptr);
         me_ptr->lib_handle.lib_mem_ptr = NULL;
      }
      // allocate new memory
      int8_t *lib_mem_ptr = (int8_t *)posal_memory_malloc(me_ptr->mem_req.lib_mem_size, me_ptr->heap_id);
      if (NULL == lib_mem_ptr)
      {
         me_ptr->mem_req.lib_mem_size = 0;
         CHECK_THROW_ERROR(me_ptr->miid,
                           CAPI_ENOMEMORY,
                           "memory allocation failed. bytes %lu",
                           me_ptr->mem_req.lib_mem_size);
      }

      // Initialize drc library pointers
      lib_result =
         drc_init_memory(&me_ptr->lib_handle, &me_ptr->lib_static_cfg, lib_mem_ptr, me_ptr->mem_req.lib_mem_size);

      if (DRC_SUCCESS != lib_result)
      {
         DRC_MSG(me_ptr->miid, DBG_ERROR_PRIO, "drc_init_memory failed, result 0x%x", lib_result);

         posal_memory_free(lib_mem_ptr);
         me_ptr->lib_handle.lib_mem_ptr = NULL;
         me_ptr->mem_req.lib_mem_size   = 0;
         return CAPI_EFAILED;
      }

      DRC_MSG(me_ptr->miid, DBG_LOW_PRIO, "drc lib init done.");

      // Initialize DRC calibration parameters with default/cached values.
      result = drc_lib_set_calib(me_ptr);
   }
   return result;
}

static uint32_t drc_get_kpps(capi_drc_t *me_ptr)
{
   uint32_t kpps = 0;

   uint32_t sample_rate      = me_ptr->lib_static_cfg.sample_rate;
   uint32_t num_channels     = me_ptr->lib_static_cfg.num_channel;
   uint32_t linked_mode      = me_ptr->lib_cfg.channelLinked;
   uint32_t downsample_level = me_ptr->lib_cfg.downSampleLevel;

   if (me_ptr->mode == DRC_BYPASSED)
   {
      // 3 packets per sample.
      kpps = (sample_rate * num_channels * 3) / 1000;
   }
   else
   {
      /*
       * Linked Mode:
       * kpps = (channels * 0.01 + (0.04/downsample_level))*sample_rate;
       * = (sample_rate*channels*1 + 4*sample_rate/downsample_level) / 100;
       *
       * Un Linked Mode:
       * kpps = channels * (0.01 + (0.04/downsample_level)) *sample_rate;
       * = (sample_rate*channels*1 + 4*sample_rate*channels/downsample_level) /
       * 100;
       */
      if (linked_mode)
      {
         kpps = (sample_rate * num_channels + (4 * sample_rate) / downsample_level) / 100;
      }
      else
      {
         kpps = (sample_rate * num_channels + (4 * sample_rate * num_channels) / downsample_level) / 100;
      }
   }
   return kpps;
}

static uint32_t drc_get_bw(capi_drc_t *me_ptr)
{
   uint32_t bw = 0;

   uint32_t sample_rate  = me_ptr->lib_static_cfg.sample_rate;
   uint32_t num_channels = me_ptr->lib_static_cfg.num_channel;

   if (me_ptr->mode == DRC_BYPASSED)
   {
      bw = 2 * sample_rate * num_channels;
   }
   else
   {
      bw = 10 * sample_rate * num_channels;
   }
   return bw;
}

void raise_kpps_delay_process_events(capi_drc_t *me_ptr)
{
   if (me_ptr->b_enable)
   {
      uint32_t new_kpps = 0, new_bw = 0;

      new_kpps = drc_get_kpps(me_ptr);

      new_bw = drc_get_bw(me_ptr);

      // raise kpps event
      capi_cmn_update_kpps_event(&me_ptr->cb_info, new_kpps);

      // raise algo delay event
      capi_cmn_update_algo_delay_event(&me_ptr->cb_info, me_ptr->delay_us);

      capi_cmn_update_bandwidth_event(&me_ptr->cb_info, 0, new_bw);
   }

   // raise process check event
   capi_cmn_update_process_check_event(&me_ptr->cb_info,
                                       ((me_ptr->b_enable) && (NULL != me_ptr->lib_handle.lib_mem_ptr)) ? TRUE : FALSE);
}

capi_err_t drc_lib_send_config_imcl(capi_drc_t *me_ptr)
{
   capi_err_t        result          = CAPI_EOK;
   uint32_t          control_port_id = 0;
   ctrl_port_data_t *port_data_ptr   = NULL;
   capi_buf_t        buf;
   buf.actual_data_len = sizeof(imc_param_header_t) + sizeof(param_id_imcl_drc_down_comp_threshold_t);
   buf.data_ptr        = NULL;
   buf.max_data_len    = buf.actual_data_len;
   imcl_outgoing_data_flag_t flags;
   flags.should_send = TRUE;
   flags.is_trigger  = FALSE;

   // Get the control port id for the intent #INTENT_ID_AVC
   capi_cmn_ctrl_port_list_get_next_port_data(&me_ptr->ctrl_port_info,
                                              INTENT_ID_DRC_CONFIG,
                                              control_port_id, // initially, an invalid port id
                                              &port_data_ptr);

   if (port_data_ptr == NULL || port_data_ptr->state != CTRL_PORT_PEER_CONNECTED)
   {
      return CAPI_EOK;
   }

   control_port_id = port_data_ptr->port_info.port_id;

   // Get one time buf from the queue.
   result |= capi_cmn_imcl_get_one_time_buf(&me_ptr->cb_info, control_port_id, buf.actual_data_len, &buf);

   if (CAPI_FAILED(result) || (NULL == buf.data_ptr))
   {
      CHECK_THROW_ERROR(me_ptr->miid, CAPI_EFAILED, "Getting one time imcl buffer failed");
   }

   imc_param_header_t *out_cfg_ptr = (imc_param_header_t *)buf.data_ptr;

   out_cfg_ptr->opcode          = PARAM_ID_IMCL_DRC_DOWN_COMP_THRESHOLD;
   out_cfg_ptr->actual_data_len = sizeof(param_id_imcl_drc_down_comp_threshold_t);

   param_id_imcl_drc_down_comp_threshold_t *payload_ptr =
      (param_id_imcl_drc_down_comp_threshold_t *)(buf.data_ptr + sizeof(imc_param_header_t));
   payload_ptr->drc_rx_enable             = (me_ptr->b_enable) ? TRUE : FALSE;
   payload_ptr->down_comp_threshold_L16Q7 = me_ptr->lib_cfg.dnCompThresholdL16Q7;

   // Send data ready to the peer module
   result = capi_cmn_imcl_send_to_peer(&me_ptr->cb_info, &buf, control_port_id, flags);
   CHECK_THROW_ERROR(me_ptr->miid, result, "failed in sending imcl buffer to peer module.");

   return result;
}
