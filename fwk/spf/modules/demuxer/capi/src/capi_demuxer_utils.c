/**
 * \file capi_demuxer_utils.c
 * \brief
 *     Source file to implement utility functions called by the CAPI Interface for demuxer Module.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_demuxer_utils.h"
#include "ar_msg.h"
/*==========================================================================
  MACROS
========================================================================== */
#define DEMUXER_KPPS_MONO_UNDER_48K 40
#define DEMUXER_KPPS_MONO_48K 90
/*==========================================================================
  Function Definitions
========================================================================== */
capi_err_t capi_demuxer_update_and_raise_kpps_event(capi_demuxer_t *me_ptr)
{
   capi_err_t capi_result = CAPI_EOK;
   uint32_t   kpps        = 0;
   if (32000 >= me_ptr->operating_mf.format.sampling_rate)
   {
      kpps = DEMUXER_KPPS_MONO_UNDER_48K;
   }
   else
   {
      kpps = DEMUXER_KPPS_MONO_48K;
   }
   kpps *= me_ptr->operating_mf.format.num_channels;
   if (kpps != me_ptr->events_config.kpps)
   {
      me_ptr->events_config.kpps = kpps;
      capi_result = capi_cmn_update_kpps_event(&me_ptr->cb_info, me_ptr->events_config.kpps);
   }
   return capi_result;
}

void capi_demuxer_get_port_index(capi_demuxer_t *me_ptr, uint32_t port_id, uint32_t *port_index)
{

   for (uint32_t i = 0; i < me_ptr->num_out_ports; i++)
   {

      if (me_ptr->out_port_state_arr[i].port_id == port_id)
      {
         *port_index = i;
		 return;
      }
   }

   *port_index = me_ptr->num_out_ports;
   return;
}

// This function is called only if media format is recevied
capi_err_t capi_demuxer_validate_out_cfg_and_raise_out_mf(capi_demuxer_t *me_ptr, uint32_t port_index)
{
   capi_err_t capi_result           = CAPI_EOK;
   bool_t     channel_indices_valid = TRUE;

   if (me_ptr->out_port_state_arr[port_index].is_cfg_received)
   {
      // Validate channel indices
      for (uint32_t channel = 0; channel < me_ptr->out_port_state_arr[port_index].num_channels; channel++)
      {
         uint16_t channel_index = me_ptr->out_port_state_arr[port_index].input_channel_index[channel];
         if (channel_index >= me_ptr->operating_mf.format.num_channels)
         {
            channel_indices_valid = FALSE;
            break;
         }
      }

      if (channel_indices_valid)
      {
         me_ptr->out_port_state_arr[port_index].operating_out_mf.format.num_channels =
            me_ptr->out_port_state_arr[port_index].num_channels;

         // Update channel type based on input channel type
         for (uint32_t channel = 0; channel < me_ptr->out_port_state_arr[port_index].num_channels; channel++)
         {

            uint16_t output_channel_type = me_ptr->out_port_state_arr[port_index].output_channel_type[channel];
            me_ptr->out_port_state_arr[port_index].operating_out_mf.channel_type[channel] = output_channel_type;
         }
      }
      else
      {
         AR_MSG(DBG_HIGH_PRIO,
                "capi_demuxer: channel_indices not valid "
                "Ignoring set_param PARAM_ID_DEMUXER_OUT_CONFIG for port id %u ",
                me_ptr->out_port_state_arr[port_index].port_id);

         // We already copied input_channel_index from set_param
         // Since this is error case, copy default values so that process function
         // will copy input to output for this port

         memscpy(&me_ptr->out_port_state_arr[port_index].input_channel_index[0],
                 sizeof(me_ptr->out_port_state_arr[port_index].input_channel_index),
                 &default_channel_index[0],
                 sizeof(default_channel_index));
      }
   }
   else
   {
      // If cfg is not received for this port, send out mf same as in mf
      // input mf will be copied to operating_out_mf for all output ports
      // and later updated based on set_param
   }
   capi_result |= capi_cmn_output_media_fmt_event_v2(&me_ptr->cb_info,
                                                     &me_ptr->out_port_state_arr[port_index].operating_out_mf,
                                                     FALSE,
                                                     port_index);

   return capi_result;
}

capi_err_t capi_demuxer_search_cached_cfg_and_update_out_cfg(capi_demuxer_t *me_ptr,
                                                                 uint32_t            port_index,
                                                                 uint32_t            port_id)
{
   capi_err_t capi_result   = CAPI_EOK;
   bool_t     port_id_found = FALSE;

   uint32_t i = 0;
   for (i = 0; i < me_ptr->num_out_ports; i++)
   {
      if (me_ptr->cached_out_cfg_arr[i].output_port_id == port_id)
      {
         port_id_found = TRUE;
         break;
      }
   }

   if (port_id_found)
   {
      me_ptr->out_port_state_arr[port_index].num_channels = me_ptr->cached_out_cfg_arr[i].num_channels;
      memscpy(&me_ptr->out_port_state_arr[port_index].input_channel_index[0],
              sizeof(me_ptr->out_port_state_arr[port_index].input_channel_index),
              &me_ptr->cached_out_cfg_arr[i].input_channel_index[0],
              sizeof(me_ptr->cached_out_cfg_arr[i].input_channel_index));
	   memscpy(&me_ptr->out_port_state_arr[port_index].output_channel_type[0],
              sizeof(me_ptr->out_port_state_arr[port_index].output_channel_type),
              &me_ptr->cached_out_cfg_arr[i].output_channel_type[0],
              sizeof(me_ptr->cached_out_cfg_arr[i].output_channel_type));
      me_ptr->out_port_state_arr[port_index].is_cfg_received = TRUE;
   }
   else
   {
      // cfg related to this port is not found. set defaults
      AR_MSG(DBG_HIGH_PRIO,
             "capi_demuxer: Port ID %u not found in cached set param "
             "Input will be sent as output for out port id %u ",
             port_id,
             port_id);
   }

   return capi_result;
}

