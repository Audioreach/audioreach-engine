/* =========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_soft_vol_island.cpp
 *
 * Implementation for soft_vol module
 */

#include "capi_soft_vol.h"
#include "capi_soft_vol_utils.h"

// Use to override AR_MSG with AR_MSG_ISLAND. Always include this after ar_msg.h
#ifdef AR_MSG_IN_ISLAND
#include "ar_msg_island_override.h"
#endif

//#define SOFT_VOL_DEBUG 1

#ifdef DO_SOFT_VOL_PROFILING
#include <q6sim_timer.h>
#endif

static capi_vtbl_t capi_soft_vol_vtbl = { capi_soft_vol_process,        capi_soft_vol_end,
                                                capi_soft_vol_set_param,      capi_soft_vol_get_param,
                                                capi_soft_vol_set_properties, capi_soft_vol_get_properties };

capi_vtbl_t *capi_soft_vol_get_vtbl()
{
   return &capi_soft_vol_vtbl;
}

capi_err_t capi_soft_vol_process(capi_t *            _pif,
                                              capi_stream_data_t *input[],
                                              capi_stream_data_t *output[])
{
   capi_err_t       result = CAPI_EOK;
   capi_soft_vol_t *me_ptr = (capi_soft_vol_t *)(_pif);

   POSAL_ASSERT(me_ptr);
   POSAL_ASSERT(input[0]);
   POSAL_ASSERT(output[0]);

   uint32_t num_in_samples;
   uint32_t max_out_buf_size_in_samples;
   uint32_t samples_to_process;
   uint32_t bytes_to_sample_conv_fac;
   uint32_t bytes_per_sample = me_ptr->SoftVolumeControlsLib.GetBytesPerSample();

   capi_buf_t *soft_vol_input  = input[0]->buf_ptr;
   capi_buf_t *soft_vol_output = output[0]->buf_ptr;

   bytes_to_sample_conv_fac = bytes_per_sample >> 1;

   num_in_samples              = soft_vol_input[0].actual_data_len >> bytes_to_sample_conv_fac;
   max_out_buf_size_in_samples = soft_vol_output[0].max_data_len >> bytes_to_sample_conv_fac;

   samples_to_process = num_in_samples < max_out_buf_size_in_samples ? num_in_samples : max_out_buf_size_in_samples;

   for (uint32_t i = 0; i < me_ptr->soft_vol_lib.numChannels; i++)
   {
      me_ptr->SoftVolumeControlsLib
         .Process(soft_vol_input[i].data_ptr,
                  soft_vol_output[i].data_ptr,
                  samples_to_process,
                  me_ptr->soft_vol_lib.pPerChannelData[me_ptr->soft_vol_lib.channelMapping[i]]);

      soft_vol_input[i].actual_data_len  = samples_to_process << bytes_to_sample_conv_fac;
      soft_vol_output[i].actual_data_len = samples_to_process << bytes_to_sample_conv_fac;
   }

   if (me_ptr->update_gain_over_imcl)
   {
      capi_soft_vol_send_gain_over_imcl(me_ptr);
      me_ptr->update_gain_over_imcl = FALSE;
   }

   output[0]->flags = input[0]->flags;
   if (input[0]->flags.is_timestamp_valid)
   {
      output[0]->timestamp = input[0]->timestamp;
   }
    AR_MSG(DBG_HIGH_PRIO,"sv_end");
   return result;
}
