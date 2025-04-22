/*______________________________________________________________________________________________________________________

 file pc_process.cpp
This file contains functions for compression-decompression container

Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
______________________________________________________________________________________________________________________*/

/*______________________________________________________________________________________________________________________
INCLUDE FILES FOR MODULE
______________________________________________________________________________________________________________________*/

#include "pc_converter.h"

/*______________________________________________________________________________________________________________________
   DESCRIPTION:

______________________________________________________________________________________________________________________*/
ar_result_t pc_endianness_process(void           *me_ptr,
                                  capi_buf_t     *input_buf_ptr,
                                  capi_buf_t     *output_buf_ptr,
                                  pc_media_fmt_t *input_media_fmt_ptr,
                                  pc_media_fmt_t *output_media_fmt_ptr)
{
   ar_result_t result = AR_EOK;
   // No need to check for the same endianness, since it is done in the init phase
   if (PC_INTERLEAVED != input_media_fmt_ptr->interleaving)
   {


      uint32_t num_bytes_to_process = input_buf_ptr[0].actual_data_len;
      uint32_t num_bytes_processed  = 0;
      for (uint16_t ch = 0; ch < input_media_fmt_ptr->num_channels; ch++)
      {
         // Buffer length remains the same since we use the same buffer
         pc_change_endianness(input_buf_ptr[ch].data_ptr,
                              output_buf_ptr[ch].data_ptr,
                              num_bytes_to_process,
                              &num_bytes_processed,
                              input_media_fmt_ptr->word_size);
      }
      output_buf_ptr[0].actual_data_len = num_bytes_processed;
   }
   else
   {
      // Buffer length remains the same since we use the same buffer
      pc_change_endianness(input_buf_ptr->data_ptr,
                           output_buf_ptr->data_ptr,
                           input_buf_ptr->actual_data_len,
                           &output_buf_ptr->actual_data_len,
                           input_media_fmt_ptr->word_size);
   }

   return result;
}


/*______________________________________________________________________________________________________________________
   DESCRIPTION:

______________________________________________________________________________________________________________________*/
ar_result_t pc_dyn_resampler_process(void           *_pif,
                                     capi_buf_t     *input_buf_ptr,
                                     capi_buf_t     *output_buf_ptr,
                                     pc_media_fmt_t *input_media_fmt_ptr,
                                     pc_media_fmt_t *output_media_fmt_ptr)
{

   ar_result_t result  = AR_EOK;
   pc_lib_t   *me_ptr  = (pc_lib_t *)_pif;
   uint32_t    heap_id = me_ptr->heap_id;

   // TODO: Currently resampler doesnt support unpacked V2, need changes in multiple places in dyn resampler module,
   // mp2_dec, resampler library, hwrs driver.
   for (uint32_t i = 1; i < input_media_fmt_ptr->num_channels; i++)
   {
      input_buf_ptr[i].actual_data_len = input_buf_ptr[0].actual_data_len;
      input_buf_ptr[i].max_data_len    = input_buf_ptr[0].max_data_len;
   }

   for (uint32_t i = 1; i < output_media_fmt_ptr->num_channels; i++)
   {
      output_buf_ptr[i].actual_data_len = output_buf_ptr[0].actual_data_len;
      output_buf_ptr[i].max_data_len    = output_buf_ptr[0].max_data_len;
   }

   result = hwsw_rs_lib_process(me_ptr->hwsw_rs_lib_ptr,
                                input_buf_ptr,
                                output_buf_ptr,
                                input_media_fmt_ptr->num_channels,
                                output_media_fmt_ptr->num_channels,
                                heap_id);
   return result;
}

/*______________________________________________________________________________________________________________________
   DESCRIPTION:

______________________________________________________________________________________________________________________*/

