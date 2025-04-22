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
   Re-mapps the main buffer data pointer to the remap-buffer by using proper media fmt
   Also, calculated the proper length and separation while remapping output buffer if there is any byte-scale involved
______________________________________________________________________________________________________________________*/
void pc_remap_from_main_buf(capi_buf_t *    main_buf_ptr,
                            capi_buf_t *    remapped_buf_ptr,
                            capi_buf_t *    remapped_input_ref_buf_ptr,
                            pc_media_fmt_t *media_fmt_ptr,
                            pc_media_fmt_t *ref_input_media_fmt_ptr,
                            bool_t          is_input_side,
                            bool_t          is_in_or_out_buf)
{
   uint32_t ch_spacing        = 0;
   uint32_t alignement_offset = 0;

   if (PC_INTERLEAVED == media_fmt_ptr->interleaving)
   {
      remapped_buf_ptr->data_ptr        = main_buf_ptr->data_ptr;
      remapped_buf_ptr->actual_data_len = main_buf_ptr->actual_data_len;
      remapped_buf_ptr->max_data_len    = main_buf_ptr->max_data_len;
   }
   else if (PC_DEINTERLEAVED_PACKED == media_fmt_ptr->interleaving)
   {
      if (is_input_side)
      {
         ch_spacing = main_buf_ptr->actual_data_len / media_fmt_ptr->num_channels;
      }
      else
      {
         if (PC_INTERLEAVED == ref_input_media_fmt_ptr->interleaving)
         {
            ch_spacing = (remapped_input_ref_buf_ptr->actual_data_len * (media_fmt_ptr->word_size >> 3) *
                          (media_fmt_ptr->num_channels)) /
                         ((ref_input_media_fmt_ptr->word_size >> 3) * (ref_input_media_fmt_ptr->num_channels));
         }
         else
         {
            ch_spacing = remapped_input_ref_buf_ptr->actual_data_len * (media_fmt_ptr->word_size >> 3) /
                         (ref_input_media_fmt_ptr->word_size >> 3);
         }
      }
      alignement_offset = ch_spacing % 4;
      ch_spacing        = ch_spacing - alignement_offset;

      for (uint32_t ch = 0; ch < media_fmt_ptr->num_channels; ch++)
      {
         remapped_buf_ptr[ch].data_ptr = main_buf_ptr->data_ptr + (ch_spacing * ch);
      }
      remapped_buf_ptr[0].actual_data_len = is_input_side ? ch_spacing : 0;
      remapped_buf_ptr[0].max_data_len    = ch_spacing;
   }
   else if (PC_DEINTERLEAVED_UNPACKED_V2 == media_fmt_ptr->interleaving)
   {
      if (is_in_or_out_buf)
      {
         for (uint32_t ch = 0; ch < media_fmt_ptr->num_channels; ch++)
         {
            remapped_buf_ptr[ch].data_ptr = main_buf_ptr[ch].data_ptr;
         }
         remapped_buf_ptr[0].actual_data_len = main_buf_ptr[0].actual_data_len;
         remapped_buf_ptr[0].max_data_len    = main_buf_ptr[0].max_data_len;
      }
      else
      {
         ch_spacing = is_input_side ? main_buf_ptr->actual_data_len / media_fmt_ptr->num_channels
                                    : main_buf_ptr->max_data_len / media_fmt_ptr->num_channels;

         alignement_offset = ch_spacing % 4;
         ch_spacing        = ch_spacing - alignement_offset;
         for (uint32_t ch = 0; ch < media_fmt_ptr->num_channels; ch++)
         {
            remapped_buf_ptr[ch].data_ptr = main_buf_ptr->data_ptr + (ch_spacing * ch);
         }
         remapped_buf_ptr[0].actual_data_len = is_input_side ? ch_spacing : 0;
         remapped_buf_ptr[0].max_data_len    = ch_spacing;
      }
   }
}
/*______________________________________________________________________________________________________________________
   DESCRIPTION:
   1. Finds out which actual buffer (process input, scratch buffer 1, scratch buffer 2 or output)
      to use to remap from the remapped buffer pointer
   2. Similarly updated needed media fmt and a flag to indicate if the main buffer is from framework or if it scratch
______________________________________________________________________________________________________________________*/
void pc_remap_buffer(pc_lib_t *      pc_ptr,
                     capi_buf_t *    input_buf_ptr,
                     capi_buf_t *    output_buf_ptr,
                     capi_buf_t *    scratch_buf_ptr_1,
                     capi_buf_t *    scratch_buf_ptr_2,
                     capi_buf_t *    remapped_ib_ptr,
                     capi_buf_t *    remapped_ob_ptr,
                     pc_media_fmt_t *input_media_fmt_ptr,
                     pc_media_fmt_t *output_media_fmt_ptr,
                     bool_t          is_input)
{
   capi_buf_t *    main_buf_ptr     = NULL;
   bool_t          is_in_out_buf    = FALSE;
   capi_buf_t *    remapped_buf_ptr = NULL;
   pc_media_fmt_t *media_fmt_ptr    = NULL;
   if (is_input)
   {
      remapped_buf_ptr = remapped_ib_ptr;
      media_fmt_ptr    = input_media_fmt_ptr;
   }
   else
   {
      remapped_buf_ptr = remapped_ob_ptr;
      media_fmt_ptr    = output_media_fmt_ptr;
   }

   if (pc_ptr->core_lib.remap_input_buf_ptr == remapped_buf_ptr)
   {
      is_in_out_buf = TRUE;
      main_buf_ptr  = input_buf_ptr;
   }
   else if (pc_ptr->core_lib.remap_scratch_buf1_ptr == remapped_buf_ptr)
   {
      main_buf_ptr = scratch_buf_ptr_1;
   }
   else if (pc_ptr->core_lib.remap_scratch_buf2_ptr == remapped_buf_ptr)
   {
      main_buf_ptr = scratch_buf_ptr_2;
   }
   else if (pc_ptr->core_lib.remap_output_buf_ptr == remapped_buf_ptr)
   {
      is_in_out_buf = TRUE;
      main_buf_ptr  = output_buf_ptr;
   }
   else
   {
      CNV_MSG(pc_ptr->miid, DBG_ERROR_PRIO, "WARNING! BUFFER NOT MATCHED");
      return;
   }

   pc_remap_from_main_buf(main_buf_ptr,
                          remapped_buf_ptr,
                          remapped_ib_ptr,
                          media_fmt_ptr,
                          input_media_fmt_ptr,
                          is_input,
                          is_in_out_buf);
}

/*______________________________________________________________________________________________________________________
   DESCRIPTION:

______________________________________________________________________________________________________________________*/
ar_result_t pc_interleaving_process(void           *me_ptr,
                                    capi_buf_t     *input_buf_ptr,
                                    capi_buf_t     *output_buf_ptr,
                                    pc_media_fmt_t *input_media_fmt_ptr,
                                    pc_media_fmt_t *output_media_fmt_ptr)
{
   ar_result_t result = AR_EOK;

   if (PC_INTERLEAVED == input_media_fmt_ptr->interleaving)
   {
      uint32_t num_samp_per_ch =
         input_buf_ptr->actual_data_len / (input_media_fmt_ptr->num_channels * input_media_fmt_ptr->word_size >> 3);
      uint32_t bytes_per_samp = input_media_fmt_ptr->word_size >> 3;

      result = spf_intlv_to_deintlv_unpacked_v2(input_buf_ptr,
                                                output_buf_ptr,
                                                input_media_fmt_ptr->num_channels,
                                                bytes_per_samp,
                                                num_samp_per_ch);
   }
   else
   {
      result = spf_deintlv_to_intlv(input_buf_ptr,
                                    output_buf_ptr,
                                    input_media_fmt_ptr->num_channels,
                                    input_media_fmt_ptr->word_size);
   }

   return result;
}

/*______________________________________________________________________________________________________________________
   DESCRIPTION:

______________________________________________________________________________________________________________________*/
ar_result_t pc_byte_morph_process(void *          me_ptr,
                                  capi_buf_t *    input_buf_ptr,
                                  capi_buf_t *    output_buf_ptr,
                                  pc_media_fmt_t *input_media_fmt_ptr,
                                  pc_media_fmt_t *output_media_fmt_ptr)
{
   ar_result_t result = AR_EOK;
   pc_lib_t *  pc_ptr = (pc_lib_t *)me_ptr;
   if (PC_INTERLEAVED == input_media_fmt_ptr->interleaving)
   {
      if (PC_BW16_W16_Q15 == output_media_fmt_ptr->byte_combo)
      {
         result = pc_intlv_16_out(input_buf_ptr,
                                  output_buf_ptr,
                                  input_media_fmt_ptr->word_size,
                                  input_media_fmt_ptr->q_factor);
      }
      else if (PC_BW24_W24_Q23 == output_media_fmt_ptr->byte_combo)
      {
         result = pc_intlv_24_out(input_buf_ptr,
                                  output_buf_ptr,
                                  input_media_fmt_ptr->word_size,
                                  input_media_fmt_ptr->q_factor);
      }
      else if ((PC_BW24_W32_Q23 == output_media_fmt_ptr->byte_combo) ||
               (PC_BW24_W32_Q27 == output_media_fmt_ptr->byte_combo) ||
               (PC_BW24_W32_Q31 == output_media_fmt_ptr->byte_combo) ||
               (PC_BW32_W32_Q31 == output_media_fmt_ptr->byte_combo) ||
               (PC_BW32_W32_FLOAT == output_media_fmt_ptr->byte_combo) ||
               (PC_BW64_W64_DOUBLE == output_media_fmt_ptr->byte_combo))
      {
         result = pc_intlv_32_out(input_buf_ptr,
                                  output_buf_ptr,
                                  input_media_fmt_ptr->word_size,
                                  input_media_fmt_ptr->q_factor,
                                  output_media_fmt_ptr->q_factor);
      }
      else
      {
         CNV_MSG(pc_ptr->miid, DBG_ERROR_PRIO, "Unsupported byte combo ");
      }
   }
   else
   {
      if (PC_BW16_W16_Q15 == output_media_fmt_ptr->byte_combo)
      {
         result = pc_deintlv_unpacked_v2_16_out(input_buf_ptr,
                                                output_buf_ptr,
                                                input_media_fmt_ptr->num_channels,
                                                input_media_fmt_ptr->word_size,
                                                input_media_fmt_ptr->q_factor);
      }
      else if (PC_BW24_W24_Q23 == output_media_fmt_ptr->byte_combo)
      {
         result = pc_deintlv_unpacked_v2_24_out(input_buf_ptr,
                                                output_buf_ptr,
                                                input_media_fmt_ptr->num_channels,
                                                input_media_fmt_ptr->word_size,
                                                input_media_fmt_ptr->q_factor);
      }
      else if ((PC_BW24_W32_Q23 == output_media_fmt_ptr->byte_combo) ||
               (PC_BW24_W32_Q27 == output_media_fmt_ptr->byte_combo) ||
               (PC_BW24_W32_Q31 == output_media_fmt_ptr->byte_combo) ||
               (PC_BW32_W32_Q31 == output_media_fmt_ptr->byte_combo) ||
               (PC_BW32_W32_FLOAT == output_media_fmt_ptr->byte_combo) ||
               (PC_BW64_W64_DOUBLE == output_media_fmt_ptr->byte_combo))
      {
         result = pc_deintlv_unpacked_v2_32_out(input_buf_ptr,
                                                output_buf_ptr,
                                                input_media_fmt_ptr->num_channels,
                                                input_media_fmt_ptr->word_size,
                                                input_media_fmt_ptr->q_factor,
                                                output_media_fmt_ptr->q_factor);
      }
      else
      {
         CNV_MSG(pc_ptr->miid, DBG_ERROR_PRIO, "Unsupported byte combo ");
      }
   }

   return result;
}
/*______________________________________________________________________________________________________________________
   DESCRIPTION:

______________________________________________________________________________________________________________________*/
ar_result_t pc_channel_mix_process(void *          me_ptr,
                                   capi_buf_t *    input_buf_ptr,
                                   capi_buf_t *    output_buf_ptr,
                                   pc_media_fmt_t *input_media_fmt_ptr,
                                   pc_media_fmt_t *output_media_fmt_ptr)
{
   ar_result_t result                                     = AR_EOK;
   pc_lib_t *  pc_ptr                                     = (pc_lib_t *)me_ptr;
   void *      local_input_buf_ptr[CAPI_MAX_CHANNELS_V2]  = { NULL };
   void *      local_output_buf_ptr[CAPI_MAX_CHANNELS_V2] = { NULL };
   uint32_t    bytes_to_process                           = 0;
   uint32_t    samples_to_process                         = 0;
   uint32_t    byte_sample_convert                        = 0;
   for (uint32_t i = 0; i < input_media_fmt_ptr->num_channels; i++)
   {
      local_input_buf_ptr[i] = (void *)input_buf_ptr[i].data_ptr;
   }

   for (uint32_t ch = 0; ch < output_media_fmt_ptr->num_channels; ch++)
   {
      local_output_buf_ptr[ch] = (void *)output_buf_ptr[ch].data_ptr;
   }

   bytes_to_process    = input_buf_ptr[0].actual_data_len;
   byte_sample_convert = (input_media_fmt_ptr->word_size == 16) ? 1 : 2;
   samples_to_process  = bytes_to_process >> (byte_sample_convert);

   ChMixerProcess(pc_ptr->ch_lib_ptr, local_output_buf_ptr, local_input_buf_ptr, samples_to_process);
   // optimization: write/read only first ch lens, and assume same lens for rest of the chs
   input_buf_ptr[0].actual_data_len  = samples_to_process << byte_sample_convert;
   output_buf_ptr[0].actual_data_len = samples_to_process << byte_sample_convert;

   return result;
}

/*______________________________________________________________________________________________________________________
   DESCRIPTION:

______________________________________________________________________________________________________________________*/
ar_result_t pc_iir_resampler_process(void *          me_ptr,
                                     capi_buf_t *    input_buf_ptr,
                                     capi_buf_t *    output_buf_ptr,
                                     pc_media_fmt_t *input_media_fmt_ptr,
                                     pc_media_fmt_t *output_media_fmt_ptr)
{

   capi_err_t result = CAPI_EOK;
   pc_lib_t * pc_ptr = (pc_lib_t *)me_ptr;
   uint32_t num_input_samples, ch;
   uint32_t processed_input_samples, generated_output_samples;

   iir_rs_lib_instance_t *port_instance_ptr = &pc_ptr->iir_rs_lib_ptr->lib_instance_per_port_ptr[0];
   num_input_samples                        = input_buf_ptr->actual_data_len >> 1;
   // num_output_samples = (num_input_samples * output_media_fmt_ptr->sampling_rate) /
   // input_media_fmt_ptr->sampling_rate;

   int8_t *input_ptr[CAPI_MAX_CHANNELS_V2]  = { 0 };
   int8_t *output_ptr[CAPI_MAX_CHANNELS_V2] = { 0 };

   processed_input_samples = generated_output_samples = 0;
   while (num_input_samples > processed_input_samples)
   {
      for (ch = 0; ch < port_instance_ptr->lib_io_config.in_channels; ch++)
      {
         input_ptr[ch]  = input_buf_ptr[ch].data_ptr + (processed_input_samples << 1);
         output_ptr[ch] = output_buf_ptr[ch].data_ptr + (generated_output_samples << 1);
      }

      // resampler process takes case of memscpy in passthrough case with different input and output pointers.
      result = iir_rs_process(pc_ptr->iir_rs_lib_ptr,
                              input_ptr,
                              output_ptr,
                              pc_ptr->in_frame_samples_per_ch,
                              pc_ptr->out_frame_samples_per_ch);

      if (AR_EOK != result)
      {
         result = CAPI_EFAILED;
      }

      processed_input_samples += pc_ptr->in_frame_samples_per_ch;
      generated_output_samples += pc_ptr->out_frame_samples_per_ch; // amith - For now.
   }

   // optimization: write/read only first ch lens, and assume same lens for rest of the chs
   output_buf_ptr[0].actual_data_len = generated_output_samples << 1;
   return result;
}

/*______________________________________________________________________________________________________________________
   DESCRIPTION:

______________________________________________________________________________________________________________________*/
ar_result_t pc_calc_and_update_max_in_len(pc_lib_t *  pc_ptr,
                                          capi_buf_t *input_buf_ptr,
                                          capi_buf_t *output_buf_ptr,
                                          capi_buf_t *remapped_ib_ptr)
{
   ar_result_t     result                       = AR_EOK;
   uint32_t        in_len_per_ch                = 0;
   uint32_t        out_len_per_ch               = 0;
   uint32_t        max_in_len_to_consume_per_ch = 0;
   pc_media_fmt_t *imf_ptr                      = &pc_ptr->core_lib.input_media_fmt;
   pc_media_fmt_t *omf_ptr                      = &pc_ptr->core_lib.output_media_fmt;

   if ((PC_INTERLEAVED == imf_ptr->interleaving) || (PC_DEINTERLEAVED_PACKED == imf_ptr->interleaving))
   {
      in_len_per_ch = input_buf_ptr->actual_data_len / imf_ptr->num_channels;
   }
   else
   {
      in_len_per_ch = input_buf_ptr->actual_data_len;
   }

   if ((PC_INTERLEAVED == omf_ptr->interleaving) || (PC_DEINTERLEAVED_PACKED == omf_ptr->interleaving))
   {
      out_len_per_ch = output_buf_ptr->max_data_len / omf_ptr->num_channels;
   }
   else
   {
      out_len_per_ch = output_buf_ptr->max_data_len;
   }

   // get output samples based on output buffer
   uint32_t output_samples_per_ch = out_len_per_ch / (omf_ptr->word_size >> 3);

   // scale max required input samples to fill output max buffer length.
   uint32_t max_req_in_per_ch = in_len_per_ch;
   if ((pc_ptr->core_lib.flags.RESAMPLER_PRE || pc_ptr->core_lib.flags.RESAMPLER_POST))
   {
      uint32_t req_in_samples_per_ch = 0;
      // If IIR resampler is used, fixed input frame size is required. The following checks are needed:
      // 1. Input length meets frame size.
      // 2. Enough output was provided to fit the input.
      if (NULL != pc_ptr->iir_rs_lib_ptr)
      {

         if ((0 == pc_ptr->in_frame_samples_per_ch) || (0 == pc_ptr->out_frame_samples_per_ch))
         {
            CNV_MSG(pc_ptr->miid, DBG_ERROR_PRIO, "pc_process: Error: Frame size not set.");
            return AR_EFAILED;
         }

         uint32_t in_frame_size_bpc = pc_ptr->in_frame_samples_per_ch * (imf_ptr->word_size >> 3);

         // Check if input length meets frame size.
         if ((in_len_per_ch < in_frame_size_bpc) || (output_samples_per_ch < pc_ptr->out_frame_samples_per_ch))
         {
#ifdef PCM_CNV_DEBUG
            CNV_MSG(pc_ptr->miid,
                    DBG_ERROR_PRIO,
                    "pc_process: Input given: bytes per ch: %ld, output samples per ch %ld, frame size bytes per ch: "
                    "%ld",
                    in_len_per_ch,
                    output_samples_per_ch,
                    in_frame_size_bpc);
#endif
            return AR_ENEEDMORE;
         }
         req_in_samples_per_ch = pc_ptr->in_frame_samples_per_ch;

         // If more input was given, allow loops over iir resampler. IIR resampler pc process function allows for
         // looping.
         if (in_len_per_ch > in_frame_size_bpc)
         {

            uint32_t in_proc_multiplier  = in_len_per_ch / in_frame_size_bpc;
            uint32_t out_proc_multiplier = output_samples_per_ch / pc_ptr->out_frame_samples_per_ch;
            uint32_t proc_multiplier     = MIN(in_proc_multiplier, out_proc_multiplier);
            req_in_samples_per_ch *= proc_multiplier;
#ifdef PCM_CNV_DEBUG
            CNV_MSG(pc_ptr->miid,
                    DBG_HIGH_PRIO,
                    "pc_process: Input given: bytes per ch: %ld, output bytes per ch %ld, consuming frame size bytes "
                    "per ch: %ld, in proc mult %ld, out proc mult %ld",
                    in_len_per_ch,
                    out_len_per_ch,
                    req_in_samples_per_ch * (imf_ptr->word_size >> 3),
                    in_proc_multiplier,
                    out_proc_multiplier);
#endif

            // Revert lib frame size back to cntr frame size when no longer in consume_partial_input mode, and
            // we are able to process a full cntr frame size worth of data.
            if ((!pc_ptr->consume_partial_input) && (pc_ptr->frame_size_us != pc_ptr->cntr_frame_size_us))
            {
               uint64_t *FRACT_TIME_PTR_NULL = NULL;

               // To convert samples per ch to us, we can use bits_per_sample = 8, num_channels = 1.
               uint32_t input_size_us =
                  capi_cmn_bytes_to_us(req_in_samples_per_ch, imf_ptr->sampling_rate, 8, 1, FRACT_TIME_PTR_NULL);

               if (input_size_us == pc_ptr->cntr_frame_size_us)
               {
                  pc_set_frame_size(pc_ptr, pc_ptr->cntr_frame_size_us);
               }
            }
         }
      }
      else if (NULL != pc_ptr->hwsw_rs_lib_ptr) // either hw or sw resampler is used.
      {
         // For software resampler, use the function to get in samples from out
         if ((NULL != pc_ptr->hwsw_rs_lib_ptr->sw_rs_mem_ptr[STAGE_ZERO]) &&
             (pc_ptr->hwsw_rs_lib_ptr->sw_rs_mem_ptr[STAGE_ZERO]->drs_mem_ptr.pStructMem))
         {
            req_in_samples_per_ch = pc_get_fixed_out_samples(pc_ptr, output_samples_per_ch);
         }
         else // For hardware resampler use differnt function to get in samples to be consumed
         {
            uint32_t local_out_sample = output_samples_per_ch;
            hwsw_rs_lib_process_get_hw_process_info(pc_ptr->hwsw_rs_lib_ptr,
                                                    in_len_per_ch / (imf_ptr->word_size >> 3),
                                                    output_samples_per_ch,
                                                    &req_in_samples_per_ch,
                                                    &local_out_sample);
         }
      }
      else
      {
         CNV_MSG(pc_ptr->miid, DBG_ERROR_PRIO, "pc_process: WARNING: Neither IIR not hwsw resamplers were enabled");
         req_in_samples_per_ch = (output_samples_per_ch * imf_ptr->sampling_rate) / omf_ptr->sampling_rate;
      }
      max_req_in_per_ch = req_in_samples_per_ch * (imf_ptr->word_size >> 3);
   }
   else // for case where resampler is not present calculate max input length from max samples output can hold
   {
      max_req_in_per_ch = output_samples_per_ch * (imf_ptr->word_size >> 3);
   }

   // Update input buffer actual data legnth based on the len thats consumed by pc process,
   //   1. If INPUT actual data length is greater than required, update actual data length
   //      based one expected input samples.
   //   2. If INPUT actual data length is lesser than expected, this is an error case. Library
   //      will operate in fixed input mode and consume all the input, so no need to update
   //      input actual data length.
   //   3. If OUTPUT max length is not enough to hold the converted data from input,
   //      calculate the corresponding input length which should be consumed
   max_in_len_to_consume_per_ch = MIN(max_req_in_per_ch, in_len_per_ch);

#ifdef PCM_CNV_LIB_DEBUG
   CNV_MSG(pc_ptr->miid,
           DBG_HIGH_PRIO,
           "pc_process: max_in_len_to_consume_per_ch %lu  in_len_per_ch %lu  output_samples_per_ch %lu "
           "max_req_in_per_ch %lu",
           max_in_len_to_consume_per_ch,
           in_len_per_ch,
           output_samples_per_ch,
           max_req_in_per_ch);
#endif

   if (PC_INTERLEAVED == imf_ptr->interleaving || PC_DEINTERLEAVED_PACKED == imf_ptr->interleaving)
   {
      input_buf_ptr->actual_data_len = max_in_len_to_consume_per_ch * imf_ptr->num_channels;
   }
   else
   {
      input_buf_ptr[0].actual_data_len = max_in_len_to_consume_per_ch;
   }

   if (PC_INTERLEAVED == imf_ptr->interleaving)
   {
      remapped_ib_ptr->actual_data_len = input_buf_ptr->actual_data_len;
   }
   else
   {
      remapped_ib_ptr[0].actual_data_len = max_in_len_to_consume_per_ch;
   }

   return result;
}

/*______________________________________________________________________________________________________________________
   DESCRIPTION:

______________________________________________________________________________________________________________________*/
ar_result_t pc_process(pc_lib_t *  pc_ptr,
                       capi_buf_t *input_buf_ptr,
                       capi_buf_t *output_buf_ptr,
                       capi_buf_t *scratch_buf_ptr_1,
                       capi_buf_t *scratch_buf_ptr_2)
{
   ar_result_t     result                   = AR_EOK;
   capi_buf_t *    next_input_buf_ptr       = pc_ptr->core_lib.remap_input_buf_ptr;
   pc_media_fmt_t *next_input_media_fmt_ptr = &pc_ptr->core_lib.input_media_fmt;
   bool_t          any_process_done         = FALSE;



   pc_remap_buffer(pc_ptr,
                   input_buf_ptr,
                   output_buf_ptr,
                   scratch_buf_ptr_1,
                   scratch_buf_ptr_2,
                   next_input_buf_ptr,
                   NULL,
                   next_input_media_fmt_ptr,
                   NULL,
                   TRUE);

   if (AR_ENEEDMORE == pc_calc_and_update_max_in_len(pc_ptr, input_buf_ptr, output_buf_ptr, next_input_buf_ptr))
   {
      return AR_ENEEDMORE;
   }

   for (uint32_t i = 0; i < NUMBER_OF_PROCESS; i++)
   {      if (NULL != pc_ptr->core_lib.pc_proc_info[i].process)
      {
         any_process_done                       = TRUE;
         capi_buf_t *    current_input_buf_ptr  = next_input_buf_ptr;
         capi_buf_t *    current_output_buf_ptr = pc_ptr->core_lib.pc_proc_info[i].output_buffer_ptr;
         pc_media_fmt_t *output_media_fmt_ptr   = &pc_ptr->core_lib.pc_proc_info[i].output_media_fmt;
         pc_remap_buffer(pc_ptr,
                         input_buf_ptr,
                         output_buf_ptr,
                         scratch_buf_ptr_1,
                         scratch_buf_ptr_2,
                         current_input_buf_ptr,
                         current_output_buf_ptr,
                         next_input_media_fmt_ptr,
                         output_media_fmt_ptr,
                         FALSE);

         result = pc_ptr->core_lib.pc_proc_info[i].process((void *)pc_ptr,
                                                           current_input_buf_ptr,
                                                           current_output_buf_ptr,
                                                           next_input_media_fmt_ptr,
                                                           output_media_fmt_ptr);

         if (current_output_buf_ptr == pc_ptr->core_lib.remap_output_buf_ptr)
         {
            output_buf_ptr->actual_data_len = current_output_buf_ptr->actual_data_len;
         }

         next_input_media_fmt_ptr = &pc_ptr->core_lib.pc_proc_info[i].output_media_fmt;
         next_input_buf_ptr       = current_output_buf_ptr;
      }
   }
   if (FALSE == any_process_done)
   {

      if (pc_ptr->core_lib.input_media_fmt.interleaving == pc_ptr->core_lib.output_media_fmt.interleaving)
      {

         if ((PC_INTERLEAVED == pc_ptr->core_lib.input_media_fmt.interleaving) ||
             (PC_DEINTERLEAVED_PACKED == pc_ptr->core_lib.input_media_fmt.interleaving))
         {

            if (input_buf_ptr->data_ptr != output_buf_ptr->data_ptr)
            {
               output_buf_ptr->actual_data_len = memscpy(output_buf_ptr->data_ptr,
                                                         output_buf_ptr->max_data_len,
                                                         input_buf_ptr->data_ptr,
                                                         input_buf_ptr->actual_data_len);
            }
         }
         else
         {

            uint32_t num_bytes_to_copy = MIN(output_buf_ptr[0].max_data_len, input_buf_ptr[0].actual_data_len);
            for (int32_t ch = 0; ch < pc_ptr->core_lib.input_media_fmt.num_channels; ch++)
            {
               memscpy(output_buf_ptr[ch].data_ptr, num_bytes_to_copy, input_buf_ptr[ch].data_ptr, num_bytes_to_copy);
            }
            output_buf_ptr[0].actual_data_len = num_bytes_to_copy;
         }

      }
      else
      {

         uint32_t bytes_per_ch = 0;

         // Enters here only for DEINTERLEAVED_PACKED <-> DEINTERLEAVED_UNPACKED conversion
         // Note: Interleaved <-> deinterleaved packed/deinterleaved unpacked case doesn't happen
         // since the interleaver process will be triggered if that happens
         if (PCM_DEINTERLEAVED_PACKED == pc_ptr->core_lib.input_media_fmt.interleaving)
         {

            bytes_per_ch = (input_buf_ptr->actual_data_len) / (pc_ptr->core_lib.input_media_fmt.num_channels);
            uint32_t num_bytes_to_copy = MIN(bytes_per_ch, output_buf_ptr[0].max_data_len);
            for (int32_t ch = 0; ch < pc_ptr->core_lib.input_media_fmt.num_channels; ch++)
            {
               memscpy(output_buf_ptr[ch].data_ptr,
                       num_bytes_to_copy,
                       input_buf_ptr->data_ptr + (bytes_per_ch * ch),
                       num_bytes_to_copy);
            }
            output_buf_ptr[0].actual_data_len = num_bytes_to_copy;
         }
         else
         {

            // We can't use output buffer lengths because, actual length will be 0 and if we use max len, it will not be
            // packed
            // Since input is unpacked, first channels length will suffice for the bytes per ch
            bytes_per_ch                    = (input_buf_ptr[0].actual_data_len);
            output_buf_ptr->actual_data_len = 0;
            for (int32_t ch = 0; ch < pc_ptr->core_lib.input_media_fmt.num_channels; ch++)
            {
               output_buf_ptr->actual_data_len += memscpy(output_buf_ptr->data_ptr + (bytes_per_ch * ch),
                                                          output_buf_ptr->max_data_len,
                                                          input_buf_ptr[ch].data_ptr,
                                                          bytes_per_ch);
            }
         }

      }

   }


   return result;
}
