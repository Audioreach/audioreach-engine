/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 * @file capi_pcm_mf_cnv_island.cpp
 *
 * Cpp source file to implement the Common Audio Post Processor Interface
 * PCM_CNV block
 */

#include "capi_pcm_mf_cnv_i.h"
#include "capi_pcm_mf_cnv_utils.h"
#define ALIGN4(o)         (((o)+3)&(~3))

static capi_vtbl_t vtbl = { capi_pcm_mf_cnv_process,        capi_pcm_mf_cnv_end,
                            capi_pcm_mf_cnv_set_param,      capi_pcm_mf_cnv_get_param,
                            capi_pcm_mf_cnv_set_properties, capi_pcm_mf_cnv_get_properties };

capi_vtbl_t *capi_pcm_mf_cnv_get_vtable()
{
   return &vtbl;
}

/*______________________________________________________________________________________________________________________
   DESCRIPTION:
   Check if the scratch buffer allocated had enough size to handle incoming data.
   LOGIC:
   max_data_len_per_ch will be set to 0 during init or library init (even if scratch buffer is allocated for enc/dec)
   At process time, src_buf_max_len_per_ch is calculated from input buffer
   If this is more than the stored value, scratch buffer allocation logic is called.

   Since scratch buffer allocation depends on the factors like ordering or the modules, calculation is a little complex.
   So we don't want it to be called at every process. To achieve this, we use the variable  me_ptr->max_data_len_per_ch
   Module reordering can only happen at capi_pcm_mf_cnv_lib_init, so this variable is reset to 0 there.
   In other situations, if local_max_data_len_per_ch lesser than stored, we don't have to realloc the scratch buffers
______________________________________________________________________________________________________________________*/
capi_err_t capi_pcm_mf_cnv_check_scratch_buffer(capi_pcm_mf_cnv_t *me_ptr,
                                                capi_buf_t *       src_buf_ptr,
                                                capi_buf_t *       dest_buf_ptr,
                                                uint32_t           port)
{
   capi_err_t capi_result            = CAPI_EOK;
   uint32_t   src_buf_max_len_per_ch = 0;

   if (CAPI_DEINTERLEAVED_UNPACKED_V2 == me_ptr->in_media_fmt[port].format.data_interleaving)
   {
      src_buf_max_len_per_ch = src_buf_ptr[port].max_data_len;
   }
   else
   {
      src_buf_max_len_per_ch = src_buf_ptr[port].max_data_len / me_ptr->in_media_fmt[port].format.num_channels;
   }
   
   src_buf_max_len_per_ch = ALIGN4(src_buf_max_len_per_ch);
   if (me_ptr->max_data_len_per_ch < src_buf_max_len_per_ch)
   {
      me_ptr->max_data_len_per_ch = src_buf_max_len_per_ch;
      capi_result                 = capi_pcm_mf_cnv_allocate_scratch_buffer(me_ptr, src_buf_max_len_per_ch);
      if (CAPI_EOK != capi_result)
      {
         CNV_MSG(me_ptr->miid, DBG_ERROR_PRIO, "[%u] Error in pcm convert process - %d", me_ptr->type, capi_result);
         capi_pcm_mf_cnv_reset_buffer_len(me_ptr, src_buf_ptr, dest_buf_ptr, port);
		 // Resetting the value of max_data_len_per_ch = 0 when scratch buffer allocation failed
		 me_ptr->max_data_len_per_ch = 0;
      }
   }
   return capi_result;
}

/*_____________________________________________________________________________________________________________________
   DESCRIPTION:
   PCM_CNV module Data Process function to process an input buffer and generates an output buffer.
   1. Checks for the buffer size vs threshold value in case of CAPI_PCM_ENCODER and CAPI_PCM_DECODER
   2. If there is EOS marker, partial data is handled even for threshold modules
   3. For non-threshold modules, buffer sizes are assumed to be correct, so no checking is done to save mips
   4. Scratch buffer sizes are checked against stored value in case an alloc or realloc is needed
  ____________________________________________________________________________________________________________________*/
capi_err_t capi_pcm_mf_cnv_process(capi_t *_pif, capi_stream_data_t *input[], capi_stream_data_t *output[])
{
   capi_err_t capi_result        = CAPI_EOK;
   uint32_t   actual_input_bytes = 0;
   bool_t     all_input_consumed = FALSE;
   uint32_t   port               = 0;

   bool_t is_output_present =
      ((output) && (output[port]) && (output[port]->buf_ptr) && (output[port]->buf_ptr->data_ptr));
   bool_t is_input_present = ((input) && (input[port]) && (input[port]->buf_ptr) && (input[port]->buf_ptr->data_ptr));

   if (is_input_present && (!is_output_present))
   {
      // mark input is unconsumed
      input[0]->buf_ptr->actual_data_len = 0;
   }

   if (!(is_output_present && is_input_present))
   {
      CNV_MSG(MIID_UNKNOWN, DBG_LOW_PRIO, "[%u] Cannot process: Output or input is NULL");
      return AR_EOK;
   }

   actual_input_bytes        = input[0]->buf_ptr->actual_data_len;
   capi_pcm_mf_cnv_t *me_ptr = (capi_pcm_mf_cnv_t *)_pif;

   capi_buf_t *src_buf_ptr  = input[port]->buf_ptr;
   capi_buf_t *dest_buf_ptr = output[port]->buf_ptr;

   /*We will call process for pcm decoder and pcm enc eventhough lib is disabled and no memory is allocated
   This is because pcm dec and enc need to raise threshold and cannot be disabled
   In this case, memcpy and return without accessing lib memory */
   if (me_ptr->lib_enable == FALSE)
   {
      bool_t is_inplace = (input[0]->buf_ptr[0].data_ptr == output[0]->buf_ptr[0].data_ptr);
      // even if processing is not done, input data needs to be copied to output & output actual len be set.
      uint32_t out_max_data_len = output[0]->buf_ptr[0].max_data_len;
      uint32_t num_bytes_to_cpy = MIN(input[0]->buf_ptr[0].actual_data_len, out_max_data_len);
      for (uint32_t i = 0; i < input[0]->bufs_num; i++)
      {
       if (!is_inplace)
       {
          memscpy(output[0]->buf_ptr[i].data_ptr, out_max_data_len, input[0]->buf_ptr[i].data_ptr, num_bytes_to_cpy);
       }
      }

      // first update only first ch len's since module only supports CAPI_DEINTERLEAVED_UNPACKED_V2
      output[0]->buf_ptr[0].actual_data_len = num_bytes_to_cpy;
      input[0]->buf_ptr[0].actual_data_len  = num_bytes_to_cpy;

      /** return here if library is not enabled*/
      return CAPI_EOK;
   }

   capi_result = capi_pcm_mf_cnv_check_scratch_buffer(me_ptr, src_buf_ptr, dest_buf_ptr, port);

   if (CAPI_EOK != capi_result)
   {
      return capi_result;
   }

   // if configured for fixed output, but input samples differ from expected number, reset values
   // if input actual len is more than expected module needs to consume only whats expected.
   // if input is less than expected, module operates in fixed input mode.
   if ((CAPI_MFC == me_ptr->type) && (FIR_RESAMPLER == me_ptr->pc[0].resampler_type) &&
       capi_pcm_mf_cnv_is_dm_enabled(me_ptr) && (FWK_EXTN_DM_FIXED_OUTPUT_MODE == me_ptr->dm_info.dm_mode))
   {
      uint32_t in_samples = src_buf_ptr->actual_data_len / (me_ptr->in_media_fmt[0].format.bits_per_sample >> 3);

      if (in_samples < me_ptr->dm_info.expected_in_samples)
      {
         // process with available input samples if EOS/EOF is set, else return need more
         // Also consume available input if got set_param for consume_partial_data.
         if (input[port]->flags.end_of_frame || input[port]->flags.marker_eos ||
             me_ptr->dm_info.should_consume_partial_input)
         {
            me_ptr->dm_info.expected_in_samples = 0;
         }
         else // return need more
         {
            CNV_MSG(me_ptr->miid,
                    DBG_LOW_PRIO,
                    "[%u] Need more inp samples! expected %lu, received %lu",
                    me_ptr->type,
                    me_ptr->dm_info.expected_in_samples,
                    in_samples);

            // mark input unconsumed, need to update only 1st ch.
            input[port]->buf_ptr[0].actual_data_len = 0;
            return CAPI_ENEEDMORE;
         }
      }
      else if (in_samples > me_ptr->dm_info.expected_in_samples)
      {
         CNV_MSG(me_ptr->miid,
                 DBG_LOW_PRIO,
                 "[%u] Got more samples! expected %lu, received %lu",
                 me_ptr->type,
                 me_ptr->dm_info.expected_in_samples,
                 in_samples);

         //resampler will not consume more than expected samples, so update input buffer actual data len before.
         //test case: gen_cntr_two_priority_sync_parallel_paths_2, test passed but glitch was observed because samples were lost.
         src_buf_ptr->actual_data_len =
            me_ptr->dm_info.expected_in_samples * (me_ptr->in_media_fmt[0].format.bits_per_sample >> 3); //todo: check dynamic resampler
      }
   }

   // PC library updates only reads/writes into first ch capi_buf_t[0] lengths, capi is expected to update rest of the
   // channel buffers if the interleaving format is not unpacked V2
   me_ptr->pc[port].heap_id  = (uint32_t)me_ptr->heap_info.heap_id;
   capi_result = ar_result_to_capi_err(pc_process(&me_ptr->pc[port],
                                                  src_buf_ptr,
                                                  dest_buf_ptr,
                                                  &me_ptr->scratch_buf_1[port],
                                                  &me_ptr->scratch_buf_2[port]));
   if (CAPI_ENEEDMORE == capi_result)
   {
	   #ifdef PCM_CNV_DEBUG
      CNV_MSG(me_ptr->miid,
              DBG_HIGH_PRIO,
              "[%u] needs more input, input actual_data_len per ch = %ld ",
              me_ptr->type,
              (input[port]->buf_ptr ? input[port]->buf_ptr[0].actual_data_len : 0));
#endif
      // Drop data for eof case. Mark data as unconsumed for non-eof case.
      if (!input[port]->flags.end_of_frame)
      {
         input[port]->buf_ptr[0].actual_data_len = 0;
      }
      return capi_result;
   }

   if (0 != capi_result)
   {
      CNV_MSG(me_ptr->miid,
              DBG_ERROR_PRIO,
              "[%u] Error while pcm converter process, inp and out media fmts are ",
              me_ptr->type);
      capi_pcm_mf_cnv_print_media_fmt(me_ptr, &me_ptr->pc[port].core_lib.input_media_fmt, port);
      capi_pcm_mf_cnv_print_media_fmt(me_ptr, &me_ptr->pc[port].core_lib.output_media_fmt, port);
   }

   uint32_t is_gapless_case = FALSE;

   if ((me_ptr->initial_silence_in_samples > 0) || (me_ptr->trailing_silence_in_samples > 0))
   {
      capi_media_fmt_v2_t *out_mf              = NULL;
      out_mf                                   = me_ptr->out_media_fmt;
      bool_t                 is_initial        = TRUE;
      uint32_t               num_bytes_removed = 0;
      uint32_t               bytes_per_sample  = (out_mf->format.bits_per_sample) / 8;
      capi_stream_data_v2_t *in_stream_ptr     = (capi_stream_data_v2_t *)input[0];
      module_cmn_md_list_t * metadata_list_ptr = in_stream_ptr->metadata_list_ptr;

      if (actual_input_bytes == input[0]->buf_ptr->actual_data_len)
      {
         all_input_consumed = TRUE;
      }
      // Detect last frame based on EOS.
      if (in_stream_ptr->flags.marker_eos && all_input_consumed && me_ptr->trailing_silence_in_samples)
      {

         is_initial = FALSE;

         uint32_t bytes_to_remove_per_channel = me_ptr->trailing_silence_in_samples * bytes_per_sample;
         capi_cmn_gapless_remove_zeroes(&bytes_to_remove_per_channel,
                                        out_mf,
                                        &output[0],
                                        is_initial,
                                        metadata_list_ptr);
         me_ptr->trailing_silence_in_samples = 0;
      }

      if ((me_ptr->initial_silence_in_samples > 0) && (output[0]->buf_ptr->actual_data_len > 0))
      {
         uint32_t bytes_to_remove_per_channel = me_ptr->initial_silence_in_samples * bytes_per_sample;
         num_bytes_removed                    = bytes_to_remove_per_channel;
         capi_cmn_gapless_remove_zeroes(&bytes_to_remove_per_channel,
                                        out_mf,
                                        &output[0],
                                        is_initial,
                                        metadata_list_ptr);
         num_bytes_removed -= bytes_to_remove_per_channel;
         if (output[0]->flags.is_timestamp_valid && num_bytes_removed)
         {
            is_gapless_case = TRUE;
            output[0]->timestamp =
               output[0]->timestamp + ((num_bytes_removed / bytes_per_sample) / out_mf->format.sampling_rate) * 1000000;
         }
         me_ptr->initial_silence_in_samples = bytes_to_remove_per_channel / bytes_per_sample;
      }
   }

   if (input[port]->flags.is_timestamp_valid && (FALSE == is_gapless_case))
   {
      output[port]->timestamp                = input[port]->timestamp - me_ptr->events_config.delay_in_us;
      output[port]->flags.is_timestamp_valid = TRUE;
   }

   return capi_result;
}
