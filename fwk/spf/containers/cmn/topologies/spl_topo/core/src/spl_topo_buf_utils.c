/**
 * \file spl_topo_buf_utils.c
 *
 * \brief
 *
 *     Topo 2 utility functions for managing buffers.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spl_topo_i.h"

/* =======================================================================
Public Function Definitions
========================================================================== */

/**
 * Calculate the buffer size given the output media format. If for_delivery is TRUE, calculates the amount
 * the buffer must be filled for it to be delivered at an external output port (admittedly this is a framework
 * layer calculation, but to take advantage of code reuse it is placed here). For fixed input fractional use
 * cases, the buffer size must be large enough to handle expected frame size deviation from the nominal frame
 * size, and the delivery criteria must be small enough to handle expected frame size deviation from the
 * nominal frame size.
 */
uint32_t spl_topo_calc_buf_size(spl_topo_t *            topo_ptr,
                                spl_topo_frame_length_t frame_len,
                                topo_media_fmt_t *      media_format_ptr,
                                bool_t                  for_delivery)
{
   uint32_t num_samples          = 0;
   uint32_t nominal_out_buf_size = 0;
   uint32_t buf_duration_us      = frame_len.frame_len_us;

   if (for_delivery)
   {
      num_samples = spl_topo_get_scaled_samples(topo_ptr,
                                                frame_len.frame_len_samples,
                                                frame_len.sample_rate,
                                                media_format_ptr->pcm.sample_rate);
   }
   else
   {
      // Allocate buffers that are 1 ms larger in size.
      if (spl_topo_fwk_ext_any_dm_module_present(topo_ptr->fwk_extn_info.dm_info))
      {
         buf_duration_us += 1000;
      }
      num_samples = topo_us_to_samples(buf_duration_us, media_format_ptr->pcm.sample_rate);
   }

   nominal_out_buf_size = topo_samples_to_bytes(num_samples, media_format_ptr);

   // For non-fractional cases, there will be no deviations from the nominal frame size. Delivery size equals
   // buffer size equals nominal size.
   return nominal_out_buf_size;
}

/*
 * Queries the topo buffer manager to retrieve a buffer for this output port, to be stored in
 * out_port_ptr->t_base.common.bufs_ptr[0].data_ptr.
 */
ar_result_t spl_topo_get_output_port_buffer(spl_topo_t *topo_ptr, spl_topo_output_port_t *out_port_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

   spl_topo_module_t *module_ptr = (spl_topo_module_t *)out_port_ptr->t_base.gu.cmn.module_ptr;

   /* Inplace buffer is used.
    * 1. If module is disabled or if it supports inplace processing. And
    * 2. Module is not at the external boundary. This is to avoid using the external buffers as internal buffers. And
    * 2. Module's threshold is not less than the container frame size.
    *    It should be able to process whole frame in one iteration.
    *    If we use inplace buffers for module which requires multiple process iteration then it can cause data
    * corruption.
    *    Because we have to adjust the input buffer for each iteration based on the data consumed, doing so corrupt the
    * output data.
    * */

   if ((module_ptr->t_base.num_proc_loops <= 1) &&
       (module_ptr->t_base.flags.inplace || module_ptr->t_base.bypass_ptr) &&
       (module_ptr->t_base.gu.input_port_list_ptr))
   {
      spl_topo_input_port_t *in_port_ptr =
         (spl_topo_input_port_t *)module_ptr->t_base.gu.input_port_list_ptr->ip_port_ptr;

      if (in_port_ptr && !in_port_ptr->t_base.gu.ext_in_port_ptr && !out_port_ptr->t_base.gu.ext_out_port_ptr)
      {
         spl_topo_output_port_t *prev_out_port_ptr = (spl_topo_output_port_t *)in_port_ptr->t_base.gu.conn_out_port_ptr;
         if (prev_out_port_ptr && prev_out_port_ptr->t_base.common.bufs_ptr[0].data_ptr)
         {
            // in place modules are SISO (already checked)
#ifdef SAFE_MODE_SDATA_BUF_LENGTHS
            for (uint32_t b = 0; b < prev_out_port_ptr->t_base.common.sdata.bufs_num; b++)
            {
               out_port_ptr->t_base.common.bufs_ptr[b].data_ptr = prev_out_port_ptr->t_base.common.bufs_ptr[b].data_ptr;
               out_port_ptr->t_base.common.bufs_ptr[b].max_data_len =
                  prev_out_port_ptr->t_base.common.bufs_ptr[b].max_data_len;
            }
#else
            for (uint32_t b = 0; b < prev_out_port_ptr->t_base.common.sdata.bufs_num; b++)
            {
               out_port_ptr->t_base.common.bufs_ptr[b].data_ptr = prev_out_port_ptr->t_base.common.bufs_ptr[b].data_ptr;
            }
            out_port_ptr->t_base.common.bufs_ptr[0].max_data_len =
               prev_out_port_ptr->t_base.common.bufs_ptr[0].max_data_len;
#endif

            out_port_ptr->t_base.common.flags.buf_origin = prev_out_port_ptr->t_base.common.flags.buf_origin;
            gen_topo_buf_mgr_wrapper_inc_ref_count(&out_port_ptr->t_base.common);

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
            TOPO_MSG(topo_ptr->t_base.gu.log_id,
                     DBG_HIGH_PRIO,
                     "using inplace buffer for output port idx %ld miid 0x%lx",
                     out_port_ptr->t_base.gu.cmn.index,
                     out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
#endif
         }
      }
   }

   if (!out_port_ptr->t_base.common.bufs_ptr[0].data_ptr)
   {
      // Get buffer from the buffer manager.
      TRY(result,
          gen_topo_buf_mgr_wrapper_get_buf(&topo_ptr->t_base,
                                           &out_port_ptr->t_base.common));

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_HIGH_PRIO,
               "got a new a new topomgr buffer for output port idx %ld miid 0x%lx",
               out_port_ptr->t_base.gu.cmn.index,
               out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
#endif
   }

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
   }

   return result;
}

/**
 * Drops all data in the buffer, destroy corresponding metadata, returns the output buffer, clears sdata flags.
 */
ar_result_t spl_topo_flush_return_output_buf(spl_topo_t *topo_ptr, spl_topo_output_port_t *out_port_ptr)
{
   INIT_EXCEPTION_HANDLING
   bool_t                  IS_DROPPED_TRUE           = TRUE;
   ar_result_t             result                    = AR_EOK;
   spl_topo_input_port_t * connected_int_in_port_ptr = NULL;
   spl_topo_output_port_t *ext_out_port_ptr          = NULL;

   spl_topo_get_connected_int_or_ext_ip_port(topo_ptr, out_port_ptr, &connected_int_in_port_ptr, &ext_out_port_ptr);

   // if internal connected input port is found then move all the metadata.
   // In case of external output port, we can drop all the metadata.
   if (connected_int_in_port_ptr)
   {
      spl_topo_module_t *downstream_module_ptr =
         (spl_topo_module_t *)connected_int_in_port_ptr->t_base.gu.cmn.module_ptr;
      bool_t is_eos_present = connected_int_in_port_ptr->t_base.common.sdata.flags.marker_eos;
      connected_int_in_port_ptr->t_base.common.sdata.flags.marker_eos = FALSE;

      if (out_port_ptr->t_base.common.bufs_ptr)
      {
         // since we are dropping the output buffer therefore adjust the md offset.
         uint32_t bytes_across_all_ch = gen_topo_get_total_actual_len(&out_port_ptr->t_base.common);
         gen_topo_metadata_adj_offset(&topo_ptr->t_base,
                                      out_port_ptr->t_base.common.media_fmt_ptr,
                                      out_port_ptr->t_base.common.sdata.metadata_list_ptr,
                                      bytes_across_all_ch,
                                      FALSE);
      }

      // move the md to connected input port
      spl_topo_transfer_md_between_ports(topo_ptr,
                                         (void *)connected_int_in_port_ptr,
                                         TRUE,
                                         (void *)out_port_ptr,
                                         FALSE);

      // if eos moved from output port to input then flush the module.
      if (connected_int_in_port_ptr->t_base.common.sdata.flags.marker_eos)
      {
         connected_int_in_port_ptr->t_base.common.sdata.flags.end_of_frame = TRUE;

         spl_topo_set_eos_zeros_to_flush_bytes(topo_ptr, downstream_module_ptr);

         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_MED_PRIO,
                  "SPL_TOPO_EOS_DEBUG: Started flushing eos zeros on miid = 0x%lx. %ld "
                  "bytes per channel of zeros to flush, module algo delay %ld",
                  downstream_module_ptr->t_base.gu.module_instance_id,
                  downstream_module_ptr->t_base.pending_zeros_at_eos,
                  downstream_module_ptr->t_base.algo_delay);
      }

      connected_int_in_port_ptr->t_base.common.sdata.flags.marker_eos |= is_eos_present;
   }

   if (out_port_ptr->t_base.common.bufs_ptr)
   {
      spl_topo_return_output_buf(topo_ptr, out_port_ptr);
   }

   if (out_port_ptr->md_list_ptr)
   {
#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "Destroying metadata for output port idx %ld miid 0x%lx",
               out_port_ptr->t_base.gu.cmn.index,
               out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
#endif

      TRY(result,
          gen_topo_destroy_all_metadata(topo_ptr->t_base.gu.log_id,
                                        (void *)out_port_ptr->t_base.gu.cmn.module_ptr,
                                        &(out_port_ptr->md_list_ptr),
                                        IS_DROPPED_TRUE));
   }

   out_port_ptr->t_base.common.sdata.flags.word                = 0;
   out_port_ptr->t_base.common.sdata.flags.stream_data_version = CAPI_STREAM_V2;

   if (connected_int_in_port_ptr && connected_int_in_port_ptr->t_base.common.sdata.flags.marker_eos)
   {
      // while preparing the input port for process, the flags are copied from the connected output port.
      // since all flags are cleared above, need to re-set the eof flag for flushing eos.
      out_port_ptr->t_base.common.sdata.flags.end_of_frame = TRUE;
   }

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
   }

   return result;
}

/**
 * Return an output buffer to the topo buffer manager.
 */
void spl_topo_return_output_buf(spl_topo_t *topo_ptr, spl_topo_output_port_t *out_port_ptr)
{
   if (!out_port_ptr->t_base.common.bufs_ptr[0].data_ptr)
   {
#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_1
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_LOW_PRIO,
               "Buffer not present to release on port idx %ld miid 0x%lx!",
               out_port_ptr->t_base.gu.cmn.index,
               out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
#endif
      return;
   }

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_MED_PRIO,
            "Topo returning buffer for output port idx %ld miid 0x%lx, actual_data_len %lu",
            out_port_ptr->t_base.gu.cmn.index,
            out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id,
            out_port_ptr->t_base.common.bufs_ptr[0].actual_data_len);
#endif

   gen_topo_set_all_bufs_len_to_zero(&out_port_ptr->t_base.common);
   out_port_ptr->t_base.common.flags.force_return_buf = TRUE;
   gen_topo_return_one_buf_mgr_buf(&topo_ptr->t_base,
                                   &out_port_ptr->t_base.common,
                                   out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id,
                                   out_port_ptr->t_base.gu.cmn.id);

   spl_topo_clear_output_timestamp_discontinuity(topo_ptr, out_port_ptr);
}

/**
 * After the capi v2 process call, if there is partially consumed input data,
 * we want to remove consumed data from the buffer. The buffer's timestamp (lives
 * in the sdata) is adjusted accordingly. Removing unconsumed data allows for the maximum
 * amount of space in the buffers which reduces the chance that the next capi_process
 * fails due to not enough space in buffers.
 */
ar_result_t spl_topo_adjust_buf(spl_topo_t            *topo_ptr,
                                topo_buf_t            *buf_ptr,
                                capi_stream_data_v2_t *sdata_ptr,
                                uint32_t               consumed_data_per_ch,
                                topo_media_fmt_t      *media_fmt_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result               = AR_EOK;
   uint32_t    copy_len             = 0;
   uint32_t    new_actual_data_size = 0;
   uint32_t    num_channels         = media_fmt_ptr->pcm.num_channels;
   bool_t      SUBTRACT_FALSE       = FALSE;
   uint32_t    max_data_len_per_ch  = buf_ptr[0].max_data_len;

   // Cannot consume more data than is available.
   VERIFY(result, (consumed_data_per_ch) <= buf_ptr[0].actual_data_len);

   new_actual_data_size = (buf_ptr[0].actual_data_len) - consumed_data_per_ch;

   for (uint32_t i = 0; i < sdata_ptr->bufs_num; i++)
   {
      // Shift data one channel at a time.

      copy_len = memsmove(buf_ptr[i].data_ptr,
                          max_data_len_per_ch,
                          buf_ptr[i].data_ptr + consumed_data_per_ch,
                          new_actual_data_size);

      if (copy_len != new_actual_data_size)
      {
         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_MED_PRIO,
                  "Error dropping consumed data, memsmove did not move expected amount of data."
                  "consumed_data_per_ch: %ld, new actual data size: %ld, buf_ptr->max_data_len: %ld",
                  consumed_data_per_ch,
                  new_actual_data_size,
                  buf_ptr[i].max_data_len);
         return AR_EFAILED;
      }

   // update actual lens after adjusting the buffer.
#ifdef SAFE_MODE_SDATA_BUF_LENGTHS
      buf_ptr[i].actual_data_len = copy_len;
   }
#else
   }
   // optimization: update only first buf outside the loop
   buf_ptr[0].actual_data_len = copy_len;
#endif

   if (sdata_ptr->metadata_list_ptr)
   {
      // Offsets have to be adjusted as well.
      gen_topo_metadata_adj_offset(&(topo_ptr->t_base),
                                   media_fmt_ptr,
                                   sdata_ptr->metadata_list_ptr,
                                   consumed_data_per_ch * num_channels,
                                   SUBTRACT_FALSE);
   }

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
   }

   return result;
}

/**
 * Returns true if the max data length of this output port's buffer is known. This is true if the media format
 * has been set.
 */
bool_t spl_topo_output_port_is_size_known(void *ctx_topo_ptr, void *ctx_out_port_ptr)
{
   spl_topo_output_port_t *out_port_ptr = (spl_topo_output_port_t *)ctx_out_port_ptr;
   return out_port_ptr->t_base.common.flags.is_mf_valid;
}

// Calculates and caches topo buffers lengths in port handles, to avoid runtime calculation of the size.
// This needs to be updated when,
// 1. Threshold changes
// 2. media format changes
ar_result_t spl_topo_update_max_buf_len_for_all_modules(spl_topo_t *topo_ptr)
{
   ar_result_t result = AR_EOK;

   /** Update topo buffer lengths for all input and output ports. */
   for (gu_sg_list_t *sg_list_ptr = topo_ptr->t_base.gu.sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         spl_topo_module_t *module_ptr = (spl_topo_module_t *)module_list_ptr->module_ptr;
         result |= spl_topo_update_max_buf_len_for_single_module(topo_ptr, module_ptr);
      }
   }

   return result;
}

// Calculates and updates topo buffers lengths for all ports, for the given module
ar_result_t spl_topo_update_max_buf_len_for_single_module(spl_topo_t *topo_ptr, spl_topo_module_t *module_ptr)
{
   ar_result_t result             = AR_EOK;
   bool_t      FOR_DELIVERY_FALSE = FALSE;

   /** Update topo buffer lengths for all module's input and output ports. */
   for (gu_output_port_list_t *out_port_list_ptr = module_ptr->t_base.gu.output_port_list_ptr;
        (NULL != out_port_list_ptr);
        LIST_ADVANCE(out_port_list_ptr))
   {
      spl_topo_output_port_t *out_port_ptr = (spl_topo_output_port_t *)out_port_list_ptr->op_port_ptr;

      if (out_port_ptr->t_base.common.flags.is_mf_valid)
      {
         out_port_ptr->t_base.common.max_buf_len = spl_topo_calc_buf_size(topo_ptr,
                                                                          topo_ptr->cntr_frame_len,
                                                                          out_port_ptr->t_base.common.media_fmt_ptr,
                                                                          FOR_DELIVERY_FALSE);

         uint32_t bufs_num = out_port_ptr->t_base.common.media_fmt_ptr->pcm.num_channels;
         out_port_ptr->t_base.common.max_buf_len_per_buf =
            topo_div_num(out_port_ptr->t_base.common.max_buf_len, bufs_num);
      }
      else
      {
         out_port_ptr->t_base.common.max_buf_len         = 0;
         out_port_ptr->t_base.common.max_buf_len_per_buf = 0;
      }

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "Max buf length is %lu bytes for output port idx %ld miid 0x%lx",
               out_port_ptr->t_base.common.max_buf_len,
               out_port_ptr->t_base.gu.cmn.index,
               out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
#endif
   }

   for (gu_input_port_list_t *in_port_list_ptr = module_ptr->t_base.gu.input_port_list_ptr; (NULL != in_port_list_ptr);
        LIST_ADVANCE(in_port_list_ptr))
   {
      spl_topo_input_port_t *in_port_ptr = (spl_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

      if (in_port_ptr->t_base.common.flags.is_mf_valid)
      {
         in_port_ptr->t_base.common.max_buf_len = spl_topo_calc_buf_size(topo_ptr,
                                                                         topo_ptr->cntr_frame_len,
                                                                         in_port_ptr->t_base.common.media_fmt_ptr,
                                                                         FOR_DELIVERY_FALSE);

         uint32_t bufs_num = in_port_ptr->t_base.common.media_fmt_ptr->pcm.num_channels;
         in_port_ptr->t_base.common.max_buf_len_per_buf =
            topo_div_num(in_port_ptr->t_base.common.max_buf_len, bufs_num);
      }
      else
      {
         in_port_ptr->t_base.common.max_buf_len         = 0;
         in_port_ptr->t_base.common.max_buf_len_per_buf = 0;
      }
#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "Max buf length is %lu bytes for input port idx %ld miid 0x%lx",
               in_port_ptr->t_base.common.max_buf_len,
               in_port_ptr->t_base.gu.cmn.index,
               in_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
#endif
   }

   return result;
}

/**
 * Implemented spl-specific feset on top of shared reset since md is stored in different location in spl output ports.
 * Also return output ports to the buf mgr since data is dropped.
 */
ar_result_t spl_topo_reset_output_port(spl_topo_t *topo_ptr, spl_topo_output_port_t *topo_out_port_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result          = AR_EOK;
   bool_t      IS_DROPPED_TRUE = TRUE;

   if (topo_out_port_ptr->md_list_ptr)
   {
#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "Destroying metadata for output port idx %ld miid 0x%lx",
               topo_out_port_ptr->t_base.gu.cmn.index,
               topo_out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
#endif

      TRY(result,
          gen_topo_destroy_all_metadata(topo_ptr->t_base.gu.log_id,
                                        (void *)topo_out_port_ptr->t_base.gu.cmn.module_ptr,
                                        &(topo_out_port_ptr->md_list_ptr),
                                        IS_DROPPED_TRUE));
   }

   topo_shared_reset_output_port(&topo_ptr->t_base, &topo_out_port_ptr->t_base, FALSE);

   if (topo_out_port_ptr->t_base.common.bufs_ptr)
   {
      spl_topo_return_output_buf(topo_ptr, topo_out_port_ptr);
   }

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
   }
   return result;
}
