/**
 * \file spl_topo_metadata.c
 *
 * \brief
 *
 *     Implementation of the metadata handler for spl_topo
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spl_topo_i.h"

/* =======================================================================
Function Definitions
========================================================================== */

/**
 * Transfer metadata from the output port process context to the output port.
 */
ar_result_t spl_topo_transfer_md_from_out_sdata_to_out_port(spl_topo_t *topo_ptr, spl_topo_output_port_t *out_port_ptr)
{
   ar_result_t                 result           = AR_EOK;
   bool_t                      ADD_TRUE         = TRUE;
   module_cmn_md_list_t **     dst_md_list_pptr = &(out_port_ptr->md_list_ptr);
   module_cmn_md_list_t **     src_md_list_pptr = &(out_port_ptr->t_base.common.sdata.metadata_list_ptr);
   gen_topo_process_context_t *pc_ptr           = &(topo_ptr->t_base.proc_context);

   uint32_t bytes_existing = pc_ptr->out_port_scratch_ptr[out_port_ptr->t_base.gu.cmn.index].prev_actual_data_len[0];

   if (0 < bytes_existing)
   {
      if (SPF_IS_PACKETIZED_OR_PCM(out_port_ptr->t_base.common.media_fmt_ptr->data_format) &&
          TU_IS_ANY_DEINTERLEAVED_UNPACKED(out_port_ptr->t_base.common.media_fmt_ptr->pcm.interleaving))
      {
         bytes_existing = (pc_ptr->out_port_scratch_ptr[out_port_ptr->t_base.gu.cmn.index].prev_actual_data_len[0]) *
                          out_port_ptr->t_base.common.sdata.bufs_num;
      }

      // Increase the metadata offsets by the amount of data present in the output buffer before this process call.
      gen_topo_metadata_adj_offset(&(topo_ptr->t_base),
                                   out_port_ptr->t_base.common.media_fmt_ptr,
                                   out_port_ptr->t_base.common.sdata.metadata_list_ptr,
                                   bytes_existing,
                                   ADD_TRUE);
   }

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
   if (*src_md_list_pptr)
   {

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_5
      module_cmn_md_list_t *node_ptr = out_port_ptr->t_base.common.sdata.metadata_list_ptr;
      while (node_ptr)
      {
         module_cmn_md_t *md_ptr = node_ptr->obj_ptr;

         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_MED_PRIO,
                  "Transferring metadata from process context to output port for port idx = %ld, miid = 0x%lx, md ptr "
                  "= 0x%lx, offset = %ld",
                  out_port_ptr->t_base.gu.cmn.index,
                  out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id,
                  md_ptr,
                  md_ptr->offset);

         node_ptr = node_ptr->next_ptr;
      }
#else
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "Transferring metadata from process context to output port for port idx = %ld, miid = 0x%lx",
               out_port_ptr->t_base.gu.cmn.index,
               out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
#endif
   }
   else
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "No metadata to transfer from process context to output port for port idx = %ld, miid = 0x%lx",
               out_port_ptr->t_base.gu.cmn.index,
               out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
   }
#endif

   // Put src md list into dst md list.
   spf_list_merge_lists((spf_list_node_t **)dst_md_list_pptr, (spf_list_node_t **)src_md_list_pptr);

   return result;
}

/**
 * Function to transfer metadata from one port (src) to another port (dst). Metadata-related flags should
 * also be transferred.
 */
ar_result_t spl_topo_transfer_md_between_ports(spl_topo_t *topo_ptr,
                                               void *      dst_port_ptr,
                                               bool_t      dst_is_input,
                                               void *      src_port_ptr,
                                               bool_t      src_is_input)
{
   ar_result_t result                      = AR_EOK;
   bool_t      new_dst_marker_eos          = FALSE;
   bool_t      is_redundant_eos_assignment = FALSE;

   module_cmn_md_list_t **dst_md_list_pptr =
      dst_is_input ? &(((spl_topo_input_port_t *)dst_port_ptr)->t_base.common.sdata.metadata_list_ptr)
                   : &(((spl_topo_output_port_t *)dst_port_ptr)->md_list_ptr);
   module_cmn_md_list_t **src_md_list_pptr =
      src_is_input ? &(((spl_topo_input_port_t *)src_port_ptr)->t_base.common.sdata.metadata_list_ptr)
                   : &(((spl_topo_output_port_t *)src_port_ptr)->md_list_ptr);

   gen_topo_common_port_t *dst_cmn_port_ptr = dst_is_input ? &(((spl_topo_input_port_t *)dst_port_ptr)->t_base.common)
                                                           : &(((spl_topo_output_port_t *)dst_port_ptr)->t_base.common);
   gen_topo_common_port_t *src_cmn_port_ptr = src_is_input ? &(((spl_topo_input_port_t *)src_port_ptr)->t_base.common)
                                                           : &(((spl_topo_output_port_t *)src_port_ptr)->t_base.common);

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
   bool_t src_port_idx = src_is_input ? ((spl_topo_input_port_t *)src_port_ptr)->t_base.gu.cmn.index
                                      : ((spl_topo_output_port_t *)src_port_ptr)->t_base.gu.cmn.index;
   uint32_t src_port_miid = src_is_input
                               ? ((spl_topo_input_port_t *)src_port_ptr)->t_base.gu.cmn.module_ptr->module_instance_id
                               : ((spl_topo_output_port_t *)src_port_ptr)->t_base.gu.cmn.module_ptr->module_instance_id;

   bool_t dst_port_idx = dst_is_input ? ((spl_topo_input_port_t *)dst_port_ptr)->t_base.gu.cmn.index
                                      : ((spl_topo_output_port_t *)dst_port_ptr)->t_base.gu.cmn.index;
   uint32_t dst_port_miid = dst_is_input
                               ? ((spl_topo_input_port_t *)dst_port_ptr)->t_base.gu.cmn.module_ptr->module_instance_id
                               : ((spl_topo_output_port_t *)dst_port_ptr)->t_base.gu.cmn.module_ptr->module_instance_id;
   if (*src_md_list_pptr)
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "SPL_TOPO_EOS_DEBUG: Transferring md from port is_input? %ld idx = %ld, miid = 0x%lx to port is_input? "
               "%ld "
               "idx = %ld, miid = 0x%lx. src eos %ld, dst eos %ld",
               src_is_input,
               src_port_idx,
               src_port_miid,
               dst_is_input,
               dst_port_idx,
               dst_port_miid,
               src_cmn_port_ptr->sdata.flags.marker_eos,
               dst_cmn_port_ptr->sdata.flags.marker_eos);
   }
   else
   {
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "SPL_TOPO_EOS_DEBUG: No md to transfer from port is_input? %ld idx = %ld, miid = 0x%lx to port "
               "is_input? "
               "%ld idx = %ld, miid = 0x%lx. src eos %ld, dst eos %ld",
               src_is_input,
               src_port_idx,
               src_port_miid,
               dst_is_input,
               dst_port_idx,
               dst_port_miid,
               src_cmn_port_ptr->sdata.flags.marker_eos,
               dst_cmn_port_ptr->sdata.flags.marker_eos);
   }
#endif

   // Put src md list into dst md list.
   spf_list_merge_lists((spf_list_node_t **)dst_md_list_pptr, (spf_list_node_t **)src_md_list_pptr);

   new_dst_marker_eos = dst_cmn_port_ptr->sdata.flags.marker_eos || src_cmn_port_ptr->sdata.flags.marker_eos;

   // If we had an eos marker on the source, it means we just put an eos into the dst, so add the marker eos flag to the
   // dst. For input ports, don't do assignment if src didn't have marker_eos, otherwise we would restart flushing when
   // no new eos came.
   is_redundant_eos_assignment = dst_is_input && (!src_cmn_port_ptr->sdata.flags.marker_eos) && new_dst_marker_eos;
   if (!is_redundant_eos_assignment)
   {
      spl_topo_assign_marker_eos_on_port(topo_ptr, dst_port_ptr, dst_is_input, new_dst_marker_eos);
   }

   // The src md list was depleted meaning it no longer has a flushing eos. Clear out marker eos from the source
   // port.
   spl_topo_assign_marker_eos_on_port(topo_ptr, src_port_ptr, src_is_input, FALSE);

   return result;
}

/**
 * Helper function for handling when assigning eos marker on a port.
 */
void spl_topo_assign_marker_eos_on_port(spl_topo_t *topo_ptr, void *port_ptr, bool_t is_input, bool_t new_marker_eos)
{
   gen_topo_common_port_t *cmn_port_ptr = is_input ? &(((spl_topo_input_port_t *)port_ptr)->t_base.common)
                                                   : &(((spl_topo_output_port_t *)port_ptr)->t_base.common);
#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_5
   bool_t port_idx = is_input ? ((spl_topo_input_port_t *)port_ptr)->t_base.gu.cmn.index
                              : ((spl_topo_output_port_t *)port_ptr)->t_base.gu.cmn.index;
   bool_t port_miid = is_input ? ((spl_topo_input_port_t *)port_ptr)->t_base.gu.cmn.module_ptr->module_instance_id
                               : ((spl_topo_output_port_t *)port_ptr)->t_base.gu.cmn.module_ptr->module_instance_id;

   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_MED_PRIO,
            "SPL_TOPO_EOS_DEBUG: port is_input? %ld idx = %ld, miid = 0x%lx marker_eos was %ld, now assigned to %ld",
            is_input,
            port_idx,
            port_miid,
            cmn_port_ptr->sdata.flags.marker_eos,
            new_marker_eos);
#endif

   cmn_port_ptr->sdata.flags.marker_eos = new_marker_eos;
   cmn_port_ptr->sdata.flags.end_of_frame |= new_marker_eos;
   topo_ptr->simpt_event_flags.check_eof |= new_marker_eos;
}

/**
 * Append zeros to the input port's buffer for cases when the fwk handles flushing eos for the given
 * module. Extends the actual data length of the buffer up to at most max length depending on the
 * amount of zeros to flush, and memsets the new space to zero.
 */
ar_result_t topo_2_append_eos_zeros(spl_topo_t *            topo_ptr,
                                    spl_topo_module_t *     module_ptr,
                                    spl_topo_input_port_t * in_port_ptr,
                                    spl_topo_output_port_t *connected_out_port_ptr,
                                    spl_topo_input_port_t * ext_in_port_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result                  = AR_EOK;
   uint32_t    num_channels            = in_port_ptr->t_base.common.media_fmt_ptr->pcm.num_channels;
   uint32_t    amount_zero_push_per_ch = 0;

   VERIFY(result, topo_ptr->t_base.topo_to_cntr_vtable_ptr->set_ext_in_port_prev_actual_data_len);

   // One should be NULL, one should be non-NULL.
   VERIFY(result, (connected_out_port_ptr || ext_in_port_ptr) && ((!connected_out_port_ptr) || (!ext_in_port_ptr)));

   VERIFY(result,
          (SPF_IS_PCM_DATA_FORMAT(in_port_ptr->t_base.common.media_fmt_ptr->data_format)) &&
             TU_IS_ANY_DEINTERLEAVED_UNPACKED(in_port_ptr->t_base.common.media_fmt_ptr->pcm.interleaving));

   // Only append zeros once per process call.
   if (in_port_ptr->fwd_kick_flags.flushed_zeros)
   {
      return result;
   }

   // If there is an upstream output port, update that buffer's actual data length (all channels stored together).
   if (connected_out_port_ptr)
   {
      // Determine amount of zeros to write, minimum of zeros remaining and empty space in buffer.
      bool_t   FOR_DELIVERY           = TRUE;
      uint32_t ch_spacing             = connected_out_port_ptr->t_base.common.bufs_ptr[0].max_data_len;
      uint32_t ch_offset              = connected_out_port_ptr->t_base.common.bufs_ptr[0].actual_data_len;
      uint32_t max_empty_space_per_ch = ch_spacing - ch_offset;

      // TODO: can we use max buf len instead of calculating ?
      uint32_t cntr_len_bytes_per_ch = capi_cmn_divide(spl_topo_calc_buf_size(topo_ptr,
                                                                              topo_ptr->cntr_frame_len,
                                                                              in_port_ptr->t_base.common.media_fmt_ptr,
                                                                              FOR_DELIVERY),
                                                       num_channels);

      // Ternary to avoid problems when buffer is already filled past the container frame length. In this case don't
      // push any zeros.
      uint32_t empty_space_up_to_cntr_len =
         (ch_offset > cntr_len_bytes_per_ch) ? 0 : (cntr_len_bytes_per_ch - ch_offset);

      // Limit zero pushing never fill the buffer more than the contaner frame length amount.
      max_empty_space_per_ch  = MIN(max_empty_space_per_ch, empty_space_up_to_cntr_len);
      amount_zero_push_per_ch = MIN(module_ptr->t_base.pending_zeros_at_eos, max_empty_space_per_ch);

      // If there is no space to push zeros or no zeros to push, return.
      if (0 == amount_zero_push_per_ch)
      {
         return result;
      }

      // Append zeros by memsetting and adjusting actual data length.
      uint32_t actual_data_len_per_ch = connected_out_port_ptr->t_base.common.bufs_ptr[0].actual_data_len;
      for (uint32_t ch_idx = 0; ch_idx < num_channels; ch_idx++)
      {
         int8_t *write_ptr = connected_out_port_ptr->t_base.common.bufs_ptr[ch_idx].data_ptr + actual_data_len_per_ch;

         memset(write_ptr, 0, amount_zero_push_per_ch);

#ifdef SAFE_MODE_SDATA_BUF_LENGTHS
         connected_out_port_ptr->t_base.common.bufs_ptr[ch_idx].actual_data_len += amount_zero_push_per_ch;
      }
#else
      }
      // optimization: update only first ch outside the for loop
      connected_out_port_ptr->t_base.common.bufs_ptr[0].actual_data_len += amount_zero_push_per_ch;
#endif
   }
   else if (ext_in_port_ptr)
   {
      // Determine amount of zeros to write, minimum of zeros remaining and empty space in buffer.
      spl_topo_ext_buf_t *ext_buf_ptr = ext_in_port_ptr->ext_in_buf_ptr;
      int8_t *            write_ptr   = NULL;
      uint32_t max_empty_space_per_ch = ext_buf_ptr->buf_ptr[0].max_data_len - ext_buf_ptr->buf_ptr[0].actual_data_len;
      amount_zero_push_per_ch         = MIN(module_ptr->t_base.pending_zeros_at_eos, max_empty_space_per_ch);

      // If there is no space to push zeros or no zeros to push, return.
      if (0 == amount_zero_push_per_ch)
      {
         return result;
      }

      // Append zeros by memsetting and adjusting actual data length.
      uint32_t actual_data_len_per_ch = ext_buf_ptr->buf_ptr[0].actual_data_len;
      for (uint32_t buf_idx = 0; buf_idx < ext_buf_ptr->num_bufs; buf_idx++)
      {
         write_ptr = ext_buf_ptr->buf_ptr[buf_idx].data_ptr + actual_data_len_per_ch;
         memset(write_ptr, 0, amount_zero_push_per_ch);

#ifdef SAFE_MODE_SDATA_BUF_LENGTHS
         ext_buf_ptr->buf_ptr[buf_idx].actual_data_len += amount_zero_push_per_ch;
      }
#else
      }
      // optimization: update only first ch outside the for loop
      ext_buf_ptr->buf_ptr[0].actual_data_len += amount_zero_push_per_ch;
#endif
      // We pushed zeros. For external ports we need to communicate this info back to the fwk to
      // properly update the previous actual data length for the after-process check if all input was consumed.
      topo_ptr->t_base.topo_to_cntr_vtable_ptr
         ->set_ext_in_port_prev_actual_data_len(&topo_ptr->t_base,
                                                ext_in_port_ptr->t_base.gu.ext_in_port_ptr,
                                                ext_buf_ptr->buf_ptr[0].actual_data_len);
   }

   module_ptr->t_base.pending_zeros_at_eos -= amount_zero_push_per_ch;

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_MED_PRIO,
            "SPL_TOPO_EOS_DEBUG: Appending %ld bytes per channel of zeros on input port idx = %ld, miid = 0x%lx. %ld "
            "bytes per channel of zeros remaining.",
            amount_zero_push_per_ch,
            in_port_ptr->t_base.gu.cmn.index,
            in_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id,
            module_ptr->t_base.pending_zeros_at_eos);
#endif

   in_port_ptr->fwd_kick_flags.flushed_zeros = TRUE;

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
   }
   return result;
}

/**
 * Call gen_topo to propagate metadata. Derives the amount of input consumed and output produced
 * by multiplying the sdata first buffer's actual_data_len by num_ch.
 *
 * WARNING This should ONLY be called from spl_topo_process() directly after calling spl_topo_process_module().
 * Otherwise actual_data_len values might not be correct.
 */
ar_result_t spl_topo_propagate_metadata(spl_topo_t *              topo_ptr,
                                        spl_topo_module_t *       module_ptr,
                                        bool_t                    input_has_metadata_or_eos,
                                        uint32_t                  input_size_before_per_ch,
                                        spl_topo_process_status_t proc_status)
{
   uint32_t                in_bytes_consumed  = 0;
   uint32_t                out_bytes_produced = 0;
   uint32_t                num_ch             = 0;
   spl_topo_input_port_t * first_in_port_ptr  = NULL;
   spl_topo_output_port_t *first_out_port_ptr = NULL;

   // Nothing needed for mimo modules.
   if (TOPO_MODULE_TYPE_MULTIPORT == module_ptr->flags.module_type)
   {
      return AR_EOK;
   }

   switch (proc_status)
   {
      case SPL_TOPO_PROCESS:
      {
         uint32_t input_size_before = input_size_before_per_ch;
         if (1 == module_ptr->t_base.gu.num_input_ports)
         {
            first_in_port_ptr = (spl_topo_input_port_t *)module_ptr->t_base.gu.input_port_list_ptr->ip_port_ptr;
            num_ch            = first_in_port_ptr->t_base.common.media_fmt_ptr->pcm.num_channels;
            in_bytes_consumed = first_in_port_ptr->t_base.common.sdata.buf_ptr
                                   ? first_in_port_ptr->t_base.common.sdata.buf_ptr[0].actual_data_len * num_ch
                                   : 0;
            input_size_before *= num_ch;
         }

         if (1 == module_ptr->t_base.gu.num_output_ports)
         {
            first_out_port_ptr = (spl_topo_output_port_t *)module_ptr->t_base.gu.output_port_list_ptr->op_port_ptr;
            num_ch             = first_out_port_ptr->t_base.common.media_fmt_ptr->pcm.num_channels;
            out_bytes_produced = first_out_port_ptr->t_base.common.sdata.buf_ptr
                                    ? first_out_port_ptr->t_base.common.sdata.buf_ptr[0].actual_data_len * num_ch
                                    : 0;
         }

         // propagate buffer associate metadata from input to output of SISO module.
         if (!module_ptr->t_base.flags.is_nblc_boundary_module && first_in_port_ptr && first_out_port_ptr)
         {
            module_cmn_md_list_t *node_ptr = first_in_port_ptr->t_base.common.sdata.metadata_list_ptr;
            module_cmn_md_list_t *next_ptr = NULL;

            input_has_metadata_or_eos = first_in_port_ptr->t_base.common.sdata.flags.marker_eos;

            while (node_ptr)
            {
               next_ptr = node_ptr->next_ptr;

               module_cmn_md_t *md_ptr = node_ptr->obj_ptr;
               bool_t           is_eos_dfg =
                  ((MODULE_CMN_MD_ID_DFG == md_ptr->metadata_id) || (MODULE_CMN_MD_ID_EOS == md_ptr->metadata_id))
                     ? TRUE
                     : FALSE;
               uint32_t is_ba_md =
                  (MODULE_CMN_MD_BUFFER_ASSOCIATED == md_ptr->metadata_flag.buf_sample_association) ? TRUE : FALSE;

               if (!is_eos_dfg && is_ba_md)
               {
                  if (topo_samples_to_bytes(md_ptr->offset, first_in_port_ptr->t_base.common.media_fmt_ptr) <=
                      in_bytes_consumed)
                  {
                     spf_list_move_node_to_another_list((spf_list_node_t **)&(
                                                           first_out_port_ptr->t_base.common.sdata.metadata_list_ptr),
                                                        (spf_list_node_t *)node_ptr,
                                                        (spf_list_node_t **)&(
                                                           first_in_port_ptr->t_base.common.sdata.metadata_list_ptr));
#ifdef METADATA_DEBUGGING
                     TOPO_MSG(topo_ptr->t_base.gu.log_id,
                              DBG_HIGH_PRIO,
                              " Metadata id 0x%x propagating for SISO, module "
                              "0x%lX",
                              md_ptr->metadata_id,
                              module_ptr->t_base.gu.module_instance_id);
#endif
                  }
               }
               else
               {
                  input_has_metadata_or_eos = TRUE;
               }

               node_ptr = next_ptr;
            }
         }

         if (input_has_metadata_or_eos || module_ptr->t_base.int_md_list_ptr)
         {
            return gen_topo_propagate_metadata(&(topo_ptr->t_base),
                                               &(module_ptr->t_base),
                                               input_has_metadata_or_eos,
                                               input_size_before,
                                               in_bytes_consumed,
                                               out_bytes_produced);
         }

         break;
      }
      default:
      {
      }
   }

   return AR_EOK;
}

/**
 * Modifies md when new data arrives for an input port. The metadata search needs to go through the input port
 * metadata list and the input port's module's internal list. The input port's module should have pending flushing zeros
 * reset.
 */
ar_result_t spl_topo_ip_modify_md_when_new_data_arrives(spl_topo_t *           topo_ptr,
                                                        spl_topo_module_t *    module_ptr,
                                                        spl_topo_input_port_t *in_port_ptr,
                                                        uint32_t               new_data_amount,
                                                        bool_t                 new_flushing_eos_arrived)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result             = AR_EOK;
   bool_t      ADD_TRUE           = TRUE;
   bool_t      new_marker_eos     = FALSE;
   bool_t      tmp_new_marker_eos = FALSE;
   bool_t      tmp_has_dfg        = FALSE;
   uint32_t    end_offset         = 0;

   // If no data and no flushing eos arrived, there's nothing to do.
   if (((0 == new_data_amount) && (!new_flushing_eos_arrived)) || (!module_ptr))
   {
      return result;
   }

   // Modify md on the module's internal list and the input port's md list.
   TRY(result,
       gen_topo_do_md_offset_math(topo_ptr->t_base.gu.log_id,
                                  &end_offset,
                                  spl_topo_get_in_port_actual_data_len(topo_ptr, in_port_ptr),
                                  in_port_ptr->t_base.common.media_fmt_ptr,
                                  ADD_TRUE));

   TRY(result,
       gen_topo_md_list_modify_md_when_new_data_arrives(&(topo_ptr->t_base),
                                                        &(module_ptr->t_base),
                                                        &(module_ptr->t_base.int_md_list_ptr),
                                                        end_offset,
                                                        &tmp_new_marker_eos,
                                                        &tmp_has_dfg));
   new_marker_eos |= tmp_new_marker_eos;

   TRY(result,
       gen_topo_md_list_modify_md_when_new_data_arrives(&(topo_ptr->t_base),
                                                        &(module_ptr->t_base),
                                                        &(in_port_ptr->t_base.common.sdata.metadata_list_ptr),
                                                        end_offset,
                                                        &tmp_new_marker_eos,
                                                        &tmp_has_dfg));
   new_marker_eos |= tmp_new_marker_eos;

   in_port_ptr->t_base.common.sdata.flags.marker_eos = new_marker_eos;

   // Reset zeros to flush if a new EOS comes in.
   if (new_marker_eos)
   {
      spl_topo_set_eos_zeros_to_flush_bytes(topo_ptr, module_ptr);
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "SPL_TOPO_EOS_DEBUG: Started flushing eos zeros on miid = 0x%lx. %ld "
               "bytes per channel of zeros to flush, module algo delay %ld",
               module_ptr->t_base.gu.module_instance_id,
               module_ptr->t_base.pending_zeros_at_eos,
               module_ptr->t_base.algo_delay);
   }
   // Clear zeros to flush if new data came in. The data will flush the eos (no zeros needed).
   else
   {
      if (0 != module_ptr->t_base.pending_zeros_at_eos)
      {
         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_MED_PRIO,
                  "SPL_TOPO_EOS_DEBUG: Stopped flushing eos zeros on miid = 0x%lx since new data arrived. %ld "
                  "bytes per channel of zeros was pending to flush, module algo delay %ld",
                  module_ptr->t_base.gu.module_instance_id,
                  module_ptr->t_base.pending_zeros_at_eos,
                  module_ptr->t_base.algo_delay);
         module_ptr->t_base.pending_zeros_at_eos = 0;
      }
   }

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
   }
   return result;
}

/**
 * Modifies md when new data arrives for an output port. The metadata search needs to go through the output port
 * md_list_ptr, the downstream connected input port metadata list, and the downstream module's internal list.
 * The downstream module should have pending flushing zeros reset.
 */
ar_result_t spl_topo_op_modify_md_when_new_data_arrives(spl_topo_t *            topo_ptr,
                                                        spl_topo_output_port_t *out_port_ptr,
                                                        uint32_t                new_data_amount,
                                                        bool_t                  new_flushing_eos_arrived)

{
   INIT_EXCEPTION_HANDLING
   ar_result_t             result                      = AR_EOK;
   bool_t                  ADD_TRUE                    = TRUE;
   bool_t                  new_marker_eos              = FALSE;
   bool_t                  tmp_new_marker_eos          = FALSE;
   bool_t                  tmp_has_dfg                 = FALSE;
   uint32_t                end_offset                  = 0;
   spl_topo_module_t *     downstream_module_ptr       = NULL;
   spl_topo_module_t *     upstream_module_ptr         = NULL;
   spl_topo_input_port_t * downstream_in_port_ptr      = NULL;
   spl_topo_output_port_t *downstream_ext_out_port_ptr = NULL;

   // If no data and no flushing eos arrived, there's nothing to do.
   if ((0 == new_data_amount) && (!new_flushing_eos_arrived))
   {
      return result;
   }

   upstream_module_ptr = (spl_topo_module_t *)out_port_ptr->t_base.gu.cmn.module_ptr;

   // Modify md on the output port's md list and the downstream connected input port's md list and downstream module's
   // internal md list.
   spl_topo_get_connected_int_or_ext_ip_port(topo_ptr,
                                             out_port_ptr,
                                             &downstream_in_port_ptr,
                                             &downstream_ext_out_port_ptr);

   TRY(result,
       gen_topo_do_md_offset_math(topo_ptr->t_base.gu.log_id,
                                  &end_offset,
                                  spl_topo_get_out_port_actual_data_len(topo_ptr, out_port_ptr),
                                  out_port_ptr->t_base.common.media_fmt_ptr,
                                  ADD_TRUE));

   // Only operate on the downstream module/input port if there is a downstream module.
   if (downstream_in_port_ptr)
   {
      downstream_module_ptr = (spl_topo_module_t *)downstream_in_port_ptr->t_base.gu.cmn.module_ptr;

      TRY(result,
          gen_topo_md_list_modify_md_when_new_data_arrives(&(topo_ptr->t_base),
                                                           &(downstream_module_ptr->t_base),
                                                           &(downstream_module_ptr->t_base.int_md_list_ptr),
                                                           end_offset,
                                                           &tmp_new_marker_eos,
                                                           &tmp_has_dfg));
      new_marker_eos |= tmp_new_marker_eos;

      TRY(result,
          gen_topo_md_list_modify_md_when_new_data_arrives(&(topo_ptr->t_base),
                                                           &(downstream_module_ptr->t_base),
                                                           &(downstream_in_port_ptr->t_base.common.sdata
                                                                .metadata_list_ptr),
                                                           end_offset,
                                                           &tmp_new_marker_eos,
                                                           &tmp_has_dfg));
      new_marker_eos |= tmp_new_marker_eos;
   }

   TRY(result,
       gen_topo_md_list_modify_md_when_new_data_arrives(&(topo_ptr->t_base),
                                                        (downstream_module_ptr ? &(downstream_module_ptr->t_base)
                                                                               : &(upstream_module_ptr->t_base)),
                                                        &(out_port_ptr->md_list_ptr),
                                                        end_offset,
                                                        &tmp_new_marker_eos,
                                                        &tmp_has_dfg));

   new_marker_eos |= tmp_new_marker_eos;

   // Assign output port's marker eos accordingly.
   out_port_ptr->t_base.common.sdata.flags.marker_eos = new_marker_eos;

   // If the downstream input port exists, assign the marker eos and reset the pending zeros to flush.
   if (downstream_in_port_ptr && downstream_module_ptr)
   {
      downstream_in_port_ptr->t_base.common.sdata.flags.marker_eos = new_marker_eos;

      // Reset zeros to flush if a new EOS comes in.
      if (new_marker_eos)
      {
         spl_topo_set_eos_zeros_to_flush_bytes(topo_ptr, downstream_module_ptr);

         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_MED_PRIO,
                  "SPL_TOPO_EOS_DEBUG: Started flushing eos zeros on miid = 0x%lx. %ld "
                  "bytes per channel of zeros to flush, module algo delay %ld",
                  downstream_module_ptr->t_base.gu.module_instance_id,
                  downstream_module_ptr->t_base.pending_zeros_at_eos,
                  downstream_module_ptr->t_base.algo_delay);
      }
      // Clear zeros to flush if new data came in. The data will flush the eos (no zeros needed).
      else
      {
         if (0 != downstream_module_ptr->t_base.pending_zeros_at_eos)
         {
            TOPO_MSG(topo_ptr->t_base.gu.log_id,
                     DBG_MED_PRIO,
                     "SPL_TOPO_EOS_DEBUG: Stopped flushing eos zeros on miid = 0x%lx since new data arrived. %ld "
                     "bytes per channel of zeros was pending to flush, module algo delay %ld",
                     downstream_module_ptr->t_base.gu.module_instance_id,
                     downstream_module_ptr->t_base.pending_zeros_at_eos,
                     downstream_module_ptr->t_base.algo_delay);
            downstream_module_ptr->t_base.pending_zeros_at_eos = 0;
         }
      }
   }

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->t_base.gu.log_id)
   {
   }
   return result;
}
