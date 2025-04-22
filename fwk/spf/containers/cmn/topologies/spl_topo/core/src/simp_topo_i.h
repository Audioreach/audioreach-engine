/**
 * \file simp_topo_i.h
 *
 * \brief
 *
 *     Internal header file for the simplified topo.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef SIMP_TOPO_I_H_
#define SIMP_TOPO_I_H_

#include "spl_topo_i.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**-------------------------------- simp_topo_utils ------------------------------*/
ar_result_t simp_topo_setup_ext_in_port_sdata(spl_topo_t *topo_ptr, spl_topo_input_port_t *in_port_ptr);
ar_result_t simp_topo_setup_proc_ctx_out_port(spl_topo_t *topo_ptr, spl_topo_output_port_t *out_port_ptr);

ar_result_t simp_topo_adjust_int_in_buf(spl_topo_t *            topo_ptr,
                                        spl_topo_input_port_t * in_port_ptr,
                                        spl_topo_output_port_t *connected_out_port_ptr);
ar_result_t simp_topo_adjust_ext_in_buf(spl_topo_t *           topo_ptr,
                                        spl_topo_input_port_t *in_port_ptr,
                                        spl_topo_ext_buf_t *   ext_buf_ptr);

void simp_topo_adjust_int_out_buf(spl_topo_t *topo_ptr, spl_topo_output_port_t *out_port_pt);
ar_result_t simp_topo_adjust_ext_out_buf(spl_topo_t *            topo_ptr,
                                         spl_topo_output_port_t *out_port_ptr,
                                         spl_topo_ext_buf_t *    ext_buf_ptr);
void simp_topo_adjust_out_buf_md(spl_topo_t *            topo_ptr,
                                 spl_topo_output_port_t *out_port_ptr,
                                 uint32_t                amount_data_generated);
/**-------------------------------- inline functions ------------------------------*/

static inline uint32_t simp_topo_sdata_flag_get_stream_version_mask()
{
	capi_stream_flags_t temp_flag;
	temp_flag.word = 0xFFFFFFFF;
	temp_flag.stream_data_version = 0;
	return (~temp_flag.word);
}

static inline uint32_t simp_topo_sdata_flag_get_marker_eos_mask()
{
	capi_stream_flags_t temp_flag;
	temp_flag.word = 0xFFFFFFFF;
	temp_flag.marker_eos = 0;
	return (~temp_flag.word);
}


/*
 * Utility function that copies sdata flags & timestamp but not buf_ptr or bufs_num
 *
 * This function doesn't touch the marker_eos, since that gets transferred along with eos md.
 */
static inline void simp_topo_copy_sdata_flags(capi_stream_data_v2_t *sdata_to_ptr, capi_stream_data_v2_t *sdata_from_ptr)
{
   sdata_to_ptr->flags     = sdata_from_ptr->flags;
   sdata_to_ptr->timestamp = sdata_from_ptr->timestamp;
   return;
}

/*
 * Utility function that clear sdata flags.
 *
 * This function doesn't touch the marker_eos, since that gets transferred along with eos md.
 */
static inline void simp_topo_clear_sdata_flags(capi_stream_data_v2_t *sdata_ptr)
{
   uint32_t clear_mask = (simp_topo_sdata_flag_get_stream_version_mask() | simp_topo_sdata_flag_get_marker_eos_mask());
   sdata_ptr->flags.word &= clear_mask;

   return;
}

/**
 * Do a framework estimation of how the input timestamp would be modified during process() call
 * of module_ptr, and assign into the output timestamp. The framework estimation subtracts the
 * module's algorithmic delay from the input timestamp. Why subtract? The module will push
 * zeros equal to its amount of algorithmic delay, and the output timestamp will correspond
 * to the first zero pushed.
 */
static ar_result_t simp_topo_fwk_estimate_process_timestamp(spl_topo_t *           topo_ptr,
                                                            spl_topo_module_t *    module_ptr,
                                                            capi_stream_data_v2_t *in_port_sdata_ptr,
                                                            capi_stream_data_v2_t *out_port_sdata_ptr)
{
   out_port_sdata_ptr->timestamp                = in_port_sdata_ptr->timestamp - (module_ptr->t_base.algo_delay);
   out_port_sdata_ptr->flags.is_timestamp_valid = in_port_sdata_ptr->flags.is_timestamp_valid;

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_MED_PRIO,
            "estimate process timestamp, algo delay microseconds: %lld, input ts = %lld, is_valid = %ld, output ts = "
            "%lld, is_valid = %ld",
            module_ptr->t_base.algo_delay,
            in_port_sdata_ptr->timestamp,
            in_port_sdata_ptr->flags.is_timestamp_valid,
            out_port_sdata_ptr->timestamp,
            out_port_sdata_ptr->flags.is_timestamp_valid);
#endif

   return AR_EOK;
}

/**
 * Setup sdata bufs before process. For input, sdata buffers should point to the beginning
 * of data.
 * - in SPL_CNTR only deinterleaved unpacked is accepted.
 * - in SPL_CNTR for input ports, buffers live in the connected output ports, so use the connected output
 *   ports to adjust ptrs and then transfer over to the input port.
 */
static inline ar_result_t simp_topo_populate_int_input_sdata_bufs(spl_topo_t *            topo_ptr,
                                                                  spl_topo_input_port_t * in_port_ptr,
                                                                  spl_topo_output_port_t *connected_out_port_ptr)

{
   ar_result_t result = AR_EOK;

#if SAFE_MODE
   // Verifying mf from either the output port itself or the input port's connected output port.
   if (!((SPF_IS_PCM_DATA_FORMAT(in_port_ptr->t_base.common.media_fmt_ptr->data_format)) &&
         TU_IS_ANY_DEINTERLEAVED_UNPACKED(in_port_ptr->t_base.common.media_fmt_ptr->pcm.interleaving)))
   {
      gen_topo_module_t *module_ptr = (gen_topo_module_t *)&in_port_ptr->t_base.gu.cmn.module_ptr;
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_ERROR_PRIO,
               "Failed in setting up the sdata for module 0x%x. Media format is not PCM deinterleaved.",
               module_ptr->gu.module_instance_id);
      return AR_EFAILED;
   }
#endif

   gen_topo_process_context_t *pc_ptr = &topo_ptr->t_base.proc_context;
   capi_buf_t                 *bufs   = pc_ptr->in_port_scratch_ptr[in_port_ptr->t_base.gu.cmn.index].bufs;

   // populate sdata for unpacked and interleaved data.
#ifdef SAFE_MODE_SDATA_BUF_LENGTHS
   for (uint32_t b = 0; b < connected_out_port_ptr->t_base.common.sdata.bufs_num; b++)
   {
      bufs[b].data_ptr        = connected_out_port_ptr->t_base.common.bufs_ptr[b].data_ptr;
      bufs[b].actual_data_len = connected_out_port_ptr->t_base.common.bufs_ptr[b].actual_data_len;
      bufs[b].max_data_len    = connected_out_port_ptr->t_base.common.bufs_ptr[b].max_data_len;
   }
#else
   for (uint32_t b = 0; b < connected_out_port_ptr->t_base.common.sdata.bufs_num; b++)
   {
      bufs[b].data_ptr = connected_out_port_ptr->t_base.common.bufs_ptr[b].data_ptr;
   }
   bufs[0].actual_data_len = connected_out_port_ptr->t_base.common.bufs_ptr[0].actual_data_len;
   bufs[0].max_data_len    = connected_out_port_ptr->t_base.common.bufs_ptr[0].max_data_len;
#endif

   // for SC, always use process context bufs. we cannot use connected out port bufs_ptr because if module doesn't
   // consume data, then using same bufs_ptr we cannot track how much data belongs to connected out port and how much to
   // input. p_eq_nt_14.
   in_port_ptr->t_base.common.bufs_ptr       = (topo_buf_t *)bufs;
   in_port_ptr->t_base.common.sdata.buf_ptr  = bufs;
   in_port_ptr->t_base.common.sdata.bufs_num = connected_out_port_ptr->t_base.common.sdata.bufs_num;
   return result;
}

/**
 * Used right before calling capi v2 process(), to track global command state.
 */
static inline void simp_topo_set_process_begin(spl_topo_t *topo_ptr)
{
   topo_ptr->cmd_state = TOPO_CMD_STATE_PROCESS;
   return;
}

/**
 * Used right after calling capi v2 process(), to clear global command state.
 */
static inline void simp_topo_set_process_end(spl_topo_t *topo_ptr)
{
   topo_ptr->cmd_state = TOPO_CMD_STATE_DEFAULT;
   return;
}

static void simp_topo_bypass_input_to_output(capi_stream_data_t *input,
                                             capi_stream_data_t *output)
{
   /**
    * Memscpy for current module.
    */
   uint32_t num_bufs = MIN(input[0].bufs_num, output[0].bufs_num);
   uint32_t copy_len = 0;

#ifdef SAFE_MODE_SDATA_BUF_LENGTHS
   for (uint32_t ch_idx = 0; ch_idx < num_bufs; ch_idx++)
   {
      if (input[0].buf_ptr[ch_idx].data_ptr == output[0].buf_ptr[ch_idx].data_ptr)
      {
         copy_len = MIN(output[0].buf_ptr[ch_idx].max_data_len, input[0].buf_ptr[ch_idx].actual_data_len);
      }
      else if (input[0].buf_ptr[ch_idx].data_ptr && output[0].buf_ptr[ch_idx].data_ptr)
      {
         copy_len = memsmove(output[0].buf_ptr[ch_idx].data_ptr,
                             output[0].buf_ptr[ch_idx].max_data_len,
                             input[0].buf_ptr[ch_idx].data_ptr,
                             input[0].buf_ptr[ch_idx].actual_data_len);
      }

      input[0].buf_ptr[ch_idx].actual_data_len  = copy_len;
      output[0].buf_ptr[ch_idx].actual_data_len = copy_len;
   }
#else
   uint32_t in_actual_len  = input[0].buf_ptr[0].actual_data_len;
   uint32_t out_max_len    = output[0].buf_ptr[0].max_data_len;

   copy_len = MIN(out_max_len, in_actual_len);

   for (uint32_t ch_idx = 0; ch_idx < num_bufs; ch_idx++)
   {
      memsmove(output[0].buf_ptr[ch_idx].data_ptr, out_max_len, input[0].buf_ptr[ch_idx].data_ptr, in_actual_len);
   }

   input[0].buf_ptr[0].actual_data_len  = copy_len;
   output[0].buf_ptr[0].actual_data_len = copy_len;
#endif
}

/**
 * Calls the module's process() vtable function using input and output sdata arguments
 * from the topo_ptr->proc_context.
 */
static ar_result_t simp_topo_process_module(spl_topo_t *topo_ptr, spl_topo_module_t *module_ptr)
{
#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_3
   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_MED_PRIO,
            "Topo entering process, module id 0x%8x",
            module_ptr->t_base.gu.module_instance_id);
#endif

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
   for (gu_input_port_list_t *in_port_list_ptr = module_ptr->t_base.gu.input_port_list_ptr; in_port_list_ptr;
        LIST_ADVANCE(in_port_list_ptr))
   {
      gu_input_port_t *in_port_ptr = in_port_list_ptr->ip_port_ptr;

      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "Input miid 0x%lx idx %ld timestamp before process: ts = %ld, is_valid = %ld",
               module_ptr->t_base.gu.module_instance_id,
               in_port_ptr->cmn.index,
               (int32_t)topo_ptr->t_base.proc_context.in_port_sdata_pptr[in_port_ptr->cmn.index]->timestamp,
               topo_ptr->t_base.proc_context.in_port_sdata_pptr[in_port_ptr->cmn.index]->flags.is_timestamp_valid);
   }

   for (gu_output_port_list_t *out_port_list_ptr = module_ptr->t_base.gu.output_port_list_ptr; out_port_list_ptr;
        LIST_ADVANCE(out_port_list_ptr))
   {
      gu_output_port_t *out_port_ptr = out_port_list_ptr->op_port_ptr;
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "Output miid 0x%lx idx %ld timestamp before process: ts = %ld, is_valid = %ld, (prev cached ts = %ld, "
               "is valid = %ld)",
               module_ptr->t_base.gu.module_instance_id,
               out_port_ptr->cmn.index,
               (int32_t)topo_ptr->t_base.proc_context.out_port_sdata_pptr[out_port_ptr->cmn.index]->timestamp,
               topo_ptr->t_base.proc_context.out_port_sdata_pptr[out_port_ptr->cmn.index]->flags.is_timestamp_valid,
               (int32_t)topo_ptr->t_base.proc_context.out_port_scratch_ptr[out_port_ptr->cmn.index].timestamp,
               topo_ptr->t_base.proc_context.out_port_scratch_ptr[out_port_ptr->cmn.index].flags.is_timestamp_valid);
   }
#endif

   simp_topo_set_process_begin(topo_ptr);
   capi_err_t proc_result;
   // clang-format off
   IRM_PROFILE_MOD_PROCESS_SECTION(module_ptr->t_base.prof_info_ptr, topo_ptr->t_base.gu.prof_mutex,
   proc_result = module_ptr->t_base.capi_ptr->vtbl_ptr
                     ->process(module_ptr->t_base.capi_ptr,
                              (capi_stream_data_t **)topo_ptr->t_base.proc_context.in_port_sdata_pptr,
                              (capi_stream_data_t **)topo_ptr->t_base.proc_context.out_port_sdata_pptr);
   );
   // clang-format on
   // Ignore need more.

   proc_result &= (~CAPI_ENEEDMORE);

   simp_topo_set_process_end(topo_ptr);

   // Check fail case, capi returns error.
   if (CAPI_FAILED(proc_result))
   {
#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
      // Check if eof marker was sent on any output ports.
      bool_t eof_sent = FALSE;
      for (gu_output_port_list_t *out_port_list_ptr = module_ptr->t_base.gu.output_port_list_ptr; out_port_list_ptr;
           LIST_ADVANCE(out_port_list_ptr))
      {
         spl_topo_output_port_t *out_port_ptr = (spl_topo_output_port_t *)out_port_list_ptr->op_port_ptr;

         eof_sent |= out_port_ptr->t_base.common.sdata.flags.end_of_frame;

         if (eof_sent)
         {
            TOPO_MSG(topo_ptr->t_base.gu.log_id,
                     DBG_MED_PRIO,
                     "Eof set after process for output port idx = %ld, miid = 0x%lx",
                     out_port_ptr->t_base.gu.cmn.index,
                     out_port_ptr->t_base.gu.cmn.module_ptr->module_instance_id);
            break;
         }
      }
#endif

      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_ERROR_PRIO,
               "Capi v2 module with mid 0x%8x returned error with code 0x%8x",
               module_ptr->t_base.gu.module_instance_id,
               proc_result);
      return AR_EFAILED;
   }

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_3
   TOPO_MSG(topo_ptr->t_base.gu.log_id,
            DBG_MED_PRIO,
            "Topo leaving process, module id 0x%8x",
            module_ptr->t_base.gu.module_instance_id);
#endif

#if SPL_TOPO_DEBUG_LEVEL >= SPL_TOPO_DEBUG_LEVEL_4
   for (gu_input_port_list_t *in_port_list_ptr = module_ptr->t_base.gu.input_port_list_ptr; in_port_list_ptr;
        LIST_ADVANCE(in_port_list_ptr))
   {
      gu_input_port_t *in_port_ptr = in_port_list_ptr->ip_port_ptr;
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "Input miid 0x%lx idx %ld timestamp after process: ts = %lld, is_valid = %ld",
               module_ptr->t_base.gu.module_instance_id,
               in_port_ptr->cmn.index,
               topo_ptr->t_base.proc_context.in_port_sdata_pptr[in_port_ptr->cmn.index]->timestamp,
               topo_ptr->t_base.proc_context.in_port_sdata_pptr[in_port_ptr->cmn.index]->flags.is_timestamp_valid);
   }

   for (gu_output_port_list_t *out_port_list_ptr = module_ptr->t_base.gu.output_port_list_ptr; out_port_list_ptr;
        LIST_ADVANCE(out_port_list_ptr))
   {
      gu_output_port_t *out_port_ptr = out_port_list_ptr->op_port_ptr;
      TOPO_MSG(topo_ptr->t_base.gu.log_id,
               DBG_MED_PRIO,
               "Output miid 0x%lx idx %ld timestamp after process: ts = %lld, is_valid = %ld",
               module_ptr->t_base.gu.module_instance_id,
               out_port_ptr->cmn.index,
               topo_ptr->t_base.proc_context.out_port_sdata_pptr[out_port_ptr->cmn.index]->timestamp,
               topo_ptr->t_base.proc_context.out_port_sdata_pptr[out_port_ptr->cmn.index]->flags.is_timestamp_valid);
   }
#endif

#ifdef SAFE_MODE
   // Check fail cases, capi writes input actual data len > max data len.
   for (gu_input_port_list_t *in_port_list_ptr = module_ptr->t_base.gu.input_port_list_ptr; in_port_list_ptr;
        LIST_ADVANCE(in_port_list_ptr))
   {
      gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

      if (!(topo_ptr->t_base.proc_context.in_port_sdata_pptr[in_port_ptr->gu.cmn.index]->buf_ptr))
      {
         continue;
      }

      if (topo_ptr->t_base.proc_context.in_port_sdata_pptr[in_port_ptr->gu.cmn.index]->buf_ptr[0].actual_data_len >
          topo_ptr->t_base.proc_context.in_port_sdata_pptr[in_port_ptr->gu.cmn.index]->buf_ptr[0].max_data_len)
      {
         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_ERROR_PRIO,
                  "Capi v2 module with mid 0x%8x has input idx %ld actual data len %ld greater "
                  "than max data len %ld",
                  module_ptr->t_base.gu.module_instance_id,
                  in_port_ptr->gu.cmn.index,
                  topo_ptr->t_base.proc_context.in_port_sdata_pptr[in_port_ptr->gu.cmn.index]
                     ->buf_ptr[0]
                     .actual_data_len,
                  topo_ptr->t_base.proc_context.in_port_sdata_pptr[in_port_ptr->gu.cmn.index]->buf_ptr[0].max_data_len);
         return AR_EFAILED;
      }
   }

   // Check fail cases, capi writes output actual data len > max data len.
   for (gu_output_port_list_t *out_port_list_ptr = module_ptr->t_base.gu.output_port_list_ptr; out_port_list_ptr;
        LIST_ADVANCE(out_port_list_ptr))
   {
      // Check here if output samples didn't match in dm mode, if not reset to zero so things can be updated again.
      gu_output_port_t *out_port_ptr = out_port_list_ptr->op_port_ptr;

      if (!(topo_ptr->t_base.proc_context.out_port_sdata_pptr[out_port_ptr->cmn.index]->buf_ptr))
      {
         continue;
      }

      if (topo_ptr->t_base.proc_context.out_port_sdata_pptr[out_port_ptr->cmn.index]->buf_ptr[0].actual_data_len >
          topo_ptr->t_base.proc_context.out_port_sdata_pptr[out_port_ptr->cmn.index]->buf_ptr[0].max_data_len)
      {
         TOPO_MSG(topo_ptr->t_base.gu.log_id,
                  DBG_ERROR_PRIO,
                  "Capi v2 module with mid 0x%8x has output idx %ld actual data len %ld greater "
                  "than max data len %ld",
                  module_ptr->t_base.gu.module_instance_id,
                  out_port_ptr->cmn.index,
                  topo_ptr->t_base.proc_context.out_port_sdata_pptr[out_port_ptr->cmn.index]
                     ->buf_ptr[0]
                     .actual_data_len,
                  topo_ptr->t_base.proc_context.out_port_sdata_pptr[out_port_ptr->cmn.index]->buf_ptr[0].max_data_len);
         return AR_EFAILED;
      }
   }
#endif

   return AR_EOK;
}




#ifdef __cplusplus
}
#endif //__cplusplus

#endif //SIMP_TOPO_I_H_
