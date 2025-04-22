/**
 * \file wc_topo_data_process.c
 * \brief
 *     This file contains functions for topo common data processing
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "wc_topo.h"
#include "wc_topo_capi.h"


static ar_result_t wcntr_topo_module_process(wcntr_topo_t *topo_ptr, wcntr_topo_module_t *module_ptr);

static ar_result_t wcntr_topo_update_buf_len_ts_after_process(wcntr_topo_t *            topo_ptr,
                                                            wcntr_topo_output_port_t *out_port_pt);


uint32_t wcntr_topo_get_bufs_num_from_med_fmt(wcntr_topo_media_fmt_t *med_fmt_ptr)
{
   uint32_t bufs_num = 1;
   if (SPF_IS_PACKETIZED_OR_PCM(med_fmt_ptr->data_format))
   {
      if (WCNTR_TOPO_DEINTERLEAVED_UNPACKED == med_fmt_ptr->pcm.interleaving)
      {
         bufs_num = med_fmt_ptr->pcm.num_channels;
      }
   }

   return ((0 == bufs_num) ? 1 : bufs_num);
}

ar_result_t wcntr_topo_initialize_bufs_sdata(wcntr_topo_t *            topo_ptr,
                                           wcntr_topo_common_port_t *cmn_port_ptr,
                                           uint32_t                miid,
                                           uint32_t                port_id,
                                           bool_t update_data_ptr,
                                           int8_t *data_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

   if ((data_ptr == NULL) && update_data_ptr)
   {
      WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                     DBG_ERROR_PRIO,
                     "initialize_bufs: Module 0x%lX, port_id 0x%lx"
                     "data_ptr cannot be NULL",
                     miid,
                     port_id);
      return AR_EFAILED;
   }

   cmn_port_ptr->sdata.flags.stream_data_version = CAPI_STREAM_V2;

   uint32_t new_bufs_num = wcntr_topo_get_bufs_num_from_med_fmt(&cmn_port_ptr->media_fmt);

   if (cmn_port_ptr->sdata.bufs_num != new_bufs_num)
   {
#ifdef VERBOSE_LOGGING	   
      uint32_t    old_bufs_num     = cmn_port_ptr->sdata.bufs_num;
      capi_buf_t *old_buf_ptr      = (capi_buf_t *)cmn_port_ptr->bufs_ptr;
#endif	
      cmn_port_ptr->sdata.bufs_num = new_bufs_num;

      MFREE_NULLIFY(cmn_port_ptr->bufs_ptr);

      if (0 != cmn_port_ptr->sdata.bufs_num)
      {
         MALLOC_MEMSET(cmn_port_ptr->bufs_ptr,
                       wcntr_topo_buf_t,
                       cmn_port_ptr->sdata.bufs_num * sizeof(wcntr_topo_buf_t),
                       topo_ptr->heap_id,
                       result);
#ifdef VERBOSE_LOGGING
         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_LOW_PRIO,
                        "initialize_bufs: Module 0x%lX, port_id 0x%lx, (num_bufs,bufs_ptr) FREEING old (%u,0x%lX) , "
                        "MALLOCD new (%u,0x%lX)",
                        miid,
                        port_id,
                        old_bufs_num,
                        old_buf_ptr,new_bufs_num,
                        cmn_port_ptr->bufs_ptr);
#endif						
      }
   }

   cmn_port_ptr->sdata.buf_ptr = (capi_buf_t *)cmn_port_ptr->bufs_ptr;
   // max_buf_len may be assigned after assigning bufs_num.
   cmn_port_ptr->max_buf_len_per_buf = (cmn_port_ptr->max_buf_len / cmn_port_ptr->sdata.bufs_num);
#ifdef VERBOSE_LOGGING
   WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                  DBG_LOW_PRIO,
                  "initialize_bufs: Module 0x%lX, cmn_port_ptr 0x%lX,port_id 0x%lx, bufs_num %lu, "
                  "max_buf_len_per_buf %lu, bufs_ptr 0x%lX",
                  miid,
                  cmn_port_ptr,
                  port_id,
                  cmn_port_ptr->sdata.bufs_num,
                  cmn_port_ptr->max_buf_len_per_buf,
                  cmn_port_ptr->bufs_ptr);
#endif

   for (uint32_t b = 0; b < cmn_port_ptr->sdata.bufs_num; b++)
   {
      cmn_port_ptr->bufs_ptr[b].data_ptr        = data_ptr + (b * cmn_port_ptr->max_buf_len_per_buf);
      cmn_port_ptr->bufs_ptr[b].actual_data_len = 0;
      cmn_port_ptr->bufs_ptr[b].max_data_len    = cmn_port_ptr->max_buf_len_per_buf;

      WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                     DBG_LOW_PRIO,
                     "initialize_bufs: Module 0x%lX, port_id 0x%lx, total bufs_num %lu, "
                     "current buf %u, data_ptr 0x%lX max_data_len %u",
                     miid,
                     port_id,
                     cmn_port_ptr->sdata.bufs_num,
                     b,
                     cmn_port_ptr->bufs_ptr[b].data_ptr,
                     cmn_port_ptr->bufs_ptr[b].max_data_len);
   }

   if (AR_DID_FAIL(result))
   {
      WCNTR_TOPO_MSG(topo_ptr->gu.log_id, DBG_ERROR_PRIO, "failure in wcntr_topo_initialize_bufs_sdata");
   }

   CATCH(result, WCNTR_TOPO_MSG_PREFIX, topo_ptr->gu.log_id)
   {

      WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                     DBG_HIGH_PRIO,
                     "failure in wcntr_topo_initialize_bufs_sdata with result 0x%lx",
                     result);
   }
   return result;
}

static capi_err_t wcntr_topo_copy_input_to_output(wcntr_topo_t *        topo_ptr,
                                                wcntr_topo_module_t * module_ptr,
                                                capi_stream_data_t *inputs[],
                                                capi_stream_data_t *outputs[])
{
   capi_err_t result = CAPI_EOK;

   wcntr_gu_input_port_list_t * in_port_list_ptr = module_ptr->gu.input_port_list_ptr;
   wcntr_topo_input_port_t *in_port_ptr      = (wcntr_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;
   uint32_t               ip_idx           = in_port_ptr->gu.cmn.index;

   // memcpy input to output
   // works for only one input port. for metadata only one output port must be present (as we are not cloning here).
   if ((1 != module_ptr->gu.num_input_ports) || (1 != module_ptr->gu.num_output_ports))
   {
      WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
               DBG_ERROR_PRIO,
               " Module 0x%lX: for memcpy from input to output, only one input or output must be present ",
               module_ptr->gu.module_instance_id);
      return CAPI_EFAILED;
   }

   for (wcntr_gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr; (NULL != out_port_list_ptr);
        LIST_ADVANCE(out_port_list_ptr))
   {
      // Copy available data as is. Don't check for threshold or for media-format.
      // this is inline with the behavior of requires_data_buf = FALSE & port_has_no_threshold
      wcntr_topo_output_port_t *out_port_ptr = (wcntr_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
      uint32_t                op_idx       = out_port_ptr->gu.cmn.index;
      if (inputs[ip_idx]->bufs_num != outputs[ip_idx]->bufs_num)
      {
         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                  DBG_ERROR_PRIO,
                  " Module 0x%lX: bufs_num must match between input[0x%lx] (%lu) and output[0x%lx] (%lu)",
                  module_ptr->gu.module_instance_id,
                  in_port_ptr->gu.cmn.id,
                  inputs[ip_idx]->bufs_num,
                  out_port_ptr->gu.cmn.id,
                  outputs[ip_idx]->bufs_num);
      }

      for (uint32_t i = 0; i < MIN(inputs[ip_idx]->bufs_num, outputs[ip_idx]->bufs_num); i++)
      {
         if (inputs[ip_idx]->buf_ptr[i].data_ptr && outputs[op_idx]->buf_ptr[i].data_ptr)
         {
            outputs[op_idx]->buf_ptr[i].actual_data_len = memsmove(outputs[op_idx]->buf_ptr[i].data_ptr,
                                                                   outputs[op_idx]->buf_ptr[i].max_data_len,
                                                                   inputs[ip_idx]->buf_ptr[i].data_ptr,
                                                                   inputs[ip_idx]->buf_ptr[i].actual_data_len);

            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                     DBG_HIGH_PRIO,
                     " Module 0x%lX: copied channel (%lu) data ",
                     module_ptr->gu.module_instance_id,
                     i);
         }
      }
   }

   return result;
}

/**
 * events raised at process-call by the modules
 * framework addresses the events and calls the modules again.
 */
static bool_t wcntr_topo_any_process_call_events(wcntr_topo_t *topo_ptr)
{

   return (topo_ptr->capi_event_flag.media_fmt_event || topo_ptr->capi_event_flag.port_thresh ||
           topo_ptr->capi_event_flag.process_state);
}

/**
 * Must not return without calling gen_topo_return_buf_mgr_buf if call to get buf was made.
 * Calling gen_topo_return_buf_mgr_buf ensures that buf is only a the last module output ptr.
 */
ar_result_t wcntr_topo_topo_process(wcntr_topo_t *topo_ptr, wcntr_gu_module_list_t **start_module_list_pptr)
{
   ar_result_t result = AR_EOK;
#ifdef VERBOSE_LOGGING
   WCNTR_TOPO_MSG(topo_ptr->gu.log_id, DBG_LOW_PRIO, " wcntr_topo_topo_process START ");
#endif
   for (wcntr_gu_module_list_t *module_list_ptr = *start_module_list_pptr; (NULL != module_list_ptr);
        LIST_ADVANCE(module_list_ptr))
   {
      wcntr_topo_module_t *module_ptr = (wcntr_topo_module_t *)module_list_ptr->module_ptr;

      if (!module_ptr->can_process_be_called)
      {
#ifdef VERBOSE_LOGGING
		 WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_LOW_PRIO,
                        " wcntr_topo_topo_process SKIPPING PROCESS for mid 0x%X ",
                        module_ptr->gu.module_instance_id);
#endif
         continue;
      }

      result |= wcntr_topo_module_process(topo_ptr, module_ptr);
   }
#ifdef VERBOSE_LOGGING
   WCNTR_TOPO_MSG(topo_ptr->gu.log_id, DBG_LOW_PRIO, " wcntr_topo_topo_process END result 0x%X",result);
#endif

   return result;
}

static inline void wcntr_topo_reset_process_context_sdata(wcntr_topo_process_context_t *pc, wcntr_topo_module_t *module_ptr)
{
   for (uint32_t ip_idx = 0; ip_idx < module_ptr->gu.max_input_ports; ip_idx++)
   {
      pc->in_port_sdata_pptr[ip_idx] = NULL;
   }

   for (uint32_t op_idx = 0; op_idx < module_ptr->gu.max_output_ports; op_idx++)
   {
      pc->out_port_sdata_pptr[op_idx] = NULL;
   }
}


/**
 * return code includes need_more
 */
static ar_result_t wcntr_topo_module_process(wcntr_topo_t *topo_ptr, wcntr_topo_module_t *module_ptr)
{
   capi_err_t result = CAPI_EOK;
   uint32_t   op_idx = 0;
   uint32_t   ip_idx = 0;
   uint32_t   m_iid  = module_ptr->gu.module_instance_id;

   wcntr_topo_process_context_t *pc           = &topo_ptr->proc_context;
   wcntr_topo_input_port_t *     in_port_ptr  = NULL;
   wcntr_topo_output_port_t *    out_port_ptr = NULL;

   uint32_t capi_event_flags_before_process = topo_ptr->capi_event_flag.word;
   
#ifdef VERBOSE_LOGGING
   WCNTR_TOPO_MSG(topo_ptr->gu.log_id, DBG_HIGH_PRIO, " m_iid 0x%X topo_module_process START", m_iid);
#endif
   wcntr_topo_reset_process_context_sdata(pc, module_ptr);

   for (wcntr_gu_input_port_list_t *in_port_list_ptr = module_ptr->gu.input_port_list_ptr; (NULL != in_port_list_ptr);
        LIST_ADVANCE(in_port_list_ptr))
   {
      in_port_ptr = (wcntr_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;
      ip_idx      = in_port_ptr->gu.cmn.index;
      
#ifdef VERBOSE_LOGGING
      for (uint32_t b = 0; b < in_port_ptr->common.sdata.bufs_num; b++)
      {
         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_LOW_PRIO,
                        " Module 0x%lX: input port id 0x%lx, process before: length_per_buf %lu of %lu. buff "
                        "addr 0x%p",
                        m_iid,
                        in_port_ptr->gu.cmn.id,
                        in_port_ptr->common.bufs_ptr[b].actual_data_len,
                        in_port_ptr->common.bufs_ptr[b].max_data_len,
                        in_port_ptr->common.bufs_ptr[b].data_ptr);
      }
#endif	  

      pc->in_port_sdata_pptr[ip_idx] = &in_port_ptr->common.sdata;
   }

   for (wcntr_gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr;
        (NULL != out_port_list_ptr);
        LIST_ADVANCE(out_port_list_ptr))
   {
      out_port_ptr = (wcntr_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
      op_idx       = out_port_ptr->gu.cmn.index;

      if (out_port_ptr->common.state != WCNTR_TOPO_PORT_STATE_STARTED)
      {

         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_LOW_PRIO,
                        " Module 0x%lX: output port id 0x%lx, not in START state continue",
                        m_iid,
                        out_port_ptr->gu.cmn.id);
         continue;
      }

      // wcntr_topo_common_port_t *port_common_ptr = &out_port_ptr->common;

      for (uint32_t b = 0; b < out_port_ptr->common.sdata.bufs_num; b++)
      {
         out_port_ptr->common.bufs_ptr[b].actual_data_len = 0;
#ifdef VERBOSE_LOGGING		 
         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_LOW_PRIO,
                        " Module 0x%lX: output port id 0x%lx, ch %u process before: length_per_buf %lu of %lu. buff "
                        "addr 0x%p",
                        m_iid,
                        out_port_ptr->gu.cmn.id,
                        b,
                        out_port_ptr->common.bufs_ptr[b].actual_data_len,
                        out_port_ptr->common.bufs_ptr[b].max_data_len,
                        out_port_ptr->common.bufs_ptr[b].data_ptr);
#endif
      }

      pc->out_port_sdata_pptr[op_idx] = &out_port_ptr->common.sdata;
   }

   /**
    * At input of process, the input->actual_len is the size of input & data starts from data_ptr.
    *                      the output->actual_len is uninitialized & CAPI can write from data_ptr.
    *                                remaining data is from data_ptr+actual_len
    * At output of process, the input->actual_len is the amount of data consumed (read) by CAPI.
    *                      the output->actual_len is output data, & data starts from data_ptr.
    */

   if (module_ptr->capi_ptr && (!module_ptr->bypass_ptr))
   {
      result = module_ptr->capi_ptr->vtbl_ptr->process(module_ptr->capi_ptr,
                                                       (capi_stream_data_t **)pc->in_port_sdata_pptr,
                                                       (capi_stream_data_t **)pc->out_port_sdata_pptr);
   }
   else // bypass use cases etc
   {

      result = wcntr_topo_copy_input_to_output(topo_ptr,
                                               module_ptr,
                                               (capi_stream_data_t **)pc->in_port_sdata_pptr,
                                               (capi_stream_data_t **)pc->out_port_sdata_pptr);
   }

   bool_t any_runtime_change = wcntr_topo_any_process_call_events(topo_ptr);

   uint32_t capi_event_flags_after_process = topo_ptr->capi_event_flag.word;

   if (any_runtime_change)
   {
      WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                     DBG_ERROR_PRIO,
                     " Run time change in events is not supported. Can be ignored if no change in capi_event_word "
                     "(before %u,after %u) process",
                     capi_event_flags_before_process,
                     capi_event_flags_after_process);
   }

   for (wcntr_gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr;
        (NULL != out_port_list_ptr);
        LIST_ADVANCE(out_port_list_ptr))
   {
      out_port_ptr = (wcntr_topo_output_port_t *)out_port_list_ptr->op_port_ptr;

      if (out_port_ptr->common.state != WCNTR_TOPO_PORT_STATE_STARTED)
      {

         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_LOW_PRIO,
                        " Module 0x%lX: output port id 0x%lx, not in START state continue",
                        m_iid,
                        out_port_ptr->gu.cmn.id);
         continue;
      }

      wcntr_topo_update_buf_len_ts_after_process(topo_ptr, out_port_ptr);
   }
#ifdef VERBOSE_LOGGING
   WCNTR_TOPO_MSG(topo_ptr->gu.log_id, DBG_HIGH_PRIO, " wcntr_topo_module_process m_iid %X END with result 0x%X", m_iid,result);
#endif
   return capi_err_to_ar_result(result);
}

static ar_result_t wcntr_topo_update_buf_len_ts_after_process(wcntr_topo_t *            topo_ptr,
                                                            wcntr_topo_output_port_t *out_port_ptr)
{
   wcntr_topo_process_context_t *pc_ptr                      = &topo_ptr->proc_context;
   wcntr_topo_module_t *         module_ptr                  = (wcntr_topo_module_t *)out_port_ptr->gu.cmn.module_ptr;
   wcntr_topo_media_fmt_t *      med_fmt_ptr                 = &out_port_ptr->common.media_fmt;
   capi_stream_data_v2_t *       sdata_ptr                   = pc_ptr->out_port_sdata_pptr[out_port_ptr->gu.cmn.index];
   uint32_t                      supposed_len_per_ch         = 0;
   wcntr_topo_input_port_t *     curr_mod_in_port_ptr        = NULL;
   wcntr_topo_common_port_t *    curr_mod_in_port_common_ptr = NULL;

   wcntr_topo_input_port_t *next_mod_in_port_ptr = (wcntr_topo_input_port_t *)out_port_ptr->gu.conn_in_port_ptr;

   if (SPF_IS_PCM_DATA_FORMAT(med_fmt_ptr->data_format) &&
       (WCNTR_TOPO_DEINTERLEAVED_UNPACKED == med_fmt_ptr->pcm.interleaving))
   {
      supposed_len_per_ch = sdata_ptr->buf_ptr[0].actual_data_len;

      for (uint32_t ch = 0; ch < med_fmt_ptr->pcm.num_channels; ch++)
      {
         if (supposed_len_per_ch != sdata_ptr->buf_ptr[ch].actual_data_len)
         {
            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_ERROR_PRIO,
                           "Module 0x%lX, Port id 0x%lx, all channels must have same amount of data (interleaving "
                           "issue),"
                           "supposed_len_per_ch = %d and actual_data_len = %d",
                           module_ptr->gu.module_instance_id,
                           out_port_ptr->gu.cmn.id,
                           supposed_len_per_ch,
                           sdata_ptr->buf_ptr[ch].actual_data_len);
            return AR_EFAILED;
         }

         if (next_mod_in_port_ptr)
         {
            next_mod_in_port_ptr->common.bufs_ptr[ch].actual_data_len = sdata_ptr->buf_ptr[ch].actual_data_len;
         }
      }
   }

   if (next_mod_in_port_ptr)
   {

      if (out_port_ptr->common.sdata.timestamp)
      {

         next_mod_in_port_ptr->common.sdata.timestamp = out_port_ptr->common.sdata.timestamp;
         memscpy(&next_mod_in_port_ptr->common.sdata.flags,
                 sizeof(capi_stream_flags_t),
                 &out_port_ptr->common.sdata.flags,
                 sizeof(capi_stream_flags_t));

         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_HIGH_PRIO,
                        "Module  output port id (0x%lX-0x%lX) has updated timestamp. copying to next input sdata TS  "
                        "LSW %u MSW %u ",
                        out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                        out_port_ptr->gu.cmn.id,
                        (uint32_t)next_mod_in_port_ptr->common.sdata.timestamp,
                        (uint32_t)(next_mod_in_port_ptr->common.sdata.timestamp >> 32));
      }
      else // if module didnt update TS, copy from input to output for SISO
      {

         if (module_ptr->gu.input_port_list_ptr)
         {
            curr_mod_in_port_ptr        = (wcntr_topo_input_port_t *)module_ptr->gu.input_port_list_ptr->ip_port_ptr;
            curr_mod_in_port_common_ptr = &curr_mod_in_port_ptr->common;

            if (module_ptr->gu.num_input_ports > 1)
            {

               WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                              DBG_HIGH_PRIO,
                              "Module 0x%lX has more greater than one input port. Updating TS with first input port "
                              "TS ",
                              out_port_ptr->gu.cmn.module_ptr->module_instance_id);
            }
         }

         if ((curr_mod_in_port_common_ptr) && curr_mod_in_port_common_ptr->sdata.timestamp)
         {
            next_mod_in_port_ptr->common.sdata.timestamp =
               curr_mod_in_port_common_ptr->sdata.timestamp - (uint64_t)module_ptr->algo_delay;
            memscpy(&next_mod_in_port_ptr->common.sdata.flags,
                    sizeof(capi_stream_flags_t),
                    &curr_mod_in_port_common_ptr->sdata.flags,
                    sizeof(capi_stream_flags_t));

            WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                           DBG_HIGH_PRIO,
                           "Module  output port id (0x%lX-0x%lX) DIDN'T update timestamp. subtract delay %u TS  LSW %u "
                           "MSW %u ",
                           out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                           out_port_ptr->gu.cmn.id,
                           module_ptr->algo_delay,
                           (uint32_t)next_mod_in_port_ptr->common.sdata.timestamp,
                           (uint32_t)(next_mod_in_port_ptr->common.sdata.timestamp >> 32));
         }
      }
   }
   return AR_EOK;
}
