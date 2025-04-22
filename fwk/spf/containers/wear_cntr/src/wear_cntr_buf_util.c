/**
 * \file wear_cntr_buf_util.c
 * \brief
 *     This file contains utility functions for WCNTR buffer handling
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "wear_cntr_i.h"
#include "apm.h"

/*----------------------------------------------------------------------------
 * Global Data Definitions
 * -------------------------------------------------------------------------*/
static ar_result_t wcntr_check_threshold_of_modules(wcntr_t *me_ptr,
                                                               uint32_t    ep_module_id,
                                                               uint32_t    ep_threshold_in_ms);
static ar_result_t wcntr_recreate_all_buffers(wcntr_t *me_ptr);

static bool_t wcntr_is_module_threshold_ep_threshold_integral_multiples(uint32_t ep_threshold_ms,
                                                                           uint32_t mod_threshold_ms)
{

   if (mod_threshold_ms > ep_threshold_ms)
   {
      return FALSE;
   }

   uint32_t int_factor = ep_threshold_ms / mod_threshold_ms;
   if ((int_factor * mod_threshold_ms) == ep_threshold_ms)
   {
      return TRUE;
   }
   return FALSE;
}

//TODO:remove in final check-in
static ar_result_t wcntr_print_buffers_of_modules(wcntr_t *me_ptr)
{
   ar_result_t result = AR_EOK;

   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_LOW_PRIO, "print_buffers_of_modules  START ");

   for (wcntr_gu_sg_list_t *sg_list_ptr = me_ptr->topo.gu.sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (wcntr_gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         wcntr_gu_module_t *module_ptr = (wcntr_gu_module_t *)module_list_ptr->module_ptr;

         for (wcntr_gu_input_port_list_t *in_port_list_ptr = module_ptr->input_port_list_ptr;
              (NULL != in_port_list_ptr);
              LIST_ADVANCE(in_port_list_ptr))
         {
            wcntr_topo_input_port_t * in_port_ptr  = (wcntr_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;
            wcntr_topo_common_port_t *cmn_port_ptr = &in_port_ptr->common;

            for (uint32_t b = 0; b < cmn_port_ptr->sdata.bufs_num; b++)
            {
               WCNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_LOW_PRIO,
                         "print_buffers_of_modules: Module 0x%lX, IN port_id 0x%lx bufs_num %u "
                         "b %u, data_ptr 0x%p actual_data_len %u max_data_len %u",
                         in_port_ptr->gu.cmn.module_ptr->module_instance_id,
                         in_port_ptr->gu.cmn.id,
                         cmn_port_ptr->sdata.bufs_num,
                         b,
                         cmn_port_ptr->bufs_ptr[b].data_ptr,
                         cmn_port_ptr->bufs_ptr[b].actual_data_len,
                         cmn_port_ptr->bufs_ptr[b].max_data_len);
            }
         }

         for (wcntr_gu_output_port_list_t *out_port_list_ptr = module_ptr->output_port_list_ptr;
              (NULL != out_port_list_ptr);
              LIST_ADVANCE(out_port_list_ptr))
         {
            wcntr_topo_output_port_t *out_port_ptr = (wcntr_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
            wcntr_topo_common_port_t *cmn_port_ptr = &out_port_ptr->common;

            for (uint32_t b = 0; b < cmn_port_ptr->sdata.bufs_num; b++)
            {
               WCNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_LOW_PRIO,
                         "print_buffers_of_modules: Module 0x%lX, OUT port_id 0x%lx bufs_num %u "
                         "b %u, data_ptr 0x%p actual_data_len %u max_data_len %u",
                         out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                         out_port_ptr->gu.cmn.id,
                         cmn_port_ptr->sdata.bufs_num,
                         b,
                         cmn_port_ptr->bufs_ptr[b].data_ptr,
                         cmn_port_ptr->bufs_ptr[b].actual_data_len,
                         cmn_port_ptr->bufs_ptr[b].max_data_len);
            }
         }
      }
   }
   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_LOW_PRIO, "print_buffers_of_modules  END ");
   return result;
}

uint32_t wcntr_topo_get_one_ms_buffer_size_in_bytes(wcntr_topo_common_port_t *port_ptr)
{

   uint32_t size_in_bytes_for_all_ch = 0;

   if (port_ptr->flags.is_mf_valid)
   {
      // Based on assumption that no support for fractional sampling rate
      size_in_bytes_for_all_ch = (port_ptr->media_fmt.pcm.sample_rate / 1000) *
                                 (port_ptr->media_fmt.pcm.bits_per_sample / 8) * port_ptr->media_fmt.pcm.num_channels;
   }
   return size_in_bytes_for_all_ch;
}

ar_result_t wcntr_handle_port_data_thresh_change_event(void *ctx_ptr)
{
   wcntr_t *   me_ptr = (wcntr_t *)ctx_ptr;
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_MED_PRIO,
             " in wcntr_handle_port_data_thresh_change_event thresh event %u, media_fmt_event %u",
             me_ptr->topo.capi_event_flag.port_thresh,
             me_ptr->topo.capi_event_flag.media_fmt_event);

   // Should not hit this as graph open does sanity
   wcntr_topo_module_t *ep_ptr = wcntr_get_stm_module(me_ptr);
   if (NULL == ep_ptr)
   {
      THROW(result, AR_EFAILED);
   }

   uint32_t ep_threshold_in_bytes = 0;
   uint32_t ep_threshold_in_ms    = 0;
   bool_t   is_media_fmt_valid    = FALSE;

   if (ep_ptr->gu.output_port_list_ptr)
   {
      wcntr_topo_output_port_t *out_port_ptr = (wcntr_topo_output_port_t *)ep_ptr->gu.output_port_list_ptr->op_port_ptr;
      is_media_fmt_valid                     = out_port_ptr ? out_port_ptr->common.flags.is_mf_valid : FALSE;
      if (is_media_fmt_valid && out_port_ptr->common.flags.port_has_threshold)
      {
         ep_threshold_in_bytes = wcntr_topo_get_curr_port_threshold(&out_port_ptr->common);
         ep_threshold_in_ms =
            (ep_threshold_in_bytes / wcntr_topo_get_one_ms_buffer_size_in_bytes(&out_port_ptr->common));
      }
      else
      {
         WCNTR_MSG(me_ptr->topo.gu.log_id,
                   DBG_ERROR_PRIO,
                   "data_thresh_change_event. End point Module 0x%lX - output "
                   "is_media_fmt_valid %u out_port_ptr 0x%lX",
                   ep_ptr->gu.module_instance_id,
                   is_media_fmt_valid,
                   out_port_ptr);

         THROW(result, AR_EFAILED);
      }
   }
   else if (ep_ptr->gu.input_port_list_ptr)
   {
      wcntr_topo_input_port_t *in_port_ptr = (wcntr_topo_input_port_t *)ep_ptr->gu.input_port_list_ptr->ip_port_ptr;
      is_media_fmt_valid                   = in_port_ptr ? in_port_ptr->common.flags.is_mf_valid : FALSE;
      if (is_media_fmt_valid && in_port_ptr->common.flags.port_has_threshold)
      {
         ep_threshold_in_bytes = wcntr_topo_get_curr_port_threshold(&in_port_ptr->common);
         ep_threshold_in_ms =
            (ep_threshold_in_bytes / wcntr_topo_get_one_ms_buffer_size_in_bytes(&in_port_ptr->common));
      }
      else
      {
         WCNTR_MSG(me_ptr->topo.gu.log_id,
                   DBG_ERROR_PRIO,
                   "data_thresh_change_event. End point Module 0x%lX - input "
                   "is_media_fmt_valid %u in_port_ptr 0x%lX",
                   ep_ptr->gu.module_instance_id,
                   is_media_fmt_valid,
                   in_port_ptr);

         THROW(result, AR_EFAILED);
      }
   }

   // Thread prio uses microseconds. so multiply by 1000
   me_ptr->cu.cntr_proc_duration     = ep_threshold_in_ms * 1000;
   me_ptr->op_frame_in_ms = ep_threshold_in_ms;

   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_MED_PRIO,
             " in wcntr_handle_port_data_thresh_change_event. ep_threshold_in_bytes %u ep_threshold_in_ms %u ",
             ep_threshold_in_bytes,
             ep_threshold_in_ms);

   TRY(result, wcntr_check_threshold_of_modules(me_ptr, ep_ptr->gu.module_instance_id, ep_threshold_in_ms));
   TRY(result, wcntr_recreate_all_buffers(me_ptr));

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_MED_PRIO, "wcntr_handle_port_data_thresh_change_event result 0x%lx ", result);

   // clear anyway as media fmt will call this func again. keeping it on triggers repeated calls from data_process.
   me_ptr->topo.capi_event_flag.port_thresh = FALSE;

   return result;
}

ar_result_t wcntr_check_threshold_of_modules(wcntr_t *me_ptr,
                                                        uint32_t    ep_module_id,
                                                        uint32_t    ep_threshold_in_ms)
{
   ar_result_t result = AR_EOK;
   if (0 == ep_threshold_in_ms)
   {

      WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "EP module 0x%lX has zero ms threshold", ep_module_id);
      return AR_EFAILED;
   }

   for (wcntr_gu_sg_list_t *sg_list_ptr = me_ptr->topo.gu.sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (wcntr_gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         wcntr_topo_module_t *module_ptr         = (wcntr_topo_module_t *)module_list_ptr->module_ptr;
         bool_t               is_media_fmt_valid = FALSE;
         for (wcntr_gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr;
              (NULL != out_port_list_ptr);
              LIST_ADVANCE(out_port_list_ptr))
         {
            wcntr_topo_output_port_t *out_port_ptr = (wcntr_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
            is_media_fmt_valid                     = out_port_ptr->common.flags.is_mf_valid;

            if (is_media_fmt_valid && out_port_ptr->common.flags.port_has_threshold)
            {
               uint32_t module_thresh_in_ms = wcntr_topo_get_curr_port_threshold(&out_port_ptr->common) /
                                              wcntr_topo_get_one_ms_buffer_size_in_bytes(&out_port_ptr->common);
               if (!wcntr_is_module_threshold_ep_threshold_integral_multiples(ep_threshold_in_ms, module_thresh_in_ms))
               {
                  WCNTR_MSG(me_ptr->topo.gu.log_id,
                            DBG_ERROR_PRIO,
                            "O/p Module 0x%lX -threshold %lu ms and end point 0x%lX threshold %lu ms are not "
                            "integral multiples",
                            module_ptr->gu.module_instance_id,
                            module_thresh_in_ms,
                            ep_module_id,
                            ep_threshold_in_ms);
                  return AR_EFAILED;
               }
            }
         }

         for (wcntr_gu_input_port_list_t *in_port_list_ptr = module_ptr->gu.input_port_list_ptr;
              (NULL != in_port_list_ptr);
              LIST_ADVANCE(in_port_list_ptr))
         {
            wcntr_topo_input_port_t *in_port_ptr = (wcntr_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;
            is_media_fmt_valid                   = in_port_ptr->common.flags.is_mf_valid;
            if (is_media_fmt_valid && in_port_ptr->common.flags.port_has_threshold)
            {
               uint32_t module_thresh_in_ms = wcntr_topo_get_curr_port_threshold(&in_port_ptr->common) /
                                              wcntr_topo_get_one_ms_buffer_size_in_bytes(&in_port_ptr->common);

               if (!wcntr_is_module_threshold_ep_threshold_integral_multiples(ep_threshold_in_ms, module_thresh_in_ms))
               {
                  WCNTR_MSG(me_ptr->topo.gu.log_id,
                            DBG_ERROR_PRIO,
                            "I/p Module 0x%lX -threshold %lu ms and end point 0x%lX threshold %lu ms are not "
                            "integral multiples. Bypassing the module",
                            module_ptr->gu.module_instance_id,
                            module_thresh_in_ms,
                            ep_module_id,
                            ep_threshold_in_ms);
                  return AR_EFAILED;
               }
            }
         }
      }
   }

   return result;
}

ar_result_t wcntr_recreate_all_buffers(wcntr_t *me_ptr)
{

   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

   uint32_t op_frame_in_ms = me_ptr->op_frame_in_ms;
   // Threshold cannot be zero
   if (op_frame_in_ms == 0)
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_HIGH_PRIO, "op_frame_in_ms cannot be zero");
      return AR_EFAILED;
   }

   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_HIGH_PRIO,
             "wcntr_recreate_all_buffers START with op_frame_in_ms %u",
             op_frame_in_ms);

   /**
    *
    * Buffers are created during threshold propagation only at output ports
    * Same output buffer information is made available to connected input port here
    * Size of the buffer is decided by op_frame_in_ms
    * TODO: Handle inplace later
    */

   for (wcntr_gu_sg_list_t *sg_list_ptr = me_ptr->topo.gu.sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (wcntr_gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         wcntr_topo_module_t *module_ptr = (wcntr_topo_module_t *)module_list_ptr->module_ptr;

         wcntr_topo_input_port_t *cur_mod_in_port_ptr = NULL;
         if (module_ptr->flags.inplace)
         {
            if (module_ptr->gu.input_port_list_ptr == NULL)
            {

               WCNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_ERROR_PRIO,
                         "Inplace module 0x%lX:  "
                         "doesn't have any input port created. Data link may be missing",
                         module_ptr->gu.module_instance_id);
               return AR_EFAILED;
            }

            cur_mod_in_port_ptr = (wcntr_topo_input_port_t *)module_ptr->gu.input_port_list_ptr->ip_port_ptr;
         }

         for (wcntr_gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr;
              (NULL != out_port_list_ptr);
              LIST_ADVANCE(out_port_list_ptr))
         {
            wcntr_topo_output_port_t *out_port_ptr = (wcntr_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
            wcntr_topo_input_port_t * next_mod_in_port_ptr =
               (wcntr_topo_input_port_t *)out_port_ptr->gu.conn_in_port_ptr;

            int8_t *data_ptr = NULL;

            if (!out_port_ptr->common.flags.is_mf_valid)
            {

               WCNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_ERROR_PRIO,
                         "Module 0x%lX: out port id 0x%lx, MF is not valid continue ",
                         out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                         out_port_ptr->gu.cmn.id);

               continue;
            }

            uint32_t req_output_buf_size_bytes =
               wcntr_topo_get_one_ms_buffer_size_in_bytes(&out_port_ptr->common) * op_frame_in_ms;

            bool_t output_size_changed = (out_port_ptr->common.max_buf_len != req_output_buf_size_bytes) ? TRUE : FALSE;

            if (out_port_ptr->common.bufs_ptr && out_port_ptr->common.bufs_ptr[0].actual_data_len != 0)
            {
               WCNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_HIGH_PRIO,
                         "Module 0x%lX: out port id 0x%lx, Warning: resetting/dropping buffer when it has "
                         "valid data %lu",
                         out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                         out_port_ptr->gu.cmn.id,
                         out_port_ptr->common.bufs_ptr[0].actual_data_len);
               // if one buf is zero, rest are also assumed to be zero.
               out_port_ptr->common.bufs_ptr[0].actual_data_len = 0;
            }

            out_port_ptr->common.max_buf_len = req_output_buf_size_bytes;
         
            if(out_port_ptr->common.bufs_ptr)
            {
                data_ptr = out_port_ptr->common.bufs_ptr[0].data_ptr;
            }
         
            if (!module_ptr->flags.inplace)
            {
               if (output_size_changed)
               {
                  MFREE_NULLIFY(data_ptr);
                  MALLOC_MEMSET(data_ptr, int8_t, out_port_ptr->common.max_buf_len, me_ptr->topo.heap_id, result);
               }
            }
            else
            {
               wcntr_topo_common_port_t *cmn_current_in_port_ptr = &cur_mod_in_port_ptr->common;
               data_ptr                                          = cmn_current_in_port_ptr->bufs_ptr[0].data_ptr;
            }

            if (output_size_changed)
            {
               TRY(result,
                   wcntr_topo_initialize_bufs_sdata(&me_ptr->topo,
                                                    &out_port_ptr->common,
                                                    out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                                                    out_port_ptr->gu.cmn.id,
                                                    TRUE,
                                                    data_ptr));
            }

            // Media format is blocked during open command using can_mf_be_propagated in subgraph structure
            // can_mf_be_propagated is updated for all other subgraph commands except open
            // Say we have two subgraphs S1-S2 opened and only S1 is in prepare state. Last module output of
            // S1 creates output buffers with correct size. When S2 also receives, first module input port will not
            // have buffers updated. Keeping the following outsize of size mismatch check will help in updating buffers
            // correctly
            // after media format is propagated
            if (next_mod_in_port_ptr)
            {
               wcntr_topo_common_port_t *cmn_in_port_ptr = &next_mod_in_port_ptr->common;
               cmn_in_port_ptr->max_buf_len              = req_output_buf_size_bytes;
               TRY(result,
                   wcntr_topo_initialize_bufs_sdata(&me_ptr->topo,
                                                    &next_mod_in_port_ptr->common,
                                                    next_mod_in_port_ptr->gu.cmn.module_ptr->module_instance_id,
                                                    next_mod_in_port_ptr->gu.cmn.id,
                                                    TRUE,
                                                    data_ptr));
            }

            out_port_ptr->common.port_event_new_threshold = 0;
         }
      }
   }

   wcntr_print_buffers_of_modules(me_ptr);

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   if (AR_DID_FAIL(result))
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "failure creating buffers");
   }

   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_HIGH_PRIO, "wcntr_recreate_all_buffers DONE");

   return result;
}
