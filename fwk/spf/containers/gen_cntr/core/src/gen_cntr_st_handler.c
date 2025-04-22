/**
 * \file gen_cntr_st_handler.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"

/* Fills buffer with null bursts for IEC60958_NON_LINEAR and IEC61937 data formats */
void gen_cntr_st_fill_null_bursts(uint32_t num_zeros_to_fill, int8_t *dst_ptr)
{
   int16_t *ptr = (int16_t *)dst_ptr;
   int16_t  pa = 0xF872, pb = 0x4E1F, pc = 0xE000, pd = 0x0000;
   // writing 8 bytes at a time, so divide by 8;
   for (int i = 0; i < (num_zeros_to_fill >> 3); i++)
   {
      *(ptr++) = pa;
      *(ptr++) = pb;
      *(ptr++) = pc;
      *(ptr++) = pd;
   }
}

ar_result_t gen_cntr_reset_downstream_of_stm_upon_stop_and_send_eos(gen_cntr_t        *me_ptr,
                                                                    gen_topo_module_t *stm_module_ptr,
                                                                    gen_topo_module_t *curr_module_ptr,
                                                                    uint32_t          *recurse_depth_ptr)
{
   ar_result_t result = AR_EOK;

   RECURSION_ERROR_CHECK_ON_FN_ENTRY(me_ptr->topo.gu.log_id, recurse_depth_ptr, 50); // 50 is recursion depth

   gen_topo_t     *topo_ptr = &me_ptr->topo;
   topo_sg_state_t sg_state = ((gen_topo_sg_t *)curr_module_ptr->gu.sg_ptr)->state;

   if ((TOPO_SG_STATE_STOPPED != sg_state))
   {
      gen_topo_reset_all_in_ports(curr_module_ptr);
      gen_topo_reset_all_out_ports(curr_module_ptr);
      gen_topo_reset_module(topo_ptr, curr_module_ptr);
   }

   for (gu_output_port_list_t *out_port_list_ptr = curr_module_ptr->gu.output_port_list_ptr;
        (NULL != out_port_list_ptr);
        LIST_ADVANCE(out_port_list_ptr))
   {
      gen_topo_output_port_t  *topo_out_port_ptr = (gen_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
      gen_cntr_ext_out_port_t *ext_out_port_ptr  = (gen_cntr_ext_out_port_t *)topo_out_port_ptr->gu.ext_out_port_ptr;

      /** need to send EOS only for ext port which is connected and which is not in stop or invalid states.
       *  Because if last module is stopped, neighbouring container also knows about it through APM.*/
      if (ext_out_port_ptr && ext_out_port_ptr->gu.downstream_handle.spf_handle_ptr &&
          (TOPO_SG_STATE_STOPPED != sg_state) && (TOPO_SG_STATE_INVALID != sg_state))
      {
         // send EOS to external clients if ext port is in different SG
         uint32_t                  INPUT_PORT_ID_NONE = 0; // NULL input port -> don't care input port id.
         module_cmn_md_eos_flags_t eos_md_flag        = { .word = 0 };
         eos_md_flag.is_flushing_eos                  = TRUE;
         eos_md_flag.is_internal_eos                  = TRUE;

         result = gen_topo_create_eos_for_cntr(&me_ptr->topo,
                                               NULL, /* input_port_ptr*/
                                               INPUT_PORT_ID_NONE,
                                               me_ptr->cu.heap_id,
                                               &ext_out_port_ptr->md_list_ptr,
                                               NULL,         /* md_flag_ptr */
                                               NULL,         /*tracking_payload_ptr*/
                                               &eos_md_flag, /* eos_payload_flags */
                                               gen_cntr_get_bytes_in_ext_out_for_md(ext_out_port_ptr),
                                               &ext_out_port_ptr->cu.media_fmt);

         GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_LOW_PRIO,
                      "MD_DBG: Created EOS for ext out port (0x%0lX, 0x%lx) with result 0x%lx",
                      topo_out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                      topo_out_port_ptr->gu.cmn.id,
                      result);

         // if there's a buffer already popped use it
         if (ext_out_port_ptr->cu.out_bufmgr_node.buf_ptr)
         {
            gen_cntr_write_data_for_peer_cntr(me_ptr, ext_out_port_ptr);
         }
         else
         {
            // unlike regular data path messages this message will use buf mgr buffer not bufQ buf
            if (AR_DID_FAIL(gen_cntr_create_send_eos_md(me_ptr, ext_out_port_ptr)))
            {
               gen_topo_destroy_all_metadata(me_ptr->topo.gu.log_id,
                                             (void *)ext_out_port_ptr->gu.int_out_port_ptr->cmn.module_ptr,
                                             &ext_out_port_ptr->md_list_ptr,
                                             TRUE /*is_dropped*/);
            }
            gen_cntr_ext_out_port_basic_reset(me_ptr, ext_out_port_ptr);
         }
      }
      else
      {
         gen_topo_input_port_t *next_in_port_ptr = (gen_topo_input_port_t *)topo_out_port_ptr->gu.conn_in_port_ptr;
         if (next_in_port_ptr)
         {
            gen_topo_module_t *next_module_ptr = (gen_topo_module_t *)next_in_port_ptr->gu.cmn.module_ptr;

            gen_cntr_reset_downstream_of_stm_upon_stop_and_send_eos(me_ptr,
                                                                    stm_module_ptr,
                                                                    next_module_ptr,
                                                                    recurse_depth_ptr);
         }
      }
   }

   RECURSION_ERROR_CHECK_ON_FN_EXIT(me_ptr->topo.gu.log_id, recurse_depth_ptr);

   return result;
}

gen_topo_module_t *gen_cntr_get_stm_module(gen_cntr_t *me_ptr)
{
   gen_topo_module_t *stm_ptr         = NULL;
   uint32_t           num_stm_modules = 0;
   for (gu_sg_list_t *sg_list_ptr = me_ptr->topo.gu.sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         gen_topo_module_t *module_ptr = (gen_topo_module_t *)module_list_ptr->module_ptr;

         if (module_ptr->flags.need_stm_extn)
         {
            stm_ptr = module_ptr;
            num_stm_modules++;
         }
      }
   }

   // Return error if there are more than one stm module in the topology.
   if (num_stm_modules > 1)
   {
      GEN_CNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "%lu > 1 STM modules found.", num_stm_modules);
      return NULL;
   }

   return stm_ptr;
}

void gen_cntr_st_check_print_overrun(gen_cntr_t *me_ptr, gen_cntr_ext_out_port_t *ext_out_port_ptr)
{
   ext_out_port_ptr->err_msg_overrun_prepare_buf_err_count++;
   if (gen_cntr_check_for_err_print(&me_ptr->topo))
   {
      GEN_CNTR_MSG_ISLAND(me_ptr->topo.gu.log_id,
                          DBG_ERROR_PRIO,
                          "Ext output port 0x%lx of Module 0x%lX, No of overrun since last print : %u",
                          ext_out_port_ptr->gu.int_out_port_ptr->cmn.id,
                          ext_out_port_ptr->gu.int_out_port_ptr->cmn.module_ptr->module_instance_id,
                          ext_out_port_ptr->err_msg_overrun_prepare_buf_err_count);

      ext_out_port_ptr->err_msg_overrun_prepare_buf_err_count = 0;
   }
}

void gen_cntr_st_underrun(gen_cntr_t *me_ptr, gen_cntr_ext_in_port_t *ext_in_port_ptr, uint32_t bytes_required_per_buf)
{
   // If NBLC end of ext in port is a trigger policy module which needs triggers in STM, then this might not be true
   // underrun. Triger policy module might have data buffered. Don't print the error here - rely on Signal trigger
   // module to print.  for true underrun cases.
   gen_topo_input_port_t *in_port_ptr         = (gen_topo_input_port_t *)ext_in_port_ptr->gu.int_in_port_ptr;
   gen_topo_input_port_t *nblc_end_ptr        = (in_port_ptr->nblc_end_ptr ? in_port_ptr->nblc_end_ptr : in_port_ptr);
   gen_topo_module_t     *nblc_end_module_ptr = (gen_topo_module_t *)nblc_end_ptr->gu.cmn.module_ptr;
   topo_buf_t            *module_bufs_ptr     = in_port_ptr->common.bufs_ptr;
   bool_t                 data_tp_active      = gen_topo_is_module_data_trigger_policy_active(nblc_end_module_ptr);

   if (!data_tp_active)
   {
      ext_in_port_ptr->err_msg_underrun_err_count++;
      if (gen_cntr_check_for_err_print(&me_ptr->topo))
      {
         GEN_CNTR_MSG_ISLAND(me_ptr->topo.gu.log_id,
                             DBG_ERROR_PRIO,
                             "Ext input port 0x%lx of Module 0x%lX, actual data len %ld max data len %ld "
                             "(both per ch). No of underrun after last print %u.",
                             in_port_ptr->gu.cmn.id,
                             in_port_ptr->gu.cmn.module_ptr->module_instance_id,
                             in_port_ptr->common.bufs_ptr[0].actual_data_len,
                             in_port_ptr->common.bufs_ptr[0].max_data_len,
                             ext_in_port_ptr->err_msg_underrun_err_count);
         ext_in_port_ptr->err_msg_underrun_err_count = 0;
      }
   }

   if (in_port_ptr->flags.processing_began)
   {
      // If input buffer has fixed point and de-interleaved packed data, fill the rest of the bytes with zeros .
      if ((SPF_FIXED_POINT | SPF_COMPR_OVER_PCM_PACKETIZED | SPF_IEC60958_PACKETIZED | SPF_GENERIC_COMPRESSED |
           SPF_FLOATING_POINT) &
          ext_in_port_ptr->cu.media_fmt.data_format)
      {
         // Fill with erasure. Don't do erasure if data_tp is active and there is partial data - in that case, send
         // partial data only
         // to the data trigger module (it can decide to underrun based on its own logic since it may have buffered
         // data).
         if ((!data_tp_active) || (0 == in_port_ptr->common.bufs_ptr[0].actual_data_len))
         {
            // deinterleaved data can come from OLC or peer-cntr. in both cases, it's copied as unpacked.
            // memset rest of the channel data to zero.
            if (in_port_ptr->common.flags.is_pcm_unpacked)
            {
               for (uint32_t b = 0; b < in_port_ptr->common.sdata.bufs_num; b++)
               {
                  memset(in_port_ptr->common.bufs_ptr[b].data_ptr + in_port_ptr->common.bufs_ptr[0].actual_data_len,
                         0,
                         bytes_required_per_buf - in_port_ptr->common.bufs_ptr[0].actual_data_len);
               }
               /* (Max data len - len at nblc end - actual data len) should be memset and
                * actual data len should be updated as (Max data len - len at nblc end)  */
               in_port_ptr->common.bufs_ptr[0].actual_data_len =  bytes_required_per_buf;
            }
            else
            {
               for (uint32_t b = 0; b < in_port_ptr->common.sdata.bufs_num; b++)
               {
                  memset(in_port_ptr->common.bufs_ptr[b].data_ptr + in_port_ptr->common.bufs_ptr[b].actual_data_len,
                         0,
                         bytes_required_per_buf - in_port_ptr->common.bufs_ptr[b].actual_data_len);

                  /* (Max data len - len at nblc end - actual data len) should be memset and
                   * actual data len should be updated as (Max data len - len at nblc end)  */
                  in_port_ptr->common.bufs_ptr[b].actual_data_len = bytes_required_per_buf;
               }
            }

            // set erasure flag for underflow.
            in_port_ptr->common.sdata.flags.erasure = TRUE;
         }
#ifdef VERBOSE_DEBUGGING
         else
         {
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_ERROR_PRIO,
                         "Not filling erasure due to tp module and partial data.");
         }
#endif
      }
      else if ((SPF_IEC60958_PACKETIZED_NON_LINEAR | SPF_IEC61937_PACKETIZED) &
               ext_in_port_ptr->cu.media_fmt.data_format)
      {
         /* In compressed packetized case, fill with null bursts instead of zeros */
         uint32_t num_bytes_to_fill = (module_bufs_ptr[0].max_data_len - module_bufs_ptr[0].actual_data_len);
         /*get the internal buffer offset*/
         int8_t *dst_ptr = module_bufs_ptr[0].data_ptr + module_bufs_ptr[0].actual_data_len;

         gen_cntr_st_fill_null_bursts(num_bytes_to_fill, dst_ptr);

         // set erasure flag for underflow.
         in_port_ptr->common.sdata.flags.erasure = TRUE;

         /* Update actual length */
         module_bufs_ptr[0].actual_data_len = module_bufs_ptr[0].max_data_len;
      }
      else
      {
// nothing to done for the RAW or packetized data or invalid.
#ifdef VERBOSE_DEBUGGING
         GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_LOW_PRIO,
                      "gen_cntr_st_check_input_and_underrun: underrun for data format %lu ",
                      ext_in_port_ptr->cu.media_fmt.data_format);
#endif
      }
#ifdef VERBOSE_DEBUGGING
      GEN_CNTR_SIM_DEBUG(me_ptr->topo.gu.log_id,
                         "max_data_len %lu bytes, actual_data_len = %lu",
                         in_port_ptr->common.bufs_ptr[0].max_data_len,
                         in_port_ptr->common.bufs_ptr[0].actual_data_len);
#endif
   }
   else
   // first time underrun : we need to move data to the end of the buf and memset the beginning of the buffer.
   // also no need to set erasure
   {
      uint32_t num_zeros_to_fill_per_buf = 0;
      if ((SPF_FIXED_POINT | SPF_COMPR_OVER_PCM_PACKETIZED | SPF_IEC60958_PACKETIZED | SPF_GENERIC_COMPRESSED |
           SPF_FLOATING_POINT) &
          ext_in_port_ptr->cu.media_fmt.data_format)
      {
         num_zeros_to_fill_per_buf =
            in_port_ptr->common.bufs_ptr[0].max_data_len - in_port_ptr->common.bufs_ptr[0].actual_data_len;

         // deinterleaved data can come from OLC or peer-cntr. in both cases, it's copied as unpacked.
         // move the data to end of buf and put underrun in the beginning.
         if (in_port_ptr->common.flags.is_pcm_unpacked)
         {
            uint32_t actual_data_len = in_port_ptr->common.bufs_ptr[0].actual_data_len;
            uint32_t max_data_len    = in_port_ptr->common.bufs_ptr[0].max_data_len;
            for (uint32_t b = 0; b < in_port_ptr->common.sdata.bufs_num; b++)
            {
               memsmove(in_port_ptr->common.bufs_ptr[b].data_ptr + num_zeros_to_fill_per_buf,
                        max_data_len - num_zeros_to_fill_per_buf,
                        in_port_ptr->common.bufs_ptr[b].data_ptr,
                        actual_data_len);

               memset(in_port_ptr->common.bufs_ptr[b].data_ptr, 0, num_zeros_to_fill_per_buf);
            }
            in_port_ptr->common.bufs_ptr[0].actual_data_len = max_data_len;
         }
         else
         {
            for (uint32_t b = 0; b < in_port_ptr->common.sdata.bufs_num; b++)
            {
               memsmove(in_port_ptr->common.bufs_ptr[b].data_ptr + num_zeros_to_fill_per_buf,
                        in_port_ptr->common.bufs_ptr[b].max_data_len - num_zeros_to_fill_per_buf,
                        in_port_ptr->common.bufs_ptr[b].data_ptr,
                        in_port_ptr->common.bufs_ptr[b].actual_data_len);
               memset(in_port_ptr->common.bufs_ptr[b].data_ptr, 0, num_zeros_to_fill_per_buf);
               in_port_ptr->common.bufs_ptr[b].actual_data_len = in_port_ptr->common.bufs_ptr[b].max_data_len;
            }
         }

         // set erasure flag for underflow.
         in_port_ptr->common.sdata.flags.erasure = FALSE;
      }
      else if ((SPF_IEC60958_PACKETIZED_NON_LINEAR | SPF_IEC61937_PACKETIZED) &
               ext_in_port_ptr->cu.media_fmt.data_format)
      {
         /* In compressed packetized case, fill with null bursts instead of zeros */
         num_zeros_to_fill_per_buf = (module_bufs_ptr[0].max_data_len - module_bufs_ptr[0].actual_data_len);
         /*get the internal buffer offset*/
         int8_t *dst_ptr = module_bufs_ptr[0].data_ptr + module_bufs_ptr[0].actual_data_len;

         gen_cntr_st_fill_null_bursts(num_zeros_to_fill_per_buf, dst_ptr);

         // set erasure flag for underflow.
         in_port_ptr->common.sdata.flags.erasure = TRUE;

         in_port_ptr->common.bufs_ptr[0].actual_data_len = in_port_ptr->common.bufs_ptr[0].max_data_len;
      }
      else
      {
         // nothing to done for the RAW or packetized data or invalid.
         GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_ERROR_PRIO,
                      "gen_cntr_st_check_input_and_underrun: underrun for data format %lu not implemented ",
                      ext_in_port_ptr->cu.media_fmt.data_format);
      }

#ifdef VERBOSE_DEBUGGING
      GEN_CNTR_SIM_DEBUG(me_ptr->topo.gu.log_id,
                         "max_data_len %lu bytes, actual_data_len = %lu, num_zeros_to_fill_per_buf %lu",
                         in_port_ptr->common.bufs_ptr[0].max_data_len,
                         in_port_ptr->common.bufs_ptr[0].actual_data_len,
                         num_zeros_to_fill_per_buf);
#endif

      if (SPF_IS_PACKETIZED_OR_PCM(in_port_ptr->common.media_fmt_ptr->data_format) &&
          in_port_ptr->common.sdata.flags.is_timestamp_valid)
      {
         uint64_t time_adjust;
         if (in_port_ptr->common.flags.is_pcm_unpacked)
         {
            time_adjust = topo_bytes_per_ch_to_us(num_zeros_to_fill_per_buf, &ext_in_port_ptr->cu.media_fmt, NULL);
         }
         else
         {
            time_adjust = topo_bytes_to_us(num_zeros_to_fill_per_buf, &ext_in_port_ptr->cu.media_fmt, NULL);
         }

         in_port_ptr->common.sdata.timestamp -= time_adjust;
      }
   }
}
