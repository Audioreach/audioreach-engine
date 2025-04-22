/**
 * \file gen_cntr_metadata.c
 * \brief
 *     This file contains functions that handle metadata.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"
#include "gen_cntr_utils.h"

ar_result_t gen_cntr_create_send_eos_md(gen_cntr_t *me_ptr, gen_cntr_ext_out_port_t *ext_out_port_ptr)
{
   ar_result_t            result = AR_EOK;
   spf_msg_t              msg;
   spf_msg_header_t *     header_ptr  = NULL;
   spf_msg_data_buffer_t *out_buf_ptr = NULL;

   if (ext_out_port_ptr->gu.downstream_handle.spf_handle_ptr)
   {
      uint32_t total_size = GET_SPF_INLINE_DATABUF_REQ_SIZE(0);
      if (AR_DID_FAIL(spf_msg_create_msg(&msg,
                                         &total_size,
                                         SPF_MSG_DATA_BUFFER,
                                         NULL,
                                         0,
                                         ext_out_port_ptr->gu.downstream_handle.spf_handle_ptr,
                                         me_ptr->cu.heap_id)))
      {
         GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_ERROR_PRIO,
                      "Failed to create a buffer for sending EOS metadata, Module 0x%lx, 0x%lx",
                      ext_out_port_ptr->gu.int_out_port_ptr->cmn.module_ptr->module_instance_id,
                      ext_out_port_ptr->gu.int_out_port_ptr->cmn.id);
         return result;
      }

      header_ptr                     = (spf_msg_header_t *)(msg.payload_ptr);
      out_buf_ptr                    = (spf_msg_data_buffer_t *)&header_ptr->payload_start;
      out_buf_ptr->max_size          = 0;
      out_buf_ptr->actual_size       = 0;
      out_buf_ptr->metadata_list_ptr = ext_out_port_ptr->md_list_ptr;
      out_buf_ptr->timestamp         = 0;
      out_buf_ptr->flags             = 0;
      ext_out_port_ptr->md_list_ptr  = NULL;
      memset(&out_buf_ptr->data_buf[0], 0, out_buf_ptr->max_size);

      result = posal_queue_push_back(ext_out_port_ptr->gu.downstream_handle.spf_handle_ptr->q_ptr,
                                     (posal_queue_element_t *)&msg);
      if (AR_DID_FAIL(result))
      {
         GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_ERROR_PRIO,
                      "Failed to send buffer for sending EOS metadata, Module 0x%lx, 0x%lx",
                      ext_out_port_ptr->gu.int_out_port_ptr->cmn.module_ptr->module_instance_id,
                      ext_out_port_ptr->gu.int_out_port_ptr->cmn.id);
         spf_msg_return_msg(&msg);
      }
      else
      {
         GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_LOW_PRIO,
                      "Sent EOS metadata from Module 0x%lx, 0x%lx",
                      ext_out_port_ptr->gu.int_out_port_ptr->cmn.module_ptr->module_instance_id,
                      ext_out_port_ptr->gu.int_out_port_ptr->cmn.id);
      }
   }

   return result;
}
