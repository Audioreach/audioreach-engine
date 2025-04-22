/**
 * \file olc_timestamp.c
 * \brief
 *     This file contains timestamp utility functions for OLC
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "olc_i.h"

ar_result_t olc_copy_timestamp_from_input(olc_t *me_ptr, olc_ext_in_port_t *ext_in_port_ptr)
{
   bool_t                 new_ts_valid       = FALSE;
   bool_t                 continue_timestamp = FALSE;
   int64_t                new_ts             = 0;
   uint32_t               size               = 0;
   spf_msg_header_t *     header             = (spf_msg_header_t *)(ext_in_port_ptr->cu.input_data_q_msg.payload_ptr);
   spf_msg_data_buffer_t *buf_ptr            = (spf_msg_data_buffer_t *)&header->payload_start;

   new_ts_valid =
      cu_get_bits(buf_ptr->flags, DATA_BUFFER_FLAG_TIMESTAMP_VALID_MASK, DATA_BUFFER_FLAG_TIMESTAMP_VALID_SHIFT);

   continue_timestamp =
      cu_get_bits(buf_ptr->flags, DATA_BUFFER_FLAG_TIMESTAMP_CONTINUE_MASK, DATA_BUFFER_FLAG_TIMESTAMP_CONTINUE_SHIFT);

   new_ts                                = buf_ptr->timestamp;
   size                                  = buf_ptr->actual_size;
   ext_in_port_ptr->inbuf_ts.value       = new_ts;
   ext_in_port_ptr->inbuf_ts.valid       = new_ts_valid;
   ext_in_port_ptr->inbuf_ts.ts_continue = continue_timestamp;
   // ignore buffers without data because GK messages without data could be sent for sending metadata;
   // we should use those timestamps as they are probably just previously set value that are not cleared when MD is
   // sent.
   if (size)
   {
      gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)ext_in_port_ptr->gu.int_in_port_ptr;
      bool_t                 discont_ts  = gen_topo_check_copy_incoming_ts(&me_ptr->topo,
                                                          (gen_topo_module_t *)in_port_ptr->gu.cmn.module_ptr,
                                                          in_port_ptr,
                                                          new_ts,
                                                          new_ts_valid,
                                                          continue_timestamp);

      if (discont_ts)
      {
         // bool_t IS_MF_PENDING_FALSE = FALSE;
         // olc_check_set_input_discontinuity_flag(me_ptr, ext_in_port_ptr, IS_MF_PENDING_FALSE);
      }
#ifdef VERBOSE_DEBUGGING
      uint32_t flags = 0;
      flags          = buf_ptr->flags;
      OLC_MSG(me_ptr->topo.gu.log_id,
              DBG_LOW_PRIO,
              "Incoming timestamp 0x%lx%lx (%ld), valid %u, continue %u. size %lu, bytes_from_prev_buf %lu. "
              "flags0x%08lX",
              (uint32_t)new_ts,
              (uint32_t)(new_ts >> 32),
              (int32_t)new_ts,
              new_ts_valid,
              continue_timestamp,
              size,
              in_port_ptr->bytes_from_prev_buf,
              flags);
#endif
   }

   return AR_EOK;
}

ar_result_t olc_timestamp_set_next_out_buf_ts(olc_t *me_ptr, olc_ext_out_port_t *ext_out_port_ptr, void *ts)
{
   gen_topo_timestamp_t *out_port_ts = (gen_topo_timestamp_t *)ts;

   ext_out_port_ptr->out_buf_ts.valid = out_port_ts->valid;
   ext_out_port_ptr->out_buf_ts.value = out_port_ts->value;

   return AR_EOK;
}
