/**
 * \file gen_topo_public_functions_md_island.c
 * \brief
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "gen_topo_capi.h"
#include "gen_topo_buf_mgr.h"

/**
 * doesn't call algo reset & destroy MD
 */
ar_result_t topo_basic_reset_output_port(gen_topo_t *me_ptr, gen_topo_output_port_t *out_port_ptr, bool_t use_bufmgr)
{
   ar_result_t result = AR_EOK;

   out_port_ptr->common.sdata.flags.word                = 0;
   out_port_ptr->common.sdata.flags.stream_data_version = CAPI_STREAM_V2;
   out_port_ptr->common.sdata.timestamp                 = 0;
   out_port_ptr->common.fract_timestamp_pns             = 0;

   gen_topo_handle_data_flow_end(me_ptr, &out_port_ptr->common, &out_port_ptr->gu.cmn);

   if (out_port_ptr->common.bufs_ptr)
   {
      gen_topo_set_all_bufs_len_to_zero(&out_port_ptr->common);
      if (use_bufmgr)
      {
         // return to buf mgr only if not shared with output
         out_port_ptr->common.flags.force_return_buf = TRUE;
         gen_topo_check_return_one_buf_mgr_buf(me_ptr,
                                               &out_port_ptr->common,
                                               out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                                               out_port_ptr->gu.cmn.id);
      }
   }

#ifdef VERBOSE_DEBUGGING
   gen_topo_module_t *module_ptr = (gen_topo_module_t *)out_port_ptr->gu.cmn.module_ptr;
   TOPO_MSG(me_ptr->gu.log_id,
            DBG_LOW_PRIO,
            "Module 0x%lX: reset output port 0x%lx",
            module_ptr->gu.module_instance_id,
            out_port_ptr->gu.cmn.id);
#endif

   //update data-flow state on the attached module ports
   if (out_port_ptr->gu.attached_module_ptr)
   {
      gen_topo_input_port_t *attached_module_ip_port_ptr =
         (gen_topo_input_port_t *)out_port_ptr->gu.attached_module_ptr->input_port_list_ptr->ip_port_ptr;
      gen_topo_output_port_t *attached_module_out_port_ptr =
         (gen_topo_output_port_t *)out_port_ptr->gu.attached_module_ptr->output_port_list_ptr->op_port_ptr;
      gen_topo_handle_data_flow_end(me_ptr, &attached_module_ip_port_ptr->common, &attached_module_ip_port_ptr->gu.cmn);

      gen_topo_handle_data_flow_end(me_ptr,
                                    &attached_module_out_port_ptr->common,
                                    &attached_module_out_port_ptr->gu.cmn);
   }

   return result;
}

/**
 * this reset func was introduced to avoid calling algo reset
 *  & destroy MD
 */
ar_result_t topo_basic_reset_input_port(gen_topo_t *me_ptr, gen_topo_input_port_t *in_port_ptr, bool_t use_bufmgr)
{
   in_port_ptr->flags.processing_began      = FALSE;
   in_port_ptr->flags.disable_ts_disc_check = FALSE;
   in_port_ptr->flags.need_more_input       = TRUE;
   in_port_ptr->flags.was_eof_set           = FALSE;

   in_port_ptr->common.sdata.flags.word                = 0;
   in_port_ptr->common.sdata.flags.stream_data_version = CAPI_STREAM_V2;
   in_port_ptr->common.sdata.timestamp                 = 0;
   in_port_ptr->common.fract_timestamp_pns             = 0;
   in_port_ptr->bytes_from_prev_buf                    = 0;

   gen_topo_handle_data_flow_end(me_ptr, &in_port_ptr->common, &in_port_ptr->gu.cmn);

   memset(&in_port_ptr->ts_to_sync, 0, sizeof(in_port_ptr->ts_to_sync));

   if (in_port_ptr->common.bufs_ptr)
   {
      gen_topo_set_all_bufs_len_to_zero(&in_port_ptr->common);
      if (use_bufmgr)
      {
         // return to buf mgr only if not shared with output
         in_port_ptr->common.flags.force_return_buf = TRUE;
         gen_topo_check_return_one_buf_mgr_buf(me_ptr,
                                               &in_port_ptr->common,
                                               in_port_ptr->gu.cmn.module_ptr->module_instance_id,
                                               in_port_ptr->gu.cmn.id);
      }
   }

#ifdef VERBOSE_DEBUGGING
   TOPO_MSG(me_ptr->gu.log_id,
            DBG_LOW_PRIO,
            "Module 0x%lX: reset input port 0x%lx",
            in_port_ptr->gu.cmn.module_ptr->module_instance_id,
            in_port_ptr->gu.cmn.id);
#endif

   return AR_EOK;
}
