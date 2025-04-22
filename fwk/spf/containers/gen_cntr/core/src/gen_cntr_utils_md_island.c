/**
 * \file gen_cntr_utils_md_island.c
 * \brief
 *     This file contains utility functions for GEN_CNTR
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"

ar_result_t gen_cntr_ext_out_port_basic_reset(gen_cntr_t *me_ptr, gen_cntr_ext_out_port_t *ext_out_port_ptr)
{
   ar_result_t result = AR_EOK;

   gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)ext_out_port_ptr->gu.int_out_port_ptr;

   topo_basic_reset_output_port(&me_ptr->topo, out_port_ptr, TRUE);

   gen_cntr_ext_out_port_int_reset(me_ptr, ext_out_port_ptr);

   return result;
}

ar_result_t gen_cntr_ext_out_port_int_reset(gen_cntr_t *me_ptr, gen_cntr_ext_out_port_t *ext_out_port_ptr)
{
   ar_result_t result = AR_EOK;

   gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)ext_out_port_ptr->gu.int_out_port_ptr;

   ext_out_port_ptr->cu.icb_info.is_prebuffer_sent = FALSE;

   if (ext_out_port_ptr->cu.out_bufmgr_node.buf_ptr)
   {
      gen_cntr_return_back_out_buf(me_ptr, ext_out_port_ptr);
   }

   ext_out_port_ptr->buf.data_ptr        = NULL;
   ext_out_port_ptr->buf.actual_data_len = 0;
   ext_out_port_ptr->buf.max_data_len    = 0;

   memset(&ext_out_port_ptr->next_out_buf_ts, 0, sizeof(ext_out_port_ptr->next_out_buf_ts));

   GEN_CNTR_MSG_ISLAND( me_ptr->topo.gu.log_id,
                        DBG_LOW_PRIO,
                        "External output reset Module 0x%lX, Port 0x%lx",
                        out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                        out_port_ptr->gu.cmn.id);

   return result;
}



