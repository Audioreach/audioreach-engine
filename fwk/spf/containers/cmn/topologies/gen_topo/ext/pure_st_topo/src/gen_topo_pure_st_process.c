/**
 * \file gen_topo_data_process_island.c
 * \brief
 *     This file contains functions for topo common data processing
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"

void st_topo_drop_stale_data(gen_topo_t *topo_ptr, gen_topo_output_port_t *out_port_ptr)
{
   out_port_ptr->err_msg_stale_data_drop_counter++;
   if (topo_ptr->topo_to_cntr_vtable_ptr->check_for_error_print)
   {
      if (topo_ptr->topo_to_cntr_vtable_ptr->check_for_error_print(topo_ptr))
      {
         TOPO_MSG_ISLAND(topo_ptr->gu.log_id,
                         DBG_HIGH_PRIO,
                         " Module 0x%lX: out port id 0x%lx, Dropping stale data %lu bytes for signal trigger. No of "
                         "stale data drop in this port after last print :%u",
                         out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                         out_port_ptr->gu.cmn.id,
                         out_port_ptr->common.bufs_ptr[0].actual_data_len,
                         out_port_ptr->err_msg_stale_data_drop_counter);
         out_port_ptr->err_msg_stale_data_drop_counter = 0;
      }
   }

   if (out_port_ptr->common.sdata.metadata_list_ptr)
   {
      // Dont need to call island exit here even when md lib is compiled in nlpi, because we would have exited
      // island while copying md at container input or propagating md
      gen_topo_drop_all_metadata_within_range(topo_ptr->gu.log_id,
                                              (gen_topo_module_t *)out_port_ptr->gu.cmn.module_ptr,
                                              &out_port_ptr->common,
                                              gen_topo_get_actual_len_for_md_prop(&out_port_ptr->common),
                                              FALSE /*keep_eos_and_ba_md*/);
   }
   gen_topo_set_all_bufs_len_to_zero(&out_port_ptr->common);
}
