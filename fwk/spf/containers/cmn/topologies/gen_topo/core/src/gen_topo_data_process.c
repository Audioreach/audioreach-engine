/**
 * \file st_topo_data_process_island.c
 * \brief
 *     This file contains functions for signal trigger topo common data processing
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "ar_msg.h"
/**
 * sufficients checks must be made prior to calling this function.
 */
void gen_topo_drop_data_after_mod_process(gen_topo_t *            topo_ptr,
                                          gen_topo_module_t *     module_ptr,
                                          gen_topo_output_port_t *out_port_ptr)
{
   gen_topo_input_port_t *conn_in_port_ptr = (gen_topo_input_port_t *)out_port_ptr->gu.conn_in_port_ptr;

   out_port_ptr->err_msg_dropped_data_counter++;
   if (topo_ptr->topo_to_cntr_vtable_ptr->check_for_error_print)
   {
      if (topo_ptr->topo_to_cntr_vtable_ptr->check_for_error_print(topo_ptr))
      {
         AR_MSG(DBG_ERROR_PRIO,
                         " Module 0x%lX: output port id 0x%lx, dropping %lu bytes connected in port state "
                         "0x%x because real time data reached a port which is not started. No of drops "
                         "after the last print %u for this port",
                         module_ptr->gu.module_instance_id,
                         out_port_ptr->gu.cmn.id,
                         out_port_ptr->common.bufs_ptr[0].actual_data_len,
                         conn_in_port_ptr->common.state,
                         out_port_ptr->err_msg_dropped_data_counter);
         out_port_ptr->err_msg_dropped_data_counter = 0;
      }
   }

   if (out_port_ptr->common.sdata.metadata_list_ptr)
   {
      // Dont need to call island exit here even when md lib is compiled in nlpi, because we would have
      // exited island while copying md in process context
      gen_topo_drop_all_metadata_within_range(topo_ptr->gu.log_id,
                                              module_ptr,
                                              &out_port_ptr->common,
                                              gen_topo_get_actual_len_for_md_prop(&out_port_ptr->common),
                                              FALSE /*keep_buffer_associated_md*/);
   }
   gen_topo_set_all_bufs_len_to_zero(&out_port_ptr->common);
}

void gen_topo_print_process_error(gen_topo_t *topo_ptr, gen_topo_module_t *module_ptr)
{
   module_ptr->err_msg_proc_failed_counter++;
   if (topo_ptr->topo_to_cntr_vtable_ptr->check_for_error_print)
   {
      if (topo_ptr->topo_to_cntr_vtable_ptr->check_for_error_print(topo_ptr))
      {
         AR_MSG(DBG_ERROR_PRIO,
                         "Process failed, No of proc failed after the last print : %u",
                         module_ptr->err_msg_proc_failed_counter);
         module_ptr->err_msg_proc_failed_counter = 0;
      }
   }
}
