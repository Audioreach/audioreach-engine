/**
 * \file gen_topo_dm_ext_island.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "gen_topo_capi.h"

// check if a DM module is the path and check if extra buffer size is required depending up on the mode.
// The extra size is required to accomodate variable samples will be produced/consumed by the DM module in the
// nblc chain. The extra sizing is determined based on the following cases,
//  1. port is in variable output path if nblc start is DM with fixed in
//  2. port is in variable input path is nblc end module is in fixed output mode.
static uint32_t gen_topo_check_if_port_in_dm_modules_variable_path(gen_topo_t *            topo_ptr,
                                                                   gen_topo_common_port_t *port_ptr,
                                                                   gen_topo_module_t *     nblc_start_module_ptr,
                                                                   gen_topo_module_t *     nblc_end_module_ptr)
{
   if (nblc_start_module_ptr && gen_topo_is_dm_enabled(nblc_start_module_ptr) &&
       (GEN_TOPO_DM_FIXED_INPUT_MODE == nblc_start_module_ptr->flags.dm_mode))
   {
      return TRUE;
   }
   else if (nblc_end_module_ptr && gen_topo_is_dm_enabled(nblc_end_module_ptr) &&
            (GEN_TOPO_DM_FIXED_OUTPUT_MODE == nblc_end_module_ptr->flags.dm_mode))
   {
      return TRUE;
   }

   return FALSE;
}

bool_t gen_topo_is_out_port_in_dm_variable_nblc(gen_topo_t *topo_ptr, gen_topo_output_port_t *out_port_ptr)
{
   // If output ports nblc start ptr is non NULL then nblc start output's module itslef is the start module.
   gen_topo_module_t *nblc_start_module_ptr =
      (gen_topo_module_t *)(out_port_ptr->nblc_start_ptr ? out_port_ptr->nblc_start_ptr->gu.cmn.module_ptr : NULL);

   // If output ports end ptr is non NULL there are two cases,
   //  1. If end output is internal port, then connected peer module of the nble end output is the end module.
   //  1. If output port is external, then there is no end module downstream of the end output.
   gen_topo_module_t *nblc_end_module_ptr = NULL;
   if (out_port_ptr->nblc_end_ptr && out_port_ptr->nblc_end_ptr->gu.conn_in_port_ptr)
   {
      nblc_end_module_ptr = (gen_topo_module_t *)out_port_ptr->nblc_end_ptr->gu.conn_in_port_ptr->cmn.module_ptr;
   }

   return gen_topo_check_if_port_in_dm_modules_variable_path(topo_ptr,
                                                             &out_port_ptr->common,
                                                             nblc_start_module_ptr,
                                                             nblc_end_module_ptr);
}

bool_t gen_topo_is_in_port_in_dm_variable_nblc(gen_topo_t *topo_ptr, gen_topo_input_port_t *in_port_ptr)
{
   // If input's nblc start ptr is non NULL there are 2 cases,
   //  1. If nblc start is an internal input port, start module will be previous module[or connected peer].
   //  2. If nblc start is an external input, then there is no connected peer module upstream so start module ptr is
   //  NULL.
   gen_topo_module_t *nblc_start_module_ptr = NULL;
   if (in_port_ptr->nblc_start_ptr && in_port_ptr->nblc_start_ptr->gu.conn_out_port_ptr)
   {
      nblc_start_module_ptr = (gen_topo_module_t *)in_port_ptr->nblc_start_ptr->gu.conn_out_port_ptr->cmn.module_ptr;
   }

   // If nblc end input is non NULL, then its already connected to the nblc end module
   gen_topo_module_t *nblc_end_module_ptr =
      (gen_topo_module_t *)(in_port_ptr->nblc_end_ptr ? in_port_ptr->nblc_end_ptr->gu.cmn.module_ptr : NULL);

   return gen_topo_check_if_port_in_dm_modules_variable_path(topo_ptr,
                                                             &in_port_ptr->common,
                                                             nblc_start_module_ptr,
                                                             nblc_end_module_ptr);
}

// Check if the given input port is in the nblc path of a DM module's variable samples stream.
// and computes the additional bytes per ch required to accomodate variable buffer size requriement of the DM module
uint32_t gen_topo_compute_if_input_needs_addtional_bytes_for_dm(gen_topo_t *           topo_ptr,
                                                                gen_topo_input_port_t *in_port_ptr)
{
   // check if additional bytes are required for input to accomodate variable samples for DM usecase
   uint32_t additional_bytes_req_for_dm_per_ch = 0;
   if (gen_topo_is_in_port_in_dm_variable_nblc(topo_ptr, in_port_ptr))
   {
      additional_bytes_req_for_dm_per_ch =
            topo_us_to_bytes_per_ch(GEN_TOPO_DM_BUFFER_MAX_EXTRA_LEN_US, in_port_ptr->common.media_fmt_ptr);
#ifdef VERBOSE_DEBUGGING
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_HIGH_PRIO,
               "Module 0x%lX: in port id 0x%lx, additional_bytes_per_ch=%lu to handle DM modules variable "
               "input/output",
               in_port_ptr->gu.cmn.module_ptr->module_instance_id,
               in_port_ptr->gu.cmn.id,
               additional_bytes_req_for_dm_per_ch);
#endif
   }

   return additional_bytes_req_for_dm_per_ch;
}

