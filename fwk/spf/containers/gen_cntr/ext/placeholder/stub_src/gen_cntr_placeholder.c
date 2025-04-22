/**
 * \file gen_cntr_placeholder.c
 * \brief
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"
#include "gen_cntr_utils.h"
#include "apm.h"
#include "cu_events.h"

const gen_cntr_fwk_module_vtable_t placeholder_vtable = {
   .set_cfg             = NULL,
   .reg_evt             = NULL,
   .raise_evt           = NULL,
   .raise_ts_disc_event = NULL,
};

ar_result_t gen_cntr_create_placeholder_module(gen_cntr_t *           me_ptr,
                                               gen_topo_module_t *    module_ptr,
                                               gen_topo_graph_init_t *graph_init_ptr)
{

   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   // do nothing else at this point. We will do the capi queries after getting the
   // real module ID
   TRY(result,
       __gpr_cmd_register(module_ptr->gu.module_instance_id,
                          graph_init_ptr->gpr_cb_fn,
                          graph_init_ptr->spf_handle_ptr));

   gen_cntr_module_t *gen_cntr_module_ptr        = (gen_cntr_module_t *)module_ptr;
   gen_cntr_module_ptr->fwk_module_ptr->vtbl_ptr = &placeholder_vtable;

   CATCH(result, GEN_CNTR_MSG_PREFIX, me_ptr->cu.gu_ptr->log_id)
   {
   }

   return result;
}

ar_result_t gen_cntr_placeholder_check_if_real_id_rcvd_at_prepare(cu_base_t *               base_ptr,
                                                                  spf_msg_cmd_graph_mgmt_t *cmd_gmgmt_ptr)
{

   return AR_EOK;
}
