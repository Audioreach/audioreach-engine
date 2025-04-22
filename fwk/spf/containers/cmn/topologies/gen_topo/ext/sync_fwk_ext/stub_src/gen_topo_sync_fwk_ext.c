/**
 * \file gen_topo_sync_fwk_ext.c
 *
 * \brief
 *
 *     Implementation of stub utilities for FWK_EXTN_SYNC in gen cntr
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#include "gen_topo_sync_fwk_ext.h"

ar_result_t gen_topo_fwk_extn_sync_will_start(gen_topo_t *me_ptr, gen_topo_module_t *module_ptr)
{
   return AR_EOK;
}

void gen_topo_fwk_extn_sync_propagate_threshold_state(gen_topo_t *me_ptr)
{
  return;
}

bool_t gen_topo_fwk_extn_does_sync_module_exist_downstream_util_(gen_topo_t *me_ptr, gen_topo_input_port_t *in_port_ptr)
{
  return FALSE;
}
