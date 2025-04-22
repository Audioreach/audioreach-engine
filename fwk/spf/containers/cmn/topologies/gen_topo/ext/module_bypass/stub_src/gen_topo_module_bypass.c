/**
 * \file gen_topo_module_bypass.c
 * \brief
 *     This file contains stub utility for handling disabled modules
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"

ar_result_t gen_topo_check_create_bypass_module(gen_topo_t *topo_ptr, gen_topo_module_t *module_ptr)
{
   return AR_EUNSUPPORTED;
}

ar_result_t gen_topo_check_destroy_bypass_module(gen_topo_t        *topo_ptr,
                                                 gen_topo_module_t *module_ptr,
                                                 bool_t             is_module_destroying)
{
   return AR_EUNSUPPORTED;
}
