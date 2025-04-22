/**
 * \file gen_cntr_err_handler.c
 * \brief
 *     This file contains functions that do error handeling.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"
#include "gen_cntr_utils.h"

static ar_result_t gen_cntr_err_handler_for_signal_miss(gen_cntr_t *me_ptr)
{
   return AR_EOK;
}

ar_result_t gen_cntr_check_handle_signal_miss(gen_cntr_t *me_ptr, bool_t is_after_process, bool_t *continue_processing)
{
   return AR_EOK;
}

bool_t gen_cntr_check_for_err_print(gen_topo_t *topo_ptr)
{
   return FALSE;
}

