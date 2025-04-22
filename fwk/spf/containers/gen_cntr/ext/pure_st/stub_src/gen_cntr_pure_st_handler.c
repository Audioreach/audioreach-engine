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

/* Checks if pure signal triggered data process frames can be used for signal triggered containers.
   If there is any module with active trigger policy then it uses generic topology, else it uses
   pure signal triggered topology. */
ar_result_t gen_cntr_check_and_assign_st_data_process_fn(gen_cntr_t *me_ptr)
{
   return AR_EOK;
}
