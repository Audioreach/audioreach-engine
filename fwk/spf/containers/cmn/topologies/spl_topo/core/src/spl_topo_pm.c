/**
 * \file spl_topo_pm.c
 *
 * \brief
 *
 *     Topo 2 power management utilities.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spl_topo_i.h"

/* =======================================================================
Public Function Definitions
========================================================================== */
ar_result_t spl_topo_aggregate_kpps_bandwidth(spl_topo_t *topo_ptr,
                                              bool_t      force_aggregate,
                                              uint32_t   *aggregate_kpps_ptr,
                                              uint32_t   *aggregate_bw_ptr,
                                              uint32_t   *scaled_kpps_agg_q4_ptr,
                                              uint32_t   *scaled_bw_agg_ptr)
{

   return gen_topo_aggregate_kpps_bandwidth(&topo_ptr->t_base,
                                            force_aggregate,
                                            aggregate_kpps_ptr,
                                            aggregate_bw_ptr,
                                            scaled_kpps_agg_q4_ptr,
                                            scaled_bw_agg_ptr);
}
