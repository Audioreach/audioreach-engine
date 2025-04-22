/**
@file irm_prev_metric_info.h

@brief Wrapper interface between IRM-Sysmon or IRM-SIM.

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#ifndef _IRM_PREV_METRIC_INFO_H_
#define _IRM_PREV_METRIC_INFO_H_

#include "posal.h"

typedef struct irm_prev_metric_processor_cycles_t
{
   uint64_t processor_cycles;
} irm_prev_metric_processor_cycles_t;

typedef struct irm_prev_metric_packet_count_t
{
   uint64_t packet_count;
} irm_prev_metric_packet_count_t;

typedef struct irm_prev_metric_mem_transactions_t
{
   uint32_t axi_rd_cnt;
   uint32_t axi_wr_cnt;
} irm_prev_metric_mem_transactions_t;

#endif /* _IRM_PREV_METRIC_INFO_H_ */
