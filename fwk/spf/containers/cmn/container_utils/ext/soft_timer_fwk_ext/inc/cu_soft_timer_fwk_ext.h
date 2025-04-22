#ifndef CU_SOFT_TIMER_FWK_H
#define CU_SOFT_TIMER_FWK_H
/**
 * // clang-format off
 * \file cu_soft_timer_fwk_ext.h
 * \brief
 *  This file contains utility functions for FWK_EXTN_SOFT_TIMER
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
// clang-format on

#include "posal.h"
#include "topo_utils.h"
#include "spf_list_utils.h"
#include "capi_fwk_extns_soft_timer.h"

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

typedef struct cu_base_t cu_base_t;

ar_result_t cu_fwk_extn_soft_timer_start(cu_base_t *  base_ptr,
                                         gu_module_t *module_ptr,
                                         uint32_t     timer_id,
                                         int64_t      duration_us);

ar_result_t cu_fwk_extn_soft_timer_disable(cu_base_t *base_ptr, gu_module_t *module_ptr, uint32_t timer_id);

ar_result_t cu_fwk_extn_soft_timer_expired(cu_base_t *base_ptr, uint32_t ch_bit_index);

void cu_fwk_extn_soft_timer_destroy_at_close(cu_base_t *base_ptr, gu_module_t *module_ptr);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif /* CU_SOFT_TIMER_FWK_H */
