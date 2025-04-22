#ifndef _CU_DUTY_CYCLE_H_
#define _CU_DUTY_CYCLE_H_

/**
 * \file cu_duty_cycle.h
 *
 * \brief
 *     This file defines Duty Cycle to container functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal.h"
#include "topo_utils.h"
#include "spf_list_utils.h"
#include "duty_cycle_cntr_if.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus


ar_result_t cu_register_with_dcm(cu_base_t *me_ptr);
ar_result_t cu_send_island_entry_ack_to_dcm(cu_base_t *me_ptr);
ar_result_t cu_dcm_island_entry_exit_handler(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr, uint32_t   pid);
ar_result_t cu_deregister_with_dcm(cu_base_t *me_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _DUTY_CYCLE_CNTR_IF_H_
