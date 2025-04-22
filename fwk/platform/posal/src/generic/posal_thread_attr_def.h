/**
 * \file thread_prio_devcfg_target_i.h
 * \brief
 *  	 This file contains thread priority structure variables that are generic
 *     for all the platforms.
 *
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_THREAD_ATTR_DEF_H_
#define POSAL_THREAD_ATTR_DEF_H_

#include "posal_thread_prio.h"

enum spf_thread_prio_id_t;

void spf_thread_determine_dyn_attr(uint32_t duration_us, bool_t is_interrupt_trig, posal_thread_prio_t *thread_prio_ptr, uint32_t *sched_policy_ptr, uint32_t *affinity_mask_ptr);
void spf_thread_determine_static_attr(spf_thread_prio_id_t req_id, bool_t is_interrupt_trig, posal_thread_prio_t *thread_prio_ptr, uint32_t *sched_policy_ptr, uint32_t *affinity_mask_ptr);

#endif //POSAL_THREAD_ATTR_DEF_H_
