/**
 * \file cu_soft_timer_fwk_ext.c
 *
 * \brief
 *
 *     Implementation of soft_timer stub fwk extension functions.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_soft_timer_fwk_ext.h"


/* Enable timer and add node to the list*/
ar_result_t cu_fwk_extn_soft_timer_start(cu_base_t   *base_ptr,
                                         gu_module_t *module_ptr,
                                         uint32_t    timer_id,
                                         int64_t     duration_us)
{
   return AR_EOK;
}

/* Disable timer and remove corresponding node from the list*/
ar_result_t cu_fwk_extn_soft_timer_disable(cu_base_t *base_ptr, gu_module_t *module_ptr, uint32_t timer_id)
{
   return AR_EOK;
}

/* Callback when timer expires */
ar_result_t cu_fwk_extn_soft_timer_expired(cu_base_t *cu_ptr, uint32_t ch_bit_index)
{
   return AR_EOK;
}

/* Called during graph close with closing subgraph list, destroy all timers in the list */
void cu_fwk_extn_soft_timer_destroy_at_close(cu_base_t *base_ptr, gu_module_t *module_ptr)
{
}
