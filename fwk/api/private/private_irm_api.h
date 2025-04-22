/**
@file private_irm_api.h

@brief Private internal api for Integrated Resource Monitor (IRM).

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#ifndef _PRIVATE_IRM_API_H_
#define _PRIVATE_IRM_API_H_

#define IRM_METRIC_ID_STACK_INFO 0x08001504

/** @weakgroup weak_irm_metric_id_stack_info_t
@{ */
#include "spf_begin_pack.h"
struct irm_metric_id_stack_info_t
{
   uint32_t current_stack_usage;
   /**< Current stack usage (in bytes). */

   uint32_t stack_size;
   /**< Maximum allowed stack size (in bytes). */
}
#include "spf_end_pack.h"
;
/** @} */ /* end_weakgroup weak_irm_metric_id_stack_info_t */
typedef struct irm_metric_id_stack_info_t irm_metric_id_stack_info_t;


#endif //_PRIVATE_IRM_API_H_