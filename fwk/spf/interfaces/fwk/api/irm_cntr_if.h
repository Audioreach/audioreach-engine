#ifndef _IRM_CNTR_IF_H_
#define _IRM_CNTR_IF_H_

/**
 * \file irm_cntr_if.h
 *
 * \brief
 *     This file defines IRM to container functions.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**********************************************************************************************************************/
/**
  @ingroup PARAM_ID_GET_CNTR_PROFILING_INFO
  This param is used to by IRM to indicate enable/disable profiling
  for mpdules present in the container. It also requires containers
  to fill the metric payload ptr info

  @gpr_hdr_fields
  Opcode -- PARAM_ID_GET_CNTR_PROFILING_INFO

  @msg_payload
  param_id_cntr_instance_handles_t

  @return
  None

  @dependencies
  None
 */
#define CNTR_PARAM_ID_GET_PROF_INFO 0x08001365

/**********************************************************************************************************************/

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

struct cntr_param_id_get_prof_info_t
{
   uint32_t is_enable;
   /**< 1 = enable, 0 = disable */

   POSAL_HEAP_ID heap_id;
   /**< Heap ID from which container must use to allocate metric payloads */

   posal_mutex_t *cntr_mutex_ptr;
   /**< posal_mutex_t is of type void*, so we need mutex ptr*/

   POSAL_HEAP_ID cntr_heap_id;
   /**< Heap ID of the container */

   uint32_t num_modules;
   /**< Number of modules */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct cntr_param_id_get_prof_info_t cntr_param_id_get_prof_info_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

struct cntr_param_id_mod_metric_info_t
{
   uint32_t instance_id;
   /**< Instance ID of the module */

   POSAL_HEAP_ID module_heap_id;
   /**< Heap ID of the module */

   uint32_t num_metrics;
   /**< number of metrics enabled */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct cntr_param_id_mod_metric_info_t cntr_param_id_mod_metric_info_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

struct cntr_param_id_mod_metric_payload_t
{
   uint32_t metric_id;
   /**< Metric ID */

   void *payload_ptr;
   /**< Ptr to the metric payload allocated by containers */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct cntr_param_id_mod_metric_payload_t cntr_param_id_mod_metric_payload_t;

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _IRM_CNTR_IF_H_
