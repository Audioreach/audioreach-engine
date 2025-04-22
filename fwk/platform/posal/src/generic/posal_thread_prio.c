/**
 * \file posal_thread_prio.c
 * \brief
 *   This file contains functional implementations that are exposed to the framework to be invoked in order to retrieve the thread priority.
 *
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "posal_thread_attr_def.h"
#include "posal_thread_attr_cfg_i.h"
#include "ar_msg.h"
/* =======================================================================
**                          Function Definitions
** ======================================================================= */

// clang-format on

/* =======================================================================
**                          Function Definitions
** ======================================================================= */

/**
 * Gets a default low priority for the passed in prio_id. Currently only implemented
 * for SPF_THREAD_STAT_CNTR_ID.
 */
static posal_thread_prio_t spf_thread_calc_floor_prio(spf_thread_prio_id_t prio_id)
{
   bool_t              found      = FALSE;
   posal_thread_prio_t prio       = SPF_THREAD_PRIO_LOWEST;

  if (prio_id == SPF_THREAD_STAT_CNTR_ID)
  {
	 prio  = SPF_CNTR_FLOOR_THREAD_PRIO;
	 found = TRUE;
  }

	if (!found)
	{
	  AR_MSG(DBG_ERROR_PRIO, "Prio id 0x%lx does not have a floor priority, returning the lowest", prio_id);
	}

   return prio;
}

/*
 * Calculates the thread priority of a thread given a
 * duration in microseconds OR static thread ID.
 *
 * @param[in]  priority query
 *
 * @return
 * Thread priority value
 */
ar_result_t posal_thread_calc_prio(prio_query_t *prio_query_ptr, posal_thread_prio_t *thread_prio_ptr)
{
   if (NULL == prio_query_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Received null ptr for the thread prio query");
      return AR_EBADPARAM;
   }
   uint32_t sched_pol, affinity_mask;

   if (SPF_THREAD_DYN_ID == prio_query_ptr->static_req_id)
   {
      spf_thread_determine_dyn_attr(prio_query_ptr->frame_duration_us, prio_query_ptr->is_interrupt_trig, thread_prio_ptr, &sched_pol, &affinity_mask);
   }
   else
   {
      spf_thread_determine_static_attr(prio_query_ptr->static_req_id, prio_query_ptr->is_interrupt_trig, thread_prio_ptr, &sched_pol, &affinity_mask);
   }
   return AR_EOK;
}

ar_result_t posal_thread_determine_attributes(prio_query_t *prio_query_ptr, posal_thread_prio_t *thread_prio_ptr, uint32_t *sched_policy_ptr, uint32_t *affinity_mask_ptr)
{
   if (NULL == prio_query_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "Received null ptr for the thread attr query");
      return AR_EBADPARAM;
   }
   if (SPF_THREAD_DYN_ID == prio_query_ptr->static_req_id)
   {
      spf_thread_determine_dyn_attr(prio_query_ptr->frame_duration_us, prio_query_ptr->is_interrupt_trig, thread_prio_ptr, sched_policy_ptr, affinity_mask_ptr);
   }
   else
   {
      spf_thread_determine_static_attr(prio_query_ptr->static_req_id, prio_query_ptr->is_interrupt_trig, thread_prio_ptr, sched_policy_ptr, affinity_mask_ptr);
   }
   return AR_EOK;
}

/**
 * Gets a default low priority for the passed in prio_id. Currently only implemented
 * for SPF_THREAD_STAT_CNTR_ID.
 */
posal_thread_prio_t posal_thread_get_floor_prio(spf_thread_prio_id_t prio_id)
{
   return spf_thread_calc_floor_prio(prio_id);
}
