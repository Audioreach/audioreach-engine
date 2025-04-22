/**
 * \file wear_cntr_pm.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "wear_cntr_i.h"
#define WCNTR_PCPPx2_DEFAULT 7

static ar_result_t wcntr_util_handle_clk_vote_change(wcntr_base_t *me_ptr,
                                                        bool_t     is_release,
                                                        bool_t     force_vote,
                                                        uint32_t   total_kpps,
                                                        uint32_t   total_bw,
                                                        uint32_t   scaled_kpps_agg_q4);

static ar_result_t wcntr_vote_latency(wcntr_base_t *me_ptr, bool_t is_release, bool_t is_realtime_usecase);

//Not external ports for wear container. 
// Hence voting will be based on modules kpps and bandwidth
// in wcntr_handle_clk_vote_change
ar_result_t wcntr_update_cntr_kpps_bw(wcntr_t *me_ptr)
{

   return AR_EOK;
}

/**
 * force_vote doesn't matter if is_release=TRUE
 *
 * is_release helps in releasing BW even when aggregated BW is nonzero, useful for suspend.
 *
 * force_vote helps in voting BW due to changes in svc & not due to CAPI events.
 */
static ar_result_t wcntr_handle_clk_vote_change(wcntr_t *me_ptr,
                                                   bool_t      is_release,
                                                   bool_t      only_aggregate,
                                                   uint32_t *  kpps_ptr,
                                                   uint32_t *  bw_ptr)
{
   ar_result_t result = AR_EOK;

   uint32_t      topo_kpps = 0, topo_bw = 0, scaled_kpps_agg_q4 = 0;
   wcntr_pm_info_t *pm_info_ptr = &me_ptr->cu.pm_info;

   bool_t is_request = !is_release;

   if (is_request || only_aggregate)
   {
      wcntr_topo_aggregate_kpps_bandwidth(&me_ptr->topo, only_aggregate, &topo_kpps, &topo_bw, &scaled_kpps_agg_q4);

      topo_kpps += pm_info_ptr->cntr_kpps;
      scaled_kpps_agg_q4 += (pm_info_ptr->cntr_kpps * WCNTR_UNITY_Q4);
      // if we don't add then avg scale factor will be wrong in wcntr_handle_clk_vote_change
      topo_bw += pm_info_ptr->cntr_bw;
   }

   /* Vote the aggregated BW and KPPS using utility */
   if (!only_aggregate)
   {
      /** force voting: vote only if diff. some SPR tests went signal miss due to run time voting*/
      result = wcntr_util_handle_clk_vote_change(&me_ptr->cu,
                                                    is_release,
                                                    FALSE /*force_vote*/,
                                                    topo_kpps,
                                                    topo_bw,
                                                    scaled_kpps_agg_q4);
   }

   SET_IF_NOT_NULL(kpps_ptr, topo_kpps);
   SET_IF_NOT_NULL(bw_ptr, topo_bw);

   return result;
}

ar_result_t wcntr_aggregate_kpps_bw(void *cu_ptr, uint32_t *kpps_ptr, uint32_t *bw_ptr)
{
   wcntr_t *me_ptr = (wcntr_t *)cu_ptr;
   wcntr_update_cntr_kpps_bw(me_ptr);
   return wcntr_handle_clk_vote_change(me_ptr, FALSE /* is_release */, TRUE /* only aggregate */, kpps_ptr, bw_ptr);
}

/**
 * conditionally votes
 * - core and bus clocks
 * - latency vote
 * - thread priority
 *
 * core & bus clocks:
 *    - Module events KPPS/BW (Process state) KPPS Scale factor change
 *    -
 * Latency vote:
 *    - KPPS Scale factor change causing container proc duration change
 *    - Threshold change resulting in container frame duration change
 * Thread priority
 *    - same as latency
 *
 * KPPS/BW/latency Need to be voted only if at least one SG is started 
 * Removing votes: All SG are stopped (data flow not checked).
 * Thread priority would be lower for FTRT cases.
 *
 * In vote_core_n_bus_clock, where we iterate over all ports, we need to, ideally, consider KPPS/BW val only
 * for those ports for which data flow has begun. However, such checks would not only be complicated, but also
 * suffer from an issue. Unless data flow happens, port state cannot be moved to data-flow-started state, however,
 * to have the data flow, power voting is necessary. In order to avoid such circular dependency, simple route is taken:
 * All started ports contribute to power voting independent of data flow state.
 */
ar_result_t wcntr_perf_vote(wcntr_t *me_ptr)
{
   ar_result_t result = AR_EOK;

   bool_t is_release = !me_ptr->cu.flags.is_cntr_started;

   if (is_release || me_ptr->cu.fwk_evt_flags.kpps_bw_scale_factor_change || me_ptr->topo.capi_event_flag.kpps ||
       me_ptr->topo.capi_event_flag.bw || me_ptr->cu.fwk_evt_flags.sg_state_change)
   {
      wcntr_handle_clk_vote_change(me_ptr, is_release, FALSE /* only aggregate */, NULL, NULL);
   }

   if (is_release || me_ptr->cu.fwk_evt_flags.cntr_run_state_change)
   {
      wcntr_get_set_thread_priority(me_ptr, NULL, TRUE /*should set */);
      wcntr_vote_latency(&me_ptr->cu,
                                 is_release,
                                 (wcntr_is_signal_triggered(me_ptr)));
   }

   return result;
}

ar_result_t wcntr_register_with_pm(wcntr_base_t *me_ptr)
{
   if (posal_power_mgr_is_registered(me_ptr->pm_info.pm_handle_ptr))
   {
      return AR_EOK;
   }

   // Set container to send only synchronous requests
   me_ptr->pm_info.is_request_async = FALSE;

   return posal_power_mgr_register(me_ptr->pm_info.register_info,
                                   &me_ptr->pm_info.pm_handle_ptr,
                                   me_ptr->gp_signal_ptr,
                                   me_ptr->gu_ptr->log_id);
}

ar_result_t wcntr_deregister_with_pm(wcntr_base_t *me_ptr)
{
   if (!posal_power_mgr_is_registered(me_ptr->pm_info.pm_handle_ptr))
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "PM deregister failed, client null");
      return AR_EOK;
   }

   return posal_power_mgr_deregister(&me_ptr->pm_info.pm_handle_ptr, me_ptr->gu_ptr->log_id);
}

static ar_result_t wcntr_vote_latency(wcntr_base_t *me_ptr, bool_t is_release, bool_t is_realtime_usecase)
{
   uint32_t    new_vote = 0;
   ar_result_t result   = AR_EOK;

   // also release votes if container proc duration is not yet known.
   is_release = is_release || (0 == me_ptr->cntr_proc_duration);

   bool_t previously_released =
      ((0 == me_ptr->pm_info.prev_latency_vote) || (WCNTR_LATENCY_VOTE_MAX == me_ptr->pm_info.prev_latency_vote));

   // if we didn't vote even once, and this call is for release, then return
   if (is_release && previously_released)
   {
      return AR_EOK;
   }

   if (!posal_power_mgr_is_registered(me_ptr->pm_info.pm_handle_ptr))
   {
      return AR_EOK;
   }

   // If no sub graphs are started, is_release will be false -> we need to release, voting value doesn't matter
   if (!is_release)
   {
      // The latency vote of a container is computed as a function of its frame length & nature of usecase
      // For RT/NRT use cases, the tolerance factor for latency is 3%/70% respectively
      // Note : This assumes that frame length is the processing time of the container
      uint32_t tolerance_factor = (is_realtime_usecase) ? WCNTR_LATENCY_VOTE_RT_FACTOR : WCNTR_LATENCY_VOTE_NRT_FACTOR;

      // Example: For 1 ms frame_len_us latency votes for RT/NRT are 30us and 700us respectively
      new_vote = (tolerance_factor * me_ptr->cntr_proc_duration) / 100;
   }
   else // release
   {
      new_vote = WCNTR_LATENCY_VOTE_MAX;
   }

   if (me_ptr->pm_info.prev_latency_vote != new_vote)
   {
      if (is_release)
      {
         posal_pm_release_info_t release;
         memset(&release, 0, sizeof(posal_pm_release_info_t));

         release.pm_handle_ptr                    = me_ptr->pm_info.pm_handle_ptr;
         release.client_log_id                    = me_ptr->gu_ptr->log_id;
         release.wait_signal_ptr                  = me_ptr->gp_signal_ptr;
         release.resources.sleep_latency.is_valid = TRUE;
         release.resources.sleep_latency.value    = new_vote;
         result                                   = posal_power_mgr_release(&release);
      }
      else
      {
         posal_pm_request_info_t request;
         memset(&request, 0, sizeof(posal_pm_request_info_t));

         request.pm_handle_ptr                    = me_ptr->pm_info.pm_handle_ptr;
         request.client_log_id                    = me_ptr->gu_ptr->log_id;
         request.wait_signal_ptr                  = me_ptr->gp_signal_ptr;
         request.resources.sleep_latency.is_valid = TRUE;
         request.resources.sleep_latency.value    = new_vote;
         result                                   = posal_power_mgr_request(&request);
      }

      if (AR_EOK == result)
      {
         WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_LOW_PRIO,
                SPF_LOG_PREFIX "Success: Latency Vote, is_release = %d, new_vote = %d, previous vote = %d",
                is_release,
                new_vote,
                me_ptr->pm_info.prev_latency_vote);
         me_ptr->pm_info.prev_latency_vote = new_vote;
      }
      else
      {
         WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "Failed: Latency Vote, is_release = %d, new_vote = %d, previous vote = %d",
                is_release,
                new_vote,
                me_ptr->pm_info.prev_latency_vote);
      }
   }
   return result;
}

static ar_result_t wcntr_util_handle_clk_vote_change(wcntr_base_t *me_ptr,
                                                        bool_t     is_release,
                                                        bool_t     force_vote,
                                                        uint32_t   total_kpps,
                                                        uint32_t   total_bw,
                                                        uint32_t   scaled_kpps_agg_q4)
{
   ar_result_t result = AR_EOK;

   /* Multiply the scale factor and pCPP to the current KPPS value and vote with new floor clock.
   Floor clock  = old_kpps * scale_factor * (WCNTR_PCPPx2_DEFAULT)/2
   Considering PCPP as 3.5, WCNTR_PCPPx2_DEFAULT is 7 for the ease of calculations. */

   /* Scale kpps with integral part, floor_clock will be in Hz*/
   uint64_t floor_clock = 0;

   // if scaled kpps is same as topo kpps then all scale factors are unity; no need of voting floor clock.
   if (scaled_kpps_agg_q4 != (total_kpps << 4))
   {
      /*shift right by 4 to convert to q4 format */
      floor_clock = ((uint64_t)scaled_kpps_agg_q4 * WCNTR_PCPPx2_DEFAULT * 1000);

      /* Need to rotate 4 + 1 times to divide by 32, 4 rotations to account for 4 fractional bits in scale
       * factor and one rotation to divide WCNTR_PCPPx2_DEFAULT by 2 to get the actual PCPP. */
      floor_clock = (floor_clock >> 5);

      WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_LOW_PRIO,
             "scaled_kpps_agg %lu, agg kpps %lu, floor_clock hz lsw: %lu msw: %lu",
             (scaled_kpps_agg_q4 >> 4),
             total_kpps,
			 (uint32_t)floor_clock,
			 (uint32_t)(floor_clock >> 32));
   }

   // If there was no event or no release-call, or no force vote or there was
   // no change, return.
   if (!((me_ptr->pm_info.prev_kpps_vote != total_kpps) || (me_ptr->pm_info.prev_bw_vote != total_bw) ||
         (me_ptr->pm_info.prev_floor_clock_vote != floor_clock) || force_vote))
   {
      return AR_EOK;
   }

   // if we didn't vote even once, and this call is for release, then return
   if (is_release && ((0 == me_ptr->pm_info.prev_kpps_vote) && (0 == me_ptr->pm_info.prev_bw_vote) &&
                      (0 == me_ptr->pm_info.prev_floor_clock_vote)))
   {
      return AR_EOK;
   }

   if (!posal_power_mgr_is_registered(me_ptr->pm_info.pm_handle_ptr))
   {
      return AR_EOK;
   }

   // whether mips req is made
   bool_t is_mips_req = ((me_ptr->pm_info.prev_kpps_vote != total_kpps) || is_release ||
                         (me_ptr->pm_info.prev_floor_clock_vote != floor_clock));
   // whether bw req is made
   bool_t is_bw_req = ((me_ptr->pm_info.prev_bw_vote != total_bw) || is_release);

   if (is_mips_req)
   {
      me_ptr->pm_info.prev_kpps_vote        = total_kpps;
      me_ptr->pm_info.prev_floor_clock_vote = floor_clock;
   }

   if (is_bw_req)
   {
      me_ptr->pm_info.prev_bw_vote = total_bw;
   }

   if (is_release)
   {
      posal_pm_release_info_t release;
      memset(&release, 0, sizeof(posal_pm_release_info_t));

      release.pm_handle_ptr            = me_ptr->pm_info.pm_handle_ptr;
      release.client_log_id            = me_ptr->gu_ptr->log_id;
      release.wait_signal_ptr          = me_ptr->gp_signal_ptr;
      release.resources.mpps.is_valid  = is_mips_req;
      release.resources.mpps.floor_clk = floor_clock;// floor_clock will be 0 by default
      release.resources.mpps.value     = total_kpps / 1000;
      release.resources.bw.is_valid    = is_bw_req;
      release.resources.bw.value       = total_bw;
      result                           = posal_power_mgr_release(&release);
   }
   else
   {
      posal_pm_request_info_t request;
      memset(&request, 0, sizeof(posal_pm_request_info_t));

      request.pm_handle_ptr            = me_ptr->pm_info.pm_handle_ptr;
      request.client_log_id            = me_ptr->gu_ptr->log_id;
      request.wait_signal_ptr          = me_ptr->gp_signal_ptr;
      request.resources.mpps.is_valid  = TRUE;
      request.resources.mpps.floor_clk = floor_clock; // floor_clock will be 0 by default
      request.resources.mpps.value     = total_kpps / 1000;
      request.resources.bw.is_valid    = TRUE;
      request.resources.bw.value       = total_bw;
      result                           = posal_power_mgr_request(&request);
   }

   me_ptr->pm_info.weighted_kpps_scale_factor_q4 = WCNTR_UNITY_Q4;
   // update only upon voting. Otherwise, voting and below value will not be in sync.
   if (scaled_kpps_agg_q4 != (total_kpps << 4))
   {
      uint32_t weighted_kpps_scale_q4 = total_kpps ? (scaled_kpps_agg_q4 / total_kpps) : WCNTR_UNITY_Q4;

      if (weighted_kpps_scale_q4)
      {
         me_ptr->pm_info.weighted_kpps_scale_factor_q4 = weighted_kpps_scale_q4;
      }
   }

   WCNTR_MSG(me_ptr->gu_ptr->log_id,
            DBG_HIGH_PRIO,
            SPF_LOG_PREFIX
            "Vote kpps bw: kpps %lu, bw %lu Bps, floor_clock = %lu. Container proc duration %lu us, weighted scale "
            "factor Q4 %lx is_release %u",
            total_kpps,
            total_bw,
            (uint32_t)floor_clock,
            me_ptr->cntr_proc_duration,
            me_ptr->pm_info.weighted_kpps_scale_factor_q4,is_release);

   return result;
}
