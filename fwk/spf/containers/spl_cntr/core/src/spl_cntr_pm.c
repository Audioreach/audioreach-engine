/**
 * \file spl_cntr_pm.c
 * \brief
 *     This file contains container spl_cntr power manager functions.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spl_cntr_i.h"

/* =======================================================================
Public Functions
========================================================================== */
ar_result_t spl_cntr_handle_clk_vote_change(spl_cntr_t       *me_ptr,
                                            cu_pm_vote_type_t vote_type,
                                            bool_t            force_vote,
                                            bool_t            force_aggregate,
                                            uint32_t         *kpps_ptr,
                                            uint32_t         *bw_ptr)
{
   ar_result_t   result             = AR_EOK;
   uint32_t      topo_kpps          = 0;
   uint32_t      topo_bw            = 0;
   uint32_t      scaled_kpps_agg_q4 = 0;
   uint32_t      scaled_bw_agg      = 0;
   cu_pm_info_t *pm_info_ptr        = &me_ptr->cu.pm_info;

   if ((CU_PM_REQ_KPPS_BW == vote_type) || force_aggregate)
   {
      spl_topo_aggregate_kpps_bandwidth(&me_ptr->topo,
                                        force_aggregate,
                                        &topo_kpps,
                                        &topo_bw,
                                        &scaled_kpps_agg_q4,
                                        &scaled_bw_agg);

      topo_kpps += pm_info_ptr->cntr_kpps;
      topo_bw += pm_info_ptr->cntr_bw;
      scaled_kpps_agg_q4 += (pm_info_ptr->cntr_kpps * UNITY_Q4);
      scaled_bw_agg += pm_info_ptr->cntr_bw;
   }

   if (!force_aggregate)
   {
      cu_handle_clk_vote_change(&me_ptr->cu,
                                vote_type,
                                force_vote,
                                topo_kpps,
                                scaled_bw_agg,
                                scaled_kpps_agg_q4,
                                FALSE /*nonblocking*/);
   }

   SET_IF_NOT_NULL(kpps_ptr, topo_kpps);
   SET_IF_NOT_NULL(bw_ptr, topo_bw);

   return result;
}

/*
 * container kpps vote estimation based on the data rate from the external ports
 * TODO: BW update
 */
ar_result_t spl_cntr_update_cntr_kpps_bw(spl_cntr_t *me_ptr, bool_t force_aggregate)
{

   uint32_t kpps = 0;

   for (gu_ext_in_port_list_t *ext_in_port_list_ptr = me_ptr->topo.t_base.gu.ext_in_port_list_ptr;
        (NULL != ext_in_port_list_ptr);
        LIST_ADVANCE(ext_in_port_list_ptr))
   {
      spl_cntr_ext_in_port_t *ext_in_port_ptr = (spl_cntr_ext_in_port_t *)ext_in_port_list_ptr->ext_in_port_ptr;
      topo_port_state_t in_port_state = ((gen_topo_input_port_t *)ext_in_port_ptr->gu.int_in_port_ptr)->common.state;

      // For force aggregation don't need to check state.
      if (!force_aggregate)
      {
         // dont update container bw unless it is in started state
         if (!(TOPO_PORT_STATE_STARTED == in_port_state))
         {
            continue;
         }
      }

      topo_media_fmt_t *media_fmt_ptr = &ext_in_port_ptr->cu.media_fmt;

      if (SPF_IS_PACKETIZED_OR_PCM(media_fmt_ptr->data_format))
      {
         kpps += topo_get_memscpy_kpps(media_fmt_ptr->pcm.bits_per_sample,
                                       media_fmt_ptr->pcm.num_channels,
                                       media_fmt_ptr->pcm.sample_rate);
      }
   }

   // Add each external output port's bw to the total bw.
   for (gu_ext_out_port_list_t *ext_out_port_list_ptr = me_ptr->topo.t_base.gu.ext_out_port_list_ptr;
        (NULL != ext_out_port_list_ptr);
        LIST_ADVANCE(ext_out_port_list_ptr))
   {
      spl_cntr_ext_out_port_t *ext_out_port_ptr = (spl_cntr_ext_out_port_t *)ext_out_port_list_ptr->ext_out_port_ptr;
      topo_media_fmt_t        *media_fmt_ptr    = &ext_out_port_ptr->cu.media_fmt;

      topo_port_state_t out_port_state = ((gen_topo_input_port_t *)ext_out_port_ptr->gu.int_out_port_ptr)->common.state;

      // if we are forcing aggregation, even prepared state is allowed
      if (!force_aggregate)
      {
         // dont update container bw unless it is in started state
         if (!(TOPO_PORT_STATE_STARTED == out_port_state))
         {
            continue;
         }
      }

      if (SPF_IS_PACKETIZED_OR_PCM(media_fmt_ptr->data_format))
      {
         kpps += topo_get_memscpy_kpps(media_fmt_ptr->pcm.bits_per_sample,
                                       media_fmt_ptr->pcm.num_channels,
                                       media_fmt_ptr->pcm.sample_rate);
      }
   }

   if (kpps != me_ptr->cu.pm_info.cntr_kpps)
   {
      me_ptr->cu.pm_info.cntr_kpps = kpps;
      SPL_CNTR_MSG(me_ptr->topo.t_base.gu.log_id, DBG_LOW_PRIO, " Container KPPS %lu", kpps);
   }

   return AR_EOK;
}

ar_result_t spl_cntr_aggregate_kpps_bw(void *cu_ptr, uint32_t *kpps_ptr, uint32_t *bw_ptr)
{
   spl_cntr_t *me_ptr = (spl_cntr_t *)cu_ptr;
   spl_cntr_update_cntr_kpps_bw(me_ptr, TRUE /*force_aggregate*/);
   return spl_cntr_handle_clk_vote_change(me_ptr,
                                          CU_PM_REQ_KPPS_BW /* is_release */,
                                          FALSE /* force_vote*/,
                                          TRUE /* force aggregate */,
                                          kpps_ptr,
                                          bw_ptr);
}
