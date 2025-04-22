/**
 * \file capi_cmn_island.c
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_cmn.h"

/*=====================================================================
  Functions
 ======================================================================*/

capi_err_t capi_cmn_raise_island_vote_event(capi_event_callback_info_t *cb_info_ptr, bool_t island_vote)
{
   if (NULL == cb_info_ptr->event_cb)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_cmn : Event callback is not set, Unable to raise island vote event!");
      return CAPI_EBADPARAM;
   }

   capi_err_t               result = CAPI_EOK;
   capi_event_island_vote_t event;
   event.island_vote = island_vote;
   capi_event_info_t event_info;
   event_info.port_info.is_valid      = FALSE;
   event_info.payload.actual_data_len = event_info.payload.max_data_len = sizeof(capi_event_island_vote_t);
   event_info.payload.data_ptr                                          = (int8_t *)(&event);
   result = cb_info_ptr->event_cb(cb_info_ptr->event_context, CAPI_EVENT_ISLAND_VOTE, &event_info);
   if (CAPI_FAILED(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_cmn : Failed to send Island vote event %lu update event", island_vote);
   }
   return result;
}

capi_err_t capi_cmn_update_kpps_event(capi_event_callback_info_t *cb_info_ptr, uint32_t kpps)
{
   if (NULL == cb_info_ptr->event_cb)
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_cmn : Event callback is not set, Unable to raise kpps event!");
      return CAPI_EBADPARAM;
   }

   capi_err_t        result = CAPI_EOK;
   capi_event_KPPS_t event;
   event.KPPS = kpps;
   capi_event_info_t event_info;
   event_info.port_info.is_valid      = FALSE;
   event_info.payload.actual_data_len = event_info.payload.max_data_len = sizeof(capi_event_KPPS_t);
   event_info.payload.data_ptr                                          = (int8_t *)(&event);
   result = cb_info_ptr->event_cb(cb_info_ptr->event_context, CAPI_EVENT_KPPS, &event_info);
   if (CAPI_FAILED(result))
   {
      AR_MSG(DBG_ERROR_PRIO, "capi_cmn : Failed to send KPPS update event with %lu", result);
   }
   return result;
}

void capi_cmn_check_print_underrun(capi_cmn_underrun_info_t *underrun_info_ptr, uint32_t iid)
{
   underrun_info_ptr->underrun_counter++;
   uint64_t curr_time = posal_timer_get_time();
   uint64_t diff      = curr_time - underrun_info_ptr->prev_time;
   if ((diff >= CAPI_CMN_STEADY_STATE_UNDERRUN_TIME_THRESH_US) || (0 == underrun_info_ptr->prev_time))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
                    "MODULE:%08lX, Underrun detected, Count:%ld, time since prev print: %ld us",
                    iid,
                    underrun_info_ptr->underrun_counter,
                    diff);
      underrun_info_ptr->prev_time        = curr_time;
      underrun_info_ptr->underrun_counter = 0;
   }

   return;
}

void capi_cmn_check_print_underrun_multiple_threshold(capi_cmn_underrun_info_t *underrun_info_ptr,
                                                      uint32_t                  iid,
                                                      bool                      need_to_reduce_underrun_print,
                                                      bool_t                    marker_eos,
                                                      bool_t                    is_capi_in_media_fmt_set)
{
   underrun_info_ptr->underrun_counter++;
   uint64_t curr_time = posal_timer_get_time();
   uint64_t diff      = curr_time - underrun_info_ptr->prev_time;
   uint64_t threshold = CAPI_CMN_UNDERRUN_TIME_THRESH_US;
   if (!need_to_reduce_underrun_print)
   {
      threshold = CAPI_CMN_STEADY_STATE_UNDERRUN_TIME_THRESH_US;
   }

   if ((diff >= threshold) || (0 == underrun_info_ptr->prev_time))
   {
      AR_MSG_ISLAND(DBG_ERROR_PRIO,
                    "MODULE:%08lX, Underrun detected, Count:%ld, time since prev print: %ld us, marker_eos: %d , "
                    "media_format_set: %d",
                    iid,
                    underrun_info_ptr->underrun_counter,
                    diff,
                    marker_eos,
                    is_capi_in_media_fmt_set);
      underrun_info_ptr->prev_time        = curr_time;
      underrun_info_ptr->underrun_counter = 0;
   }

   return;
}
