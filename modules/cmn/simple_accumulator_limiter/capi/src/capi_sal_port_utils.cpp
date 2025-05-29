/* ======================================================================== */
/**
   @file capi_sal_port_utils.cpp

   Source file to implement port specific utility functions called by the
   CAPI Interface for Simple Accumulator-Limiter (SAL) Module.
*/

/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================== */

/*==========================================================================
Include files
========================================================================== */
#include "capi_sal_utils.h"

extern capi_err_t capi_sal_set_input_process_info(capi_sal_t *me_ptr);

uint32_t capi_sal_get_new_ref_port_index(capi_sal_t *me_ptr, uint32_t curr_ref_port_idx)
{
   uint32_t same_mf_index  = SAL_INVALID_PORT_IDX;
   uint32_t other_mf_index = SAL_INVALID_PORT_IDX;
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      if (curr_ref_port_idx == i)
      {
         continue;
      }
      if ((DATA_PORT_STARTED == me_ptr->in_port_arr[i].state) && (me_ptr->in_port_arr[i].port_flags.mf_rcvd))
      {
         if ((FALSE == me_ptr->in_port_arr[i].port_flags.data_drop))
         {
            same_mf_index = i;
            break;
         }
         else // data_drop = true =>different media format (akr TBD: have
              // ranking: if other mf, most matches)
         {
            other_mf_index = i;
         }
      }
   }
   return ((same_mf_index == SAL_INVALID_PORT_IDX) ? other_mf_index : same_mf_index);
}

static inline void capi_sal_handle_data_port_stop(capi_sal_t *me_ptr, uint32_t data_port_index)
{
   if (0 == me_ptr->num_in_ports_started)
   {
      return;
   }

   // set stopped port to invalid index and decrement num_in_ports_started
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      if ((int32_t)data_port_index == me_ptr->started_in_port_index_arr[i])
      {
         me_ptr->started_in_port_index_arr[i] = -1;
         me_ptr->num_in_ports_started--;
         break;
      }
   }

   if (0 == me_ptr->num_in_ports_started)
   {
      return;
   }

   // reset values in started port list to remove any holes created by stopped port
   uint32_t num_ports_started = 0;
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      int32_t port_index = me_ptr->started_in_port_index_arr[i];
      if (-1 != port_index)
      {
         me_ptr->started_in_port_index_arr[num_ports_started] = port_index;
         if (num_ports_started != i)
         {
            me_ptr->started_in_port_index_arr[i] = -1;
         }
         num_ports_started++;
      }
      if (me_ptr->num_in_ports_started == num_ports_started)
      {
         break;
      }
   }

   if (num_ports_started != me_ptr->num_in_ports_started)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "Unexepected result - num_ports_started: %lu should be same as num_in_ports_started: %lu",
             num_ports_started,
             me_ptr->num_in_ports_started);
   }
}

static inline void capi_sal_handle_data_port_start(capi_sal_t *me_ptr, uint32_t data_port_index)
{
   if ((me_ptr->num_in_ports_started < me_ptr->num_in_ports) &&
       (-1 == me_ptr->started_in_port_index_arr[me_ptr->num_in_ports_started]))
   {
      me_ptr->started_in_port_index_arr[me_ptr->num_in_ports_started] = data_port_index;
      me_ptr->num_in_ports_started++;
   }
}

capi_err_t capi_sal_handle_data_flow_stop(capi_sal_t *me_ptr, uint32_t data_port_index, bool_t data_produced)
{
   capi_err_t result                          = CAPI_EOK;
   me_ptr->in_port_arr[data_port_index].state = DATA_PORT_STOPPED;
   capi_sal_handle_data_port_stop(me_ptr, data_port_index);

   if (me_ptr->in_port_arr[data_port_index].port_flags.is_ref_port)
   {
      me_ptr->in_port_arr[data_port_index].port_flags.is_ref_port = FALSE;
      me_ptr->in_port_arr[data_port_index].port_flags.data_drop   = TRUE; // won't be accumulated

      // to pick a new ref_port, we need to iterate and check who satisfies the
      // conditions
      uint32_t index = capi_sal_get_new_ref_port_index(me_ptr, data_port_index);

      SAL_MSG_ISLAND(me_ptr->iid, DBG_MED_PRIO, "new ref port_index %lu", index);

      if (SAL_INVALID_PORT_IDX != index)
      {
         // accept and raise mf if it is different from the current operating mf
         if (FALSE == capi_cmn_media_fmt_equal(me_ptr->operating_mf_ptr, &me_ptr->in_port_arr[index].mf))
         {
            bool_t MF_RAISED_UNUSED  = FALSE;
            me_ptr->operating_mf_ptr = NULL;

            capi_sal_accept_omf_alloc_mem_and_raise_events(me_ptr, index, data_produced, &MF_RAISED_UNUSED);
         }
         else
         {
            me_ptr->operating_mf_ptr                      = &me_ptr->in_port_arr[index].mf;
            me_ptr->module_flags.raise_mf_on_next_process = FALSE;
         }

         capi_sal_assign_ref_port(me_ptr, index);
         me_ptr->in_port_arr[index].port_flags.data_drop = FALSE; // will be accumulated

         capi_sal_compare_port_mfs_to_omf_and_asign_data_drops(me_ptr);

         if (SAL_PARAM_NATIVE == me_ptr->bps_cfg_mode)
         {
            me_ptr->out_port_cache_cfg.q_factor = me_ptr->operating_mf_ptr->format.q_factor;
            me_ptr->out_port_cache_cfg.word_size_bytes =
               me_ptr->operating_mf_ptr->format.bits_per_sample >> 3; // will be 2 or 4
         }
         SAL_MSG_ISLAND(me_ptr->iid, DBG_MED_PRIO, "port_index %lu is the new ref port", index);
      }
      else
      {
         // if there are no other eligible ports at stop, there is no omf, so no
         // need to raise
         me_ptr->operating_mf_ptr                      = NULL;
         me_ptr->module_flags.raise_mf_on_next_process = FALSE;
      }
   }
   else
   {
      // non-ref port stop nothing to do
   }

   // figure out if lim needs to be in bypass or not (active siso)

   capi_sal_check_and_update_lim_bypass_mode(me_ptr);

   SAL_MSG_ISLAND(me_ptr->iid,
                  DBG_MED_PRIO,
                  "Port index %lu stopped. lim_bypass %lu inplace %lu",
                  data_port_index,
                  me_ptr->lim_bypass,
                  me_ptr->module_flags.is_inplace);
   capi_sal_print_operating_mf(me_ptr);
   return result;
}

ar_result_t capi_sal_evaluate_non_ref_port_imf(capi_sal_t *         me_ptr,
                                               uint32_t             size_to_copy,
                                               capi_media_fmt_v2_t *media_fmt_ptr,
                                               uint32_t             actual_data_len,
                                               uint32_t             port_index)
{
   // accept the media format into the port struct
   memscpy(&me_ptr->in_port_arr[port_index].mf, size_to_copy, media_fmt_ptr, actual_data_len);
   me_ptr->in_port_arr[port_index].port_flags.mf_rcvd = TRUE;
   // if omf exists, we should validate, and decide data_drop flag
   if (me_ptr->operating_mf_ptr)
   {
      if (FALSE == capi_cmn_media_fmt_equal(media_fmt_ptr, me_ptr->operating_mf_ptr))
      {
         me_ptr->in_port_arr[port_index].port_flags.data_drop = TRUE; // won't be accumulated
         SAL_MSG(me_ptr->iid,
                 DBG_MED_PRIO,
                 "MF received on port index %lu is different than OMF, caching and setting data_drop flag",
                 port_index);
      }
      else
      {
         me_ptr->in_port_arr[port_index].port_flags.data_drop = FALSE; // will be accumulated
      }
   }
   else
   {
      // omf is not set. Check if this port is started, we can set this to be OMF
      if (DATA_PORT_STARTED == me_ptr->in_port_arr[port_index].state) // and we now have mf
      {
         bool_t MF_RAISED_UNUSED = FALSE;
         // we accept as the new operating MF
         capi_sal_accept_omf_alloc_mem_and_raise_events(me_ptr, port_index, FALSE /*data_produced*/, &MF_RAISED_UNUSED);
         capi_sal_assign_ref_port(me_ptr, port_index);
         me_ptr->in_port_arr[port_index].port_flags.data_drop = FALSE; // will be accumulated
         capi_sal_compare_port_mfs_to_omf_and_asign_data_drops(me_ptr);
      }
   }
   return AR_EOK;
}

ar_result_t capi_sal_evaluate_ref_port_imf(capi_sal_t *         me_ptr,
                                           uint32_t             size_to_copy,
                                           capi_media_fmt_v2_t *media_fmt_ptr,
                                           uint32_t             actual_data_len,
                                           uint32_t             port_index)
{
   if (FALSE == capi_cmn_media_fmt_equal(media_fmt_ptr, me_ptr->operating_mf_ptr))
   {
      me_ptr->operating_mf_ptr = NULL;
      // accept the media format into the port struct
      memscpy(&me_ptr->in_port_arr[port_index].mf, size_to_copy, media_fmt_ptr, actual_data_len);
      me_ptr->in_port_arr[port_index].port_flags.mf_rcvd = TRUE;

      SAL_MSG(me_ptr->iid,
              DBG_MED_PRIO,
              "MF received on ref port index %lu is different from its prev mf (OMF).",
              port_index);
      // check if any other port is being accumulated (started, mf received and data_drop = FALSE)
      // if so , that can become omf and this can be data drop

      uint32_t new_index = capi_sal_get_new_ref_port_index(me_ptr, port_index);
      SAL_MSG(me_ptr->iid, DBG_MED_PRIO, "port_index %lu setting omf to NULL", port_index);

      if (SAL_INVALID_PORT_IDX != new_index)
      {
         bool_t MF_RAISED_UNUSED = FALSE;
         capi_sal_accept_omf_alloc_mem_and_raise_events(me_ptr, new_index, FALSE /*data_produced*/, &MF_RAISED_UNUSED);
         capi_sal_assign_ref_port(me_ptr, new_index);
         me_ptr->in_port_arr[new_index].port_flags.data_drop = FALSE; // will be accumulated
         SAL_MSG(me_ptr->iid, DBG_MED_PRIO, "New Ref port %lu", new_index);
      }
      else
      {
         bool_t MF_RAISED_UNUSED = FALSE;
         capi_sal_accept_omf_alloc_mem_and_raise_events(me_ptr, port_index, FALSE /*data_produced*/, &MF_RAISED_UNUSED);
         capi_sal_assign_ref_port(me_ptr, port_index);
         me_ptr->in_port_arr[port_index].port_flags.data_drop = FALSE; // will be accumulated
         SAL_MSG(me_ptr->iid, DBG_MED_PRIO, "Same Ref port %lu, new OMF", port_index);
      }

      capi_sal_compare_port_mfs_to_omf_and_asign_data_drops(me_ptr);
   }
   // else, nothing to do (same as it was)
   return AR_EOK;
}

ar_result_t capi_sal_compare_port_mfs_to_omf_and_asign_data_drops(capi_sal_t *me_ptr)
{
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      if (me_ptr->in_port_arr[i].port_flags.mf_rcvd)
      {
         me_ptr->in_port_arr[i].port_flags.data_drop =
            !capi_cmn_media_fmt_equal(me_ptr->operating_mf_ptr, &me_ptr->in_port_arr[i].mf);
      }
   }
   return AR_EOK;
}

void capi_sal_check_and_update_lim_bypass_mode(capi_sal_t *me_ptr)
{
   uint32_t count          = 0;
   bool_t   can_bypass_lim = FALSE;
   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      if ((DATA_PORT_STARTED == me_ptr->in_port_arr[i].state) &&
          (FALSE == me_ptr->in_port_arr[i].port_flags.data_drop) && (FALSE == me_ptr->in_port_arr[i].port_flags.at_gap))
      {
         count++;
      }
   }

   if (1 >= count)
   {
      can_bypass_lim = TRUE; // siso to lim ->no accumulation ->bypass
   }

   if (SAL_LIM_ENABLED_ALWAYS == me_ptr->limiter_enabled)
   {
      can_bypass_lim = FALSE;
   }

   me_ptr->lim_bypass = can_bypass_lim;

   // update limiter bypass mode
   if (capi_sal_check_limiting_required(me_ptr))
   {
      // check if limiter library is initialized
      if (me_ptr->limiter_memory.mem_ptr)
      {
         SAL_MSG_ISLAND(me_ptr->iid, DBG_HIGH_PRIO, "Setting limiter lib bypass mode to %lu", me_ptr->lim_bypass);

         if (LIMITER_SUCCESS !=
             limiter_set_param(&me_ptr->lib_mem, LIMITER_PARAM_BYPASS, &me_ptr->lim_bypass, sizeof(int32_t)))
         {
            SAL_MSG_ISLAND(me_ptr->iid,
                           DBG_ERROR_PRIO,
                           "Error! limiter lib bypass mode couldnt be updated to %lu",
                           me_ptr->lim_bypass);
         }
      }
   }
   // posal_island_trigger_island_exit();
   // dyn inplace is dependent on lim_bypass flag hence update
   capi_sal_check_and_raise_process_state_events(me_ptr);

   return;
}

/* Called when setting new operating media format */
void capi_sal_print_operating_mf(capi_sal_t *me_ptr)
{
   if (NULL == me_ptr->operating_mf_ptr)
   {
      SAL_MSG(me_ptr->iid, DBG_LOW_PRIO, "Operating MF not set yet");
      return;
   }

   // update input process info struture
   capi_sal_set_input_process_info(me_ptr);

   SAL_MSG(me_ptr->iid,
           DBG_HIGH_PRIO,
           "Input Operating MF: SR = %lu, QF = %lu, BPS = %lu, num_ch = %lu, lim_enabled = %u, op_mf_requires_limiting "
           "= "
           "%u",
           me_ptr->operating_mf_ptr->format.sampling_rate,
           me_ptr->operating_mf_ptr->format.q_factor,
           me_ptr->operating_mf_ptr->format.bits_per_sample,
           me_ptr->operating_mf_ptr->format.num_channels,
           me_ptr->limiter_enabled,
           me_ptr->module_flags.op_mf_requires_limiting);
}

capi_err_t capi_sal_assign_ref_port(capi_sal_t *me_ptr, uint32_t ref_port_index)
{
   capi_err_t result = CAPI_EOK;

   for (uint32_t i = 0; i < me_ptr->num_in_ports; i++)
   {
      me_ptr->in_port_arr[i].port_flags.is_ref_port = (ref_port_index == i);
   }

   return result;
}

bool_t capi_sal_handle_data_flow_start_force_ref(capi_sal_t *me_ptr, uint32_t force_ref_port_idx)
{
   capi_err_t result    = CAPI_EOK;
   bool_t     raised_mf = FALSE;
   if (me_ptr->in_port_arr[force_ref_port_idx].state != DATA_PORT_STARTED)
   {
      me_ptr->in_port_arr[force_ref_port_idx].state = DATA_PORT_STARTED;
      capi_sal_handle_data_port_start(me_ptr, force_ref_port_idx);
   }

   // if operating mf is different than input port mf then reassign operating mf.
   if (!(me_ptr->operating_mf_ptr &&
         capi_cmn_media_fmt_equal(me_ptr->operating_mf_ptr, &me_ptr->in_port_arr[force_ref_port_idx].mf)))
   {
      // this port will be the OMF port since it satisfies all conditions
      result = capi_sal_accept_omf_alloc_mem_and_raise_events(me_ptr,
                                                              force_ref_port_idx,
                                                              FALSE /*data produced*/,
                                                              &raised_mf);
      if (CAPI_EOK == result)
      {
         SAL_MSG_ISLAND(me_ptr->iid, DBG_MED_PRIO, "Raised mf from data flow start %ld", raised_mf);
      }
   }
   me_ptr->operating_mf_ptr = &me_ptr->in_port_arr[force_ref_port_idx].mf;
   capi_sal_assign_ref_port(me_ptr, force_ref_port_idx);
   me_ptr->in_port_arr[force_ref_port_idx].port_flags.data_drop = FALSE; // will be accumulated

   capi_sal_compare_port_mfs_to_omf_and_asign_data_drops(me_ptr);

   // set out port bps and qf from here if we're in native mode and we have an
   // operating mf_ptr
   if (SAL_PARAM_NATIVE == me_ptr->bps_cfg_mode)
   {
      me_ptr->out_port_cache_cfg.q_factor = me_ptr->operating_mf_ptr->format.q_factor;
      me_ptr->out_port_cache_cfg.word_size_bytes =
         me_ptr->operating_mf_ptr->format.bits_per_sample >> 3; // will be 2 or 4
   }

   // figure out if lim needs to be in bypass or not (active siso)

   capi_sal_check_and_update_lim_bypass_mode(me_ptr);

   SAL_MSG_ISLAND(me_ptr->iid,
                  DBG_MED_PRIO,
                  "Port index %lu started, is_ref_port = %u lim_bypass %lu is_inplace %lu",
                  force_ref_port_idx,
                  me_ptr->in_port_arr[force_ref_port_idx].port_flags.is_ref_port,
                  me_ptr->lim_bypass,
                  me_ptr->module_flags.is_inplace);

   capi_sal_print_operating_mf(me_ptr);
   return raised_mf;
}

bool_t capi_sal_handle_data_flow_start(capi_sal_t *me_ptr, uint32_t data_port_index)
{
   capi_err_t result    = CAPI_EOK;
   bool_t     raised_mf = FALSE;
   if (me_ptr->in_port_arr[data_port_index].state != DATA_PORT_STARTED)
   {
      me_ptr->in_port_arr[data_port_index].state = DATA_PORT_STARTED;
      capi_sal_handle_data_port_start(me_ptr, data_port_index);
   }

   // If there's no operating media format when we start, set this port to omf if
   // mf was received.
   if ((NULL == me_ptr->operating_mf_ptr) && me_ptr->in_port_arr[data_port_index].port_flags.mf_rcvd &&
       (!me_ptr->in_port_arr[data_port_index].port_flags.is_ref_port))
   {
      // this port will be the OMF port since it satisfies all conditions
      result =
         capi_sal_accept_omf_alloc_mem_and_raise_events(me_ptr, data_port_index, FALSE /*data produced*/, &raised_mf);
      if (CAPI_EOK == result)
      {
         SAL_MSG_ISLAND(me_ptr->iid, DBG_MED_PRIO, "Raised mf from data flow start %ld", raised_mf);
      }

      capi_sal_assign_ref_port(me_ptr, data_port_index);
      me_ptr->in_port_arr[data_port_index].port_flags.data_drop = FALSE; // will be accumulated
      capi_sal_compare_port_mfs_to_omf_and_asign_data_drops(me_ptr);

      // set out port bps and qf from here if we're in native mode and we have an
      // operating mf_ptr
      if (SAL_PARAM_NATIVE == me_ptr->bps_cfg_mode)
      {
         me_ptr->out_port_cache_cfg.q_factor = me_ptr->operating_mf_ptr->format.q_factor;
         me_ptr->out_port_cache_cfg.word_size_bytes =
            me_ptr->operating_mf_ptr->format.bits_per_sample >> 3; // will be 2 or 4
      }
      SAL_MSG_ISLAND(me_ptr->iid, DBG_MED_PRIO, "port_index %lu is the new ref port", data_port_index);
   }
   // else nothing changes

   // figure out if lim needs to be in bypass or not (active siso)
   capi_sal_check_and_update_lim_bypass_mode(me_ptr);

   SAL_MSG_ISLAND(me_ptr->iid,
                  DBG_MED_PRIO,
                  "Port index %lu started, is_ref_port = %u, lim_bypass %lu, dynamic_inplace %lu",
                  data_port_index,
                  me_ptr->in_port_arr[data_port_index].port_flags.is_ref_port,
                  me_ptr->lim_bypass,
                  me_ptr->module_flags.is_inplace);

   capi_sal_print_operating_mf(me_ptr);
   return raised_mf;
}
