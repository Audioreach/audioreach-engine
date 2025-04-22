/**
 * \file olc_cmd_handler.c
 * \brief
 *     This file contains olc functions for command handling.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "olc_driver.h"
#include "olc_i.h"
#include "apm.h"

// Maximum number of commands expected ever in command queue.
static const uint32_t OLC_MAX_SYS_Q_ELEMENTS = 8;

// clang-format off

// function table for error response handling.
// Shared with SGM driver to call when satellite down notification is received to acknowledge the pending commands.
static sgmc_rsp_h_vtable_t sgm_servreg_error_rsp_hp =
{
   .graph_open_rsp_h                  = olc_graph_open_error_rsp_h,
   .graph_prepare_rsp_h               = olc_graph_prepare_rsp_h,
   .graph_start_rsp_h                 = olc_graph_start_rsp_h,
   .graph_suspend_rsp_h               = olc_graph_suspend_rsp_h,
   .graph_stop_rsp_h                  = olc_graph_stop_rsp_h,
   .graph_flush_rsp_h                 = olc_graph_flush_rsp_h,
   .graph_close_rsp_h                 = olc_graph_close_rsp_h,
   .graph_set_get_cfg_rsp_h           = olc_graph_set_get_cfg_rsp_h,
   .graph_set_get_cfg_packed_rsp_h    = olc_graph_set_get_packed_cfg_rsp_h,
   .graph_set_persistent_rsp_h        = olc_graph_set_persistent_cfg_rsp_h, //gpr cmd
   .graph_set_persistent_packed_rsp_h = olc_graph_set_persistent_packed_rsp_h, //gkmsg from apm
   .graph_event_reg_rsp_h             = olc_graph_event_reg_rsp_h
};

// clang-format on

/* =======================================================================
Static Function Definitions
========================================================================== */

static ar_result_t olc_serv_reg_spf_pending_cmd_resp_hndlr_init(olc_t *me_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   uint32_t log_id = me_ptr->topo.gu.log_id;

   me_ptr->satellite_up_down_status = SPF_SYS_UTIL_SSR_STATUS_UP;
   TRY(result, sgm_set_servreg_error_notify_cmd_rsp_fn_handler(&me_ptr->spgm_info, (void *)&sgm_servreg_error_rsp_hp));

   OLC_MSG(log_id, DBG_HIGH_PRIO, "OLC SSR: serv_reg_spf_pending_cmd_resp_hndlr_init done");

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
      OLC_MSG(log_id, DBG_ERROR_PRIO, "OLC SSR: serv_reg_spf_pending_cmd_resp_hndlr_init failed, result %lu", result);
   }

   return result;
}

/**
 * Handling of the ssr pdr signal handler
 */
static ar_result_t olc_servreg_notify_satellite_down_event_handler(cu_base_t *base_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   olc_t *  me_ptr = (olc_t *)base_ptr;
   uint32_t log_id = me_ptr->topo.gu.log_id;

   OLC_MSG(log_id,
           DBG_HIGH_PRIO,
           "OLC SSR:  start executing satellite down event handler. current channel mask=0x%x",
           me_ptr->cu.curr_chan_mask);

   // Things to do
   // 0.  add a system Q with higher priority
   // 1.  respond to the pending commands. (open/close would need additional handling)
   // 2.  stop listening to response, event queue, data queues.
   // 3.  maintain a state variable to indicate that the satellite is down.  /// depends on the payload. //todo
   // 4.  other commands that can come after this state would be close.
   // 5. Should we wait for the close to come or do it part of the down notification.

   me_ptr->cu.curr_chan_mask = OLC_SYSTEM_Q_BIT_MASK | OLC_CMD_BIT_MASK;
   TRY(result, sgm_servreg_notify_event_handler(base_ptr, &me_ptr->spgm_info));

   OLC_MSG(log_id,
           DBG_HIGH_PRIO,
           "OLC SSR: Done executing satellite down event handler"
           "current channel mask=0x%x. result=0x%lx.",
           base_ptr->curr_chan_mask,
           result);

   CATCH(result, OLC_MSG_PREFIX, log_id)
   {
   }
   return result;
}

/**
 * Handling of the control path graph open command error response when satellite is down.
 */
ar_result_t olc_graph_open_error_rsp_h(cu_base_t *base_ptr, spgm_cmd_rsp_node_t *rsp_info)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   olc_t *                   me_ptr       = (olc_t *)base_ptr;
   spf_msg_cmd_graph_open_t *open_cmd_ptr = NULL;

   OLC_MSG(me_ptr->topo.gu.log_id,
           DBG_HIGH_PRIO,
           "CMD:GRAPH_OPEN: Executing graph open command error response."
           " current channel mask=0x%x",
           me_ptr->cu.curr_chan_mask);

   VERIFY(result, (AR_EOK == rsp_info->rsp_result));

   me_ptr->cu.cmd_msg = *rsp_info->cmd_msg;

   spf_msg_header_t *header_ptr = (spf_msg_header_t *)rsp_info->cmd_msg->payload_ptr;
   VERIFY(result, header_ptr->payload_size >= sizeof(spf_msg_cmd_graph_open_t));

   open_cmd_ptr = (spf_msg_cmd_graph_open_t *)&header_ptr->payload_start;

   olc_handle_failure_at_graph_open(me_ptr, open_cmd_ptr, result, TRUE);

   CATCH(result, OLC_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   me_ptr->cu.curr_chan_mask |= (OLC_CMD_BIT_MASK);
   OLC_MSG(me_ptr->topo.gu.log_id,
           DBG_HIGH_PRIO,
           "CMD:GRAPH_OPEN: Done executing graph open command. current channel mask=0x%x. result=0x%lx.",
           me_ptr->cu.curr_chan_mask,
           result);

   return result;
}

static ar_result_t olc_sys_util_handle_gpr_cmd(olc_t *me_ptr, spf_msg_t *msg_ptr)
{
   ar_result_t result = AR_EOK;

   /** Do nothing - no handler implementation for now*/
   return result;
}

static ar_result_t olc_sys_util_handle_svc_status(olc_t *me_ptr, param_id_sys_util_svc_status_t *payload_ptr)
{
   ar_result_t result = AR_EOK;
   uint32_t    log_id = me_ptr->topo.gu.log_id;

   OLC_MSG(log_id, DBG_MED_PRIO, "OLC: Handling ssr notification, status = %lu", payload_ptr->status);

   /** Handle the UP/DOWN notification based on whether the APM is running in Master or Satellite DSP*/
   switch (payload_ptr->status)
   {
      case SPF_SYS_UTIL_SSR_STATUS_UP:
      {
         me_ptr->satellite_up_down_status = SPF_SYS_UTIL_SSR_STATUS_UP;
         break;
      }
      case SPF_SYS_UTIL_SSR_STATUS_DOWN:
      {
         olc_servreg_notify_satellite_down_event_handler(&me_ptr->cu);
         me_ptr->satellite_up_down_status = SPF_SYS_UTIL_SSR_STATUS_DOWN;
         break;
      }
      default:
      {
         break;
      }
   }

   return result;
}

static ar_result_t olc_sys_util_set_handler(olc_t *me_ptr, spf_msg_t *msg_ptr)
{
   ar_result_t                   result             = AR_EOK;
   spf_msg_header_t *            msg_header_ptr     = (spf_msg_header_t *)msg_ptr->payload_ptr;
   spf_msg_cmd_param_data_cfg_t *param_data_cfg_ptr = NULL;
   apm_module_param_data_t **    param_data_pptr    = NULL;
   uint32_t                      log_id             = me_ptr->topo.gu.log_id;

   /** Payload size sanity check */
   if (msg_header_ptr->payload_size < sizeof(spf_msg_cmd_param_data_cfg_t))
   {
      OLC_MSG(log_id,
              DBG_ERROR_PRIO,
              "OLC: Insufficient paylaod size %lu, required %lu",
              msg_header_ptr->payload_size,
              sizeof(spf_msg_cmd_param_data_cfg_t));
      return AR_EBADPARAM;
   }

   param_data_cfg_ptr = (spf_msg_cmd_param_data_cfg_t *)&msg_header_ptr->payload_start;
   param_data_pptr    = (apm_module_param_data_t **)param_data_cfg_ptr->param_data_pptr;

   for (uint32_t i = 0; i < param_data_cfg_ptr->num_param_id_cfg; i++)
   {
      apm_module_param_data_t *param_data_ptr = (apm_module_param_data_t *)param_data_pptr[i];

      switch (param_data_ptr->param_id)
      {
         case PARAM_ID_SYS_UTIL_SVC_STATUS:
         {
            param_id_sys_util_svc_status_t *payload_ptr = (param_id_sys_util_svc_status_t *)(param_data_ptr + 1);
            result |= olc_sys_util_handle_svc_status(me_ptr, payload_ptr);
            break;
         }
         default:
         {
            OLC_MSG(log_id, DBG_ERROR_PRIO, "OLC: Unsupported param id 0x%X", param_data_ptr->param_id);
         }
      }
   }

   return result;
}

static ar_result_t olc_sys_util_handle_spf_cmd(olc_t *me_ptr, spf_msg_t *msg_ptr)
{
   ar_result_t       result         = AR_EOK;
   uint32_t          log_id         = me_ptr->topo.gu.log_id;
   spf_msg_header_t *msg_header_ptr = NULL;

   /** Sanity Check */
   if (NULL == msg_ptr)
   {
      OLC_MSG(log_id, DBG_ERROR_PRIO, "OLC: NULL msg ptr");
      return AR_EBADPARAM;
   }
   msg_header_ptr = (spf_msg_header_t *)msg_ptr->payload_ptr;
   if (msg_header_ptr->payload_size < sizeof(spf_msg_cmd_param_data_cfg_t))
   {
      spf_msg_ack_msg(msg_ptr, AR_EBADPARAM);
      return AR_EBADPARAM;
   }

   OLC_MSG(log_id, DBG_HIGH_PRIO, "OLC SSR: Handling command, opcode 0x%X", msg_ptr->msg_opcode);

   switch (msg_ptr->msg_opcode)
   {
      case SPF_MSG_CMD_SET_CFG:
      {
         result = olc_sys_util_set_handler(me_ptr, msg_ptr);
         break;
      }
      default:
      {
         OLC_MSG(log_id, DBG_HIGH_PRIO, "OLC SSR: Unsupported CMD recieved 0x%lx", msg_ptr->msg_opcode);
         result = AR_EBADPARAM;
         break;
      }
   }

   /** End GK message with failed status   */
   spf_msg_ack_msg(msg_ptr, result);

   return result;
}

ar_result_t olc_serv_reg_notify_process(cu_base_t *base_ptr, uint32_t ch_bit_index)
{
   ar_result_t            result       = AR_EOK;
   uint32_t               sys_q_status = 0;
   olc_t *                me_ptr       = (olc_t *)base_ptr;
   spf_sys_util_handle_t *handle_ptr   = me_ptr->serv_reg_handle.sys_util_ptr;
   uint32_t               log_id       = me_ptr->topo.gu.log_id;
   spf_msg_t              msg;

   /** poll the channel for sys q wait mask */
   while ((sys_q_status = posal_channel_poll(me_ptr->cu.channel_ptr, me_ptr->serv_reg_handle.sys_util_ptr->sys_queue_mask)))
   {
      /** Pop the message buffer from the system cmd queue */
      if (AR_EOK != (result = posal_queue_pop_front(handle_ptr->sys_queue_ptr, (posal_queue_element_t *)&msg)))
      {
         OLC_MSG(log_id, DBG_HIGH_PRIO, "OLC SSR: Failed to pop buf from system cmd_q, result: 0x%lx", result);
         return result;
      }

      /** call gpr or spf-cmd handlers based on type of the msg*/
      if (SPF_MSG_CMD_GPR == msg.msg_opcode)
      {
         result |= olc_sys_util_handle_gpr_cmd(me_ptr, &msg);
      }
      else
      {
         result |= olc_sys_util_handle_spf_cmd(me_ptr, &msg);
      }
   }
   return result;
}

ar_result_t olc_serv_reg_notify_register(olc_t *me_ptr, uint32_t satellite_proc_domain_id)
{
   ar_result_t            result              = AR_EOK;
   spf_sys_util_handle_t *handle_ptr          = me_ptr->serv_reg_handle.sys_util_ptr;
   uint32_t               log_id              = me_ptr->topo.gu.log_id;
   uint32_t               num_proc_domain_ids = 0;

   /** Sanity Check */
   if (NULL == handle_ptr)
   {
      OLC_MSG(log_id, DBG_HIGH_PRIO, "OLC SSR: Sys util handle is NULL");
      result = AR_EFAILED;
      return result;
   }

   num_proc_domain_ids = 1;
   /** Register for SSR notification with given proc domain id list */
   result = spf_sys_util_ssr_register(handle_ptr, num_proc_domain_ids, &satellite_proc_domain_id);
   if (AR_EOK != result)
   {
      OLC_MSG(log_id,
              DBG_HIGH_PRIO,
              "OLC SSR: Failed to register for satellite serv reg notification, result: 0x%lx",
              result);
   }

   return result;
}

static ar_result_t olc_sys_util_init(olc_t *me_ptr, uint32_t log_id)
{
   ar_result_t result = AR_EOK;

   // Queue names.
   char_t sysQ_name[POSAL_DEFAULT_NAME_LEN];

   snprintf(sysQ_name, POSAL_DEFAULT_NAME_LEN, "%s%8lX", "SOLC", me_ptr->topo.gu.log_id);

   /** Get the sys util handle with don't-care bit mask, get proper bitmask and store */
   me_ptr->serv_reg_handle.sys_util_ptr = NULL;

   result = spf_sys_util_get_handle(&me_ptr->serv_reg_handle.sys_util_ptr,
                                    NULL,
                                    me_ptr->cu.channel_ptr,
                                    sysQ_name,
                                    OLC_SYSTEM_Q_BIT_MASK,
                                    me_ptr->cu.heap_id,
                                    OLC_MAX_SYS_Q_ELEMENTS,
                                    OLC_MAX_SYS_Q_ELEMENTS);

   if ((AR_EOK != result))
   {
      OLC_MSG(log_id, DBG_HIGH_PRIO, "OLC SSR: Failed to get the sys util handle, result: 0x%lx", result);
      return result;
   }

   return result;
}

ar_result_t olc_serv_reg_notify_init(olc_t *me_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   // create the sys_util handler which internally creates system Queue.
   TRY(result, olc_sys_util_init(me_ptr, me_ptr->topo.gu.log_id));

   // Clear masks from available bitmasks.
   me_ptr->cu.available_bit_mask &= (~(OLC_SYSTEM_Q_BIT_MASK));

   cu_set_handler_for_bit_mask(&me_ptr->cu, OLC_SYSTEM_Q_BIT_MASK, olc_serv_reg_notify_process);

   TRY(result, olc_serv_reg_spf_pending_cmd_resp_hndlr_init(me_ptr));

   OLC_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "OLC SSR: serv_reg_notify_init done", result);

   CATCH(result, OLC_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
      OLC_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "OLC SSR: serv_reg_notify_init failed result = %d", result);
   }

   return result;
}

ar_result_t olc_serv_reg_notify_deinit(olc_t *me_ptr)
{
   ar_result_t result = AR_EOK;

   /** release the sys util handle */
   if (NULL != me_ptr->serv_reg_handle.sys_util_ptr)
   {
      result                      = spf_sys_util_release_handle(&me_ptr->serv_reg_handle.sys_util_ptr);
      me_ptr->serv_reg_handle.sys_util_ptr = NULL;
   }

   return result;
}
