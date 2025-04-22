/**
 * \file apm_sys_util.c
 *
 * \brief
 *
 *     This file contains apm sys utilities.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_sys_util.h"
#include "spf_sys_util.h"
#include "apm_internal.h"
#include "apm_cmd_utils.h"

ar_result_t apm_sys_util_process(apm_t *apm_info_ptr);
ar_result_t apm_sys_util_register(uint32_t num_proc_domain_ids, uint32_t *proc_domain_list);
void * apm_sys_util_get_sys_q_handle();
bool_t apm_sys_util_is_pd_info_available();
/**==============================================================================
   Global Defines
==============================================================================*/
/**< APM sys util handle */
spf_sys_util_handle_t *g_apm_sys_util_handle_ptr;

apm_sys_util_vtable_t g_apm_sys_util_funcs = {.apm_sys_util_process_fptr          = apm_sys_util_process,
                                              .apm_sys_util_register_fptr         = apm_sys_util_register,
                                              .apm_sys_util_get_sys_q_handle_fptr = apm_sys_util_get_sys_q_handle,
                                              .apm_sys_util_is_pd_info_available_fptr =
                                                 apm_sys_util_is_pd_info_available };

static ar_result_t apm_sys_util_handle_gpr_cmd(apm_t *apm_info_ptr, spf_msg_t *msg_ptr)
{
   ar_result_t result = AR_EOK;

   /** Do nothing - no handler implementation for now*/
   return result;
}

static ar_result_t apm_sys_util_handle_ssr_status_up(apm_t *                         apm_info_ptr,
                                                     bool_t                          is_master,
                                                     param_id_sys_util_svc_status_t *payload_ptr)
{
   ar_result_t result = AR_EOK;
   if (is_master)
   {
      /** Case where Master APM gets UP notification of a Satellite APM
          This implies that the Satellite DSP has booted up and it can receive gpr commands.
          We are sending Master proc domain id to satellite apm when this happens */
      if (apm_info_ptr->ext_utils.offload_vtbl_ptr &&
          apm_info_ptr->ext_utils.offload_vtbl_ptr->apm_offload_send_master_pd_info_fptr)
      {
         /* Send command to satellite dsp informing the
          * proc domain id of the master dsp */
         result =
            apm_info_ptr->ext_utils.offload_vtbl_ptr->apm_offload_send_master_pd_info_fptr(apm_info_ptr,
                                                                                           payload_ptr->proc_domain_id);
      }
   }
   else
   {
      /** Case where Satellite APM gets UP notification of the Master APM.
          This implies master has booted up.
          Currently Satelite doesn't have to do anything when the Master DSP boots up
          So, just clean up the resources */
      apm_cmd_ctrl_clear_cmd_pending_status(apm_info_ptr->curr_cmd_ctrl_ptr, result);
      apm_end_cmd(apm_info_ptr);
   }
   return result;
}

static ar_result_t apm_sys_util_handle_ssr_status_down(apm_t *                         apm_info_ptr,
                                                       bool_t                          is_master,
                                                       param_id_sys_util_svc_status_t *payload_ptr)
{
   ar_result_t result = AR_EOK;
   if (is_master)
   {
      /** Case where Master APM gets DOWN notification of a Satellite APM
          This implies that the Satellite DSP has crashed.
          Clean up bookeeping */
      result = apm_info_ptr->ext_utils.offload_vtbl_ptr->apm_offload_sat_cleanup_fptr(payload_ptr->proc_domain_id);
      apm_cmd_ctrl_clear_cmd_pending_status(apm_info_ptr->curr_cmd_ctrl_ptr, result);
      apm_end_cmd(apm_info_ptr);
   }
   else
   {
      /** Case where Satellite APM gets DOWN notification of the Master APM.
          This implies master has crashed.
          Satellite needs to clean up everything and go to boot up state
          So, do a close-all */

      /** Enable this flag so set-get sequencer also does a close all */
      apm_info_ptr->curr_cmd_ctrl_ptr->set_cfg_cmd_ctrl.is_close_all_needed = TRUE;

      /** Call the command sequencer, corresponding to current
       *  opcode under process */
      if (AR_EOK != (result = apm_cmd_sequencer_cmn_entry(apm_info_ptr)))
      {
         AR_MSG(DBG_ERROR_PRIO, "CMD seq failed for handling ssr status down, result[%lu]", result);
      }
   }
   return result;
}

static ar_result_t apm_sys_util_handle_svc_status(apm_t *apm_info_ptr, param_id_sys_util_svc_status_t *payload_ptr)
{
   ar_result_t result    = AR_EOK;
   bool_t      is_master = TRUE;

   if (apm_info_ptr->ext_utils.offload_vtbl_ptr &&
       apm_info_ptr->ext_utils.offload_vtbl_ptr->apm_offload_is_master_pid_fptr)
   {
      is_master = apm_info_ptr->ext_utils.offload_vtbl_ptr->apm_offload_is_master_pid_fptr();
   }

   AR_MSG(DBG_MED_PRIO,
          "APM: Handling ssr notification, status = %lu, is_master = %lu",
          payload_ptr->status,
          is_master);

   /** Handle the UP/DOWN notification based on whether the APM is running in Master or Satellite DSP*/
   switch (payload_ptr->status)
   {
      case SPF_SYS_UTIL_SSR_STATUS_UP:
      {
         result = apm_sys_util_handle_ssr_status_up(apm_info_ptr, is_master, payload_ptr);
         break;
      }
      case SPF_SYS_UTIL_SSR_STATUS_DOWN:
      {
         result = apm_sys_util_handle_ssr_status_down(apm_info_ptr, is_master, payload_ptr);
         break;
      }
      default:
      {
         /** Free up resources if it is neither service up or down notificaion */
         apm_cmd_ctrl_clear_cmd_pending_status(apm_info_ptr->curr_cmd_ctrl_ptr, result);
         apm_end_cmd(apm_info_ptr);
         break;
      }
   }

   return result;
}

static ar_result_t apm_sys_util_set_handler(apm_t *apm_info_ptr, spf_msg_t *msg_ptr)
{
   ar_result_t                   result             = AR_EOK;
   spf_msg_header_t *            msg_header_ptr     = (spf_msg_header_t *)msg_ptr->payload_ptr;
   spf_msg_cmd_param_data_cfg_t *param_data_cfg_ptr = NULL;
   apm_module_param_data_t **    param_data_pptr    = NULL;

   /** Payload size sanity check */
   if (msg_header_ptr->payload_size < sizeof(spf_msg_cmd_param_data_cfg_t))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "APM: Insufficient paylaod size %lu, required",
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
            result |= apm_sys_util_handle_svc_status(apm_info_ptr, payload_ptr);
            break;
         }
         default:
         {
            AR_MSG(DBG_ERROR_PRIO, "APM: Unsupported param id 0x%X", param_data_ptr->param_id);
         }
      }
   }

   return result;
}

static ar_result_t apm_sys_util_handle_spf_cmd(apm_t *apm_info_ptr, spf_msg_t *msg_ptr)
{
   ar_result_t result = AR_EOK;

   /** Sanily Check */
   if (NULL == msg_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM: NULL msg ptr");
      return AR_EBADPARAM;
   }

   /** Allocate command handler resources  */
   if (AR_EOK != (result = apm_allocate_cmd_hdlr_resources(apm_info_ptr, msg_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_cmdq_spf_cmd_handler(), Failed to allocate rsc for cmd/msg opcode: 0x%8lX",
             msg_ptr->msg_opcode);

      goto __bailout_sys_util_rsc_alloc;
   }

   AR_MSG(DBG_HIGH_PRIO, "APM: Handling sys command, opcode 0x%X", msg_ptr->msg_opcode);

   switch (msg_ptr->msg_opcode)
   {
      case SPF_MSG_CMD_SET_CFG:
      {
         result = apm_sys_util_set_handler(apm_info_ptr, msg_ptr);
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO, "APM: Unsupported opcode 0x%X", msg_ptr->msg_opcode);
         break;
      }
   }

   if (AR_EOK != result)
   {
      /** End the command with failed status   */
      apm_cmd_ctrl_clear_cmd_pending_status(apm_info_ptr->curr_cmd_ctrl_ptr, result);

      apm_end_cmd(apm_info_ptr);

      return result;
   }

   return result;

__bailout_sys_util_rsc_alloc:

   /** End GK message with failed status   */
   spf_msg_ack_msg(msg_ptr, result);

   return result;
}

bool_t apm_sys_util_is_pd_info_available()
{
   spf_sys_util_handle_t *handle_ptr = g_apm_sys_util_handle_ptr;

   AR_MSG(DBG_HIGH_PRIO, "APM: SYS_UTIL: num_proc_domain_ids: %lu", handle_ptr->num_proc_domain_ids);

   AR_MSG(DBG_HIGH_PRIO,
          "APM: SYS_UTIL: is pd info available: %lu (0 = no, 1 = yes)",
          (0 != handle_ptr->num_proc_domain_ids));

   return (0 != handle_ptr->num_proc_domain_ids);
}

void *apm_sys_util_get_sys_q_handle()
{
   spf_sys_util_handle_t *handle_ptr = g_apm_sys_util_handle_ptr;

   /** Return the sys que ptr of it exists else return NULL */
   if (handle_ptr && handle_ptr->sys_queue_ptr)
   {
      return handle_ptr->sys_queue_ptr;
   }
   return NULL;
}

ar_result_t apm_sys_util_register(uint32_t num_proc_domain_ids, uint32_t *proc_domain_list)
{
   ar_result_t            result     = AR_EOK;
   spf_sys_util_handle_t *handle_ptr = g_apm_sys_util_handle_ptr;

   /** Sanity Check */
   if (NULL == handle_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM: SYS_UTIL: Sys util handle is NULL");
      result = AR_EFAILED;
      return result;
   }

   /** Register for SSR notification with given proc domain id list */
   result = spf_sys_util_ssr_register(handle_ptr, num_proc_domain_ids, proc_domain_list);
   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO, "APM: SYS_UTIL: Failed to register for sev reg, result: 0x%lx", result);
   }

#if APM_SSR_TEST_CODE
   // TODO:pbm+wchaffin - remove after testing
   uint32_t host_domain_id = 0;
   __gpr_cmd_get_host_domain_id(&host_domain_id);
   if ((2 == host_domain_id))
   {
      if (0 != num_proc_domain_ids)
      {
         AR_MSG(DBG_ERROR_PRIO, "APM: SYS_UTIL: WCHAFFIN: Pushing test cb to sys queue");
         spf_svc_test_cb(g_apm_sys_util_handle_ptr, TRUE);
      }
   }
   else // wchaffin: don't need to worry about this part
   {
      result |= spf_sys_util_ssr_deregister(handle_ptr);
      spf_svc_test_cb(g_apm_sys_util_handle_ptr, FALSE); // Try and make it crash as master w/o deregister
   }
#endif
   return result;
}

ar_result_t apm_sys_util_process(apm_t *apm_info_ptr)
{
   ar_result_t result       = AR_EOK;
   uint32_t    sys_q_status = 0;

   /** poll the channel for sys q wait mask */
   while ((sys_q_status = posal_channel_poll(apm_info_ptr->channel_ptr, g_apm_sys_util_handle_ptr->sys_queue_mask)))
   {
      spf_msg_t              msg;
      spf_sys_util_handle_t *handle_ptr = g_apm_sys_util_handle_ptr;

      /** Pop the message buffer from the system cmd queue */
      if (AR_EOK != (result = posal_queue_pop_front(handle_ptr->sys_queue_ptr, (posal_queue_element_t *)&msg)))
      {
         AR_MSG(DBG_ERROR_PRIO, "APM: SYS_UTIL: Failed to pop buf from svc cmd_q, result: 0x%lx", result);
         return result;
      }

      /** call gpr or spf-cmd handlers based on type of the msg*/
      if (SPF_MSG_CMD_GPR == msg.msg_opcode)
      {
         result |= apm_sys_util_handle_gpr_cmd(apm_info_ptr, &msg);
      }
      else
      {
         result |= apm_sys_util_handle_spf_cmd(apm_info_ptr, &msg);
      }
   }
   return result;
}

ar_result_t apm_sys_util_init(apm_t *apm_info_ptr)
{
   ar_result_t result        = AR_EOK;
   g_apm_sys_util_handle_ptr = NULL;

   /** Initialize the vtable */
   apm_info_ptr->ext_utils.sys_util_vtbl_ptr = &g_apm_sys_util_funcs;

   /** Get the sys util handle with don't-care bit mask, get proper bitmask and store */
   result = spf_sys_util_get_handle(&g_apm_sys_util_handle_ptr,
                                    NULL,
                                    apm_info_ptr->channel_ptr,
                                    "apm_sys_q",
                                    POSAL_CHANNEL_MASK_DONT_CARE,
                                    APM_INTERNAL_STATIC_HEAP_ID,
                                    APM_MAX_SYS_Q_ELEMENTS,
                                    APM_PREALLOC_SYS_Q_ELEMENTS);
   if ((AR_EOK != result) || (NULL == g_apm_sys_util_handle_ptr))
   {
      AR_MSG(DBG_ERROR_PRIO, "APM: SYS_UTIL: Failed to get the sis util handle, result: 0x%lx", result);
      return result;
   }

   /** Update the sys q bit mask in the handle*/
   g_apm_sys_util_handle_ptr->sys_queue_mask = posal_queue_get_channel_bit(g_apm_sys_util_handle_ptr->sys_queue_ptr);

   /** Store the bit mask handle in the apm wait mask, so the workloop can wait on it */
   apm_info_ptr->curr_wait_mask |= g_apm_sys_util_handle_ptr->sys_queue_mask;
   return result;
}

ar_result_t apm_sys_util_deinit()
{
   ar_result_t result = AR_EOK;

   /** release the sys util handle */
   if (NULL != g_apm_sys_util_handle_ptr)
   {
      result = spf_sys_util_release_handle(&g_apm_sys_util_handle_ptr);
   }

   return result;
}
