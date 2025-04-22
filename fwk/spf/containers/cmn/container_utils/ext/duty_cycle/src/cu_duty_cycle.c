/**
 * \file cu_duty_cycle.c
 * \brief
 *     This file contains container common Duty Cycling manager functions.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "platform_internal_dcm_if.h"
#include "cu_i.h"

/* =======================================================================
Public Functions
========================================================================== */

// Duty cycling containers registering with Duty Cycling Manager
ar_result_t cu_register_with_dcm(cu_base_t *me_ptr)
{
   dcm_payload_t payload;

   snprintf(payload.client_info.client_name, DCM_CLIENT_NAME_LENGTH, "%2lx", me_ptr->gu_ptr->log_id);
   payload.client_info.client_log_id       = me_ptr->gu_ptr->container_instance_id;
   payload.signal_ptr                      = NULL;
   payload.client_info.spf_handle          = me_ptr->spf_handle;
   me_ptr->pm_info.flags.register_with_dcm = TRUE;
   me_ptr->cntr_vtbl_ptr->dcm_topo_set_param((void *)me_ptr);
   ar_result_t result = posal_power_mgr_send_command(DCM_CMD_REGISTER, &payload, sizeof(dcm_payload_t));
#ifdef DCM_DEBUG
   CU_MSG(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "Registered with DCM ");
#endif

   return result;
}

ar_result_t cu_deregister_with_dcm(cu_base_t *me_ptr)
{
   dcm_payload_t payload;

   snprintf(payload.client_info.client_name, DCM_CLIENT_NAME_LENGTH, "%2lx", me_ptr->gu_ptr->log_id);
   payload.client_info.client_log_id       = me_ptr->gu_ptr->container_instance_id;
   payload.signal_ptr                      = NULL;
   payload.client_info.spf_handle          = me_ptr->spf_handle;
   me_ptr->pm_info.flags.register_with_dcm = FALSE;
   ar_result_t result = posal_power_mgr_send_command(DCM_CMD_DEREGISTER, &payload, sizeof(dcm_payload_t));
#ifdef DCM_DEBUG
   CU_MSG(me_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "De-Registered with DCM ");
#endif

   return result;
}

// Duty cycling containers sending Island Entry command response to Duty Cycling Manager
ar_result_t cu_send_island_entry_ack_to_dcm(cu_base_t *me_ptr)
{
   dcm_island_entry_exit_ack_t payload;
   ar_result_t                 result = AR_EOK;

   memset(&payload, 0, sizeof(dcm_island_entry_exit_ack_t));
   snprintf(payload.client_info.client_name, DCM_CLIENT_NAME_LENGTH, "%2lx", me_ptr->gu_ptr->log_id);
   payload.client_info.client_log_id = me_ptr->gu_ptr->container_instance_id;
   if (!me_ptr->pm_info.flags.module_disallows_duty_cycling)
   {
      payload.ack_result = ISLAND_ENTRY_ALLOWED;
      result             = posal_power_mgr_send_command(DCM_ACK_FROM_CLIENT_FOR_ISLAND_ENTRY,
                                            &payload,
                                            sizeof(dcm_island_entry_exit_ack_t));
   }
   else
   {
      payload.ack_result = ISLAND_ENTRY_BLOCKED_GAPLESS;
   }

   return result;
}

static ar_result_t cu_dcm_initiate_island_entry(cu_base_t *base_ptr,
                                                int8_t *   param_payload_ptr,
                                                uint32_t * param_size_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   cntr_param_id_unblock_island_entry_for_duty_cycling_t *cmd_ptr = NULL;
   VERIFY(result, (NULL != base_ptr->cntr_vtbl_ptr));
#ifdef DCM_DEBUG
   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "CMD:UNBLOCK_ISLAND_ENTRY_FOR_DUTY_CYCLING: Executing island entry cmd");
#endif
   VERIFY(result, *param_size_ptr >= sizeof(cntr_param_id_unblock_island_entry_for_duty_cycling_t));
   cmd_ptr = (cntr_param_id_unblock_island_entry_for_duty_cycling_t *)param_payload_ptr;
   VERIFY(result, (NULL != cmd_ptr));

   if (cmd_ptr->enable)
   {
      CU_SET_ONE_FWK_EVENT_FLAG(base_ptr, need_to_handle_dcm_req_for_unblock_island_entry);
   }

   result |= base_ptr->cntr_vtbl_ptr->initiate_duty_cycle_island_entry(base_ptr);

   CATCH(result, CU_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }
#ifdef DCM_DEBUG
   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "CMD:UNBLOCK_ISLAND_ENTRY_FOR_DUTY_CYCLING: Done Executing island entry cmd");
#endif
   return result;
}

static ar_result_t cu_dcm_initiate_island_exit(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   cntr_param_id_dcm_req_for_island_exit_t *cmd_ptr = NULL;
   VERIFY(result, (NULL != base_ptr->cntr_vtbl_ptr));
#ifdef DCM_DEBUG
   CU_MSG(base_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "CMD:DCM_REQ_FOR_ISLAND_EXIT: Executing island exit cmd");
#endif

   VERIFY(result, *param_size_ptr >= sizeof(cntr_param_id_dcm_req_for_island_exit_t));
   cmd_ptr = (cntr_param_id_dcm_req_for_island_exit_t *)param_payload_ptr;
   VERIFY(result, (NULL != cmd_ptr));

   if (cmd_ptr->enable)
   {
      CU_SET_ONE_FWK_EVENT_FLAG(base_ptr, need_to_handle_dcm_req_for_island_exit);
   }

   result |= base_ptr->cntr_vtbl_ptr->initiate_duty_cycle_island_exit(base_ptr);
   CATCH(result, CU_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }
#ifdef DCM_DEBUG
   CU_MSG(base_ptr->gu_ptr->log_id, DBG_HIGH_PRIO, "CMD:DCM_REQ_FOR_ISLAND_EXIT: Done Executing island exit cmd");
#endif

   return result;
}

ar_result_t cu_dcm_island_entry_exit_handler(cu_base_t *base_ptr,
                                             int8_t *   param_payload_ptr,
                                             uint32_t * param_size_ptr,
                                             uint32_t   pid)
{
   ar_result_t result = AR_EOK;
   switch (pid)
   {
      case CNTR_PARAM_ID_UNBLOCK_ISLAND_ENTRY_FOR_DUTY_CYCLING:
      {

         result = cu_dcm_initiate_island_entry(base_ptr, param_payload_ptr, param_size_ptr);
         break;
      }
      case CNTR_PARAM_ID_DCM_REQ_FOR_ISLAND_EXIT:
      {

         result = cu_dcm_initiate_island_exit(base_ptr, param_payload_ptr, param_size_ptr);
         break;
      }
      default:
      {
         result = AR_EUNEXPECTED;
         break;
      }
   }
   return result;
}
