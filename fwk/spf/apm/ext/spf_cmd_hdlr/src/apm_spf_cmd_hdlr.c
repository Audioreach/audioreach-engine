/**
 * \file apm_spf_cmd_hdlr.c
 *
 * \brief
 *     This file contains framework internal SPF command handler definitions
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include "apm_proxy_utils.h"
#include "apm_proxy_vcpm_utils.h"
#include "apm_cmd_utils.h"
#include "apm_cmd_sequencer.h"
#include "apm_internal_if.h"
/****************************************************************************
 * Function Definitions
 ****************************************************************************/

ar_result_t apm_cmdq_spf_cmd_handler(apm_t *apm_info_ptr, spf_msg_t *msg_ptr)
{
   ar_result_t      result = AR_EOK;
   apm_ext_utils_t *ext_utils_ptr;
   bool_t           defer_cmd_proc = FALSE;
   bool_t           cmd_def_check_reqd = TRUE;

   AR_MSG(DBG_HIGH_PRIO,
          "apm_cmdq_spf_cmd_handler():"
          " Received command, cmd/event ID: 0x%lx",
          msg_ptr->msg_opcode);

   /** Get the pointer to APM ext utils vtbl ptr  */
   ext_utils_ptr = &apm_info_ptr->ext_utils;

   /** Allocate command handler resources  */
   if (AR_EOK != (result = apm_allocate_cmd_hdlr_resources(apm_info_ptr, msg_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_cmdq_spf_cmd_handler(), Failed to allocate rsc for cmd/msg opcode: 0x%8lX",
             msg_ptr->msg_opcode);

      spf_msg_ack_msg(msg_ptr, result);

      return result;
   }

   switch (msg_ptr->msg_opcode)
   {

      case SPF_MSG_CMD_PROXY_GRAPH_START:
      case SPF_MSG_CMD_PROXY_GRAPH_STOP:
      case SPF_MSG_CMD_PROXY_GRAPH_PREPARE:
      {
         result = apm_proxy_graph_mgmt_cmd_hdlr(apm_info_ptr, msg_ptr, &cmd_def_check_reqd);

         break;
      }
      case SPF_EVT_TO_APM_FOR_PATH_DELAY:
      {
         if (ext_utils_ptr->data_path_vtbl_ptr && ext_utils_ptr->data_path_vtbl_ptr->apm_path_delay_event_hdlr_fptr)
         {
            result = ext_utils_ptr->data_path_vtbl_ptr->apm_path_delay_event_hdlr_fptr(apm_info_ptr, msg_ptr);
         }
         else
         {
            result = AR_ENOTIMPL;
         }

         break;
      }
      case SPF_MSG_CMD_GET_CFG:
      {
         spf_msg_header_t *msg_header_ptr = (spf_msg_header_t *)msg_ptr->payload_ptr;
         if (msg_header_ptr->payload_size < sizeof(spf_msg_cmd_param_data_cfg_t))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "apm_cmdq_spf_cmd_handler(), Not enought payload size for msg opcode: 0x%8lX",
                   msg_ptr->msg_opcode);
            return AR_EFAILED;
         }
         spf_msg_cmd_param_data_cfg_t *param_data_cfg_ptr =
            (spf_msg_cmd_param_data_cfg_t *)&msg_header_ptr->payload_start;
         apm_module_param_data_t **param_data_pptr = (apm_module_param_data_t **)param_data_cfg_ptr->param_data_pptr;

         for (uint32_t i = 0; i < param_data_cfg_ptr->num_param_id_cfg; i++)
         {
            apm_module_param_data_t *param_data_ptr = (apm_module_param_data_t *)param_data_pptr[i];
            switch (param_data_ptr->param_id)
            {
               case APM_PARAM_ID_GET_CNTR_HANDLES:
               case APM_PARAM_ID_GET_ALL_CNTR_HANDLES:
               {
                  if (ext_utils_ptr->db_query_vtbl_ptr &&
                      ext_utils_ptr->db_query_vtbl_ptr->apm_db_query_preprocess_get_param_fptr)
                  {
                     result |=
                        ext_utils_ptr->db_query_vtbl_ptr->apm_db_query_preprocess_get_param_fptr(apm_info_ptr,
                                                                                                   param_data_ptr);
                  }
                  break;
               }
               default:
               {
                  AR_MSG(DBG_ERROR_PRIO,
                         "apm_cmdq_spf_cmd_handler(), Rcvd un-supported param id: 0x%8lX",
                         param_data_ptr->param_id);
                  break;
               }
            }
         }
         break;
      }
      case SPF_MSG_CMD_SET_CFG:
      {
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cmdq_spf_cmd_handler(), Rcvd un-supported cmd/msg opcode: 0x%8lX",
                msg_ptr->msg_opcode);

         result = AR_EUNSUPPORTED;
         break;
      }

   } /** End of switch (msg_ptr->msg_opcode) */

   if (AR_EOK != result)
   {
      /** End the command with failed status   */
      apm_cmd_ctrl_clear_cmd_pending_status(apm_info_ptr->curr_cmd_ctrl_ptr, result);

      apm_end_cmd(apm_info_ptr);

      return result;
   }
   /** Now check if this command processing needs to be
    *  deferred */
   if (cmd_def_check_reqd &&
       apm_info_ptr->ext_utils.parallel_cmd_utils_vtbl_ptr &&
       apm_info_ptr->ext_utils.parallel_cmd_utils_vtbl_ptr->apm_check_and_defer_cmd_processing_fptr)
   {

      if (AR_EOK != (result = apm_info_ptr->ext_utils.parallel_cmd_utils_vtbl_ptr
                                 ->apm_check_and_defer_cmd_processing_fptr(apm_info_ptr, &defer_cmd_proc)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cmdq_spf_cmd_handler(), Deferred cmd proc check failed for cmd_opcode: 0x%8lX defer_cmd_proc[%d] result[%d]",
                msg_ptr->msg_opcode, defer_cmd_proc, result);
         if (result == AR_EBUSY)
         {
            goto __bailout_spf_msg_rsc_alloc;
         }
         /** Set up sequencer to begin with error handling */
         apm_cmd_seq_set_up_err_hdlr(apm_info_ptr->curr_cmd_ctrl_ptr);
      }
   }

   /** Check if the current command processing needs to be
    *  deferred, then return */
   if ((AR_EOK == result) && defer_cmd_proc)
   {
      return AR_EOK;
   }

   /** Execution falls through    */

   /** Now invoke the command sequencer based upon the  current
    *  cmd/msg opcode */

   /** Any failures beyond this point are handled from within
    *  the sequencer. The sequencer also takes care of ending
    *  the GPR command with appropriate error code */

   /** Call the command sequencer, corresponding to current
    *  opcode under process */
   if (AR_EOK != (result = apm_cmd_sequencer_cmn_entry(apm_info_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_cmdq_spf_cmd_handler(): cmd seq failed, cmd_opcode[0x%lX], result[%lu]",
             msg_ptr->msg_opcode,
             result);
   }

   return result;

__bailout_spf_msg_rsc_alloc:

   /** End cmd with failed status   */
   apm_end_cmd(apm_info_ptr);

   return result;
}
