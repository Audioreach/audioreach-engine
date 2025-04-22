/**
 * \file apm_close_all_utils.c
 *
 * \brief
 *
 *     This file contains APM_CMD_CLOSE_ALL processing utilities.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "amdb_static.h"
#include "apm_cmd_sequencer.h"
#include "apm_proxy_utils.h"
#include "apm_proxy_vcpm_utils.h"
#include "irm.h"
#include "core_drv.h"

/**==============================================================================
   Function Declaration
==============================================================================*/

ar_result_t apm_cmd_close_all_sequencer(apm_t *apm_info_ptr);
ar_result_t apm_cmd_set_cfg_close_all_seq(apm_t *apm_info_ptr);
/**==============================================================================
   Global Defines
==============================================================================*/

apm_close_all_utils_vtable_t close_all_util_funcs = {.apm_cmd_close_all_seqncer_fptr = apm_cmd_close_all_sequencer,
                                                     .apm_cmd_set_cfg_close_all_seq_fptr =
                                                        apm_cmd_set_cfg_close_all_seq };

ar_result_t apm_cmd_set_cfg_close_all_seq(apm_t *apm_info_ptr)
{
   ar_result_t            result               = AR_EOK;
   apm_cont_msg_opcode_t *cont_msg_opcode_ptr  = NULL;
   apm_cmd_ctrl_t *       apm_cmd_ctrl_ptr     = NULL;
   apm_sub_graph_state_t  sub_graph_list_state = APM_SG_STATE_INVALID;

   /** Get the pointer to current command control   */
   apm_cmd_ctrl_ptr     = apm_info_ptr->curr_cmd_ctrl_ptr;
   cont_msg_opcode_ptr  = &apm_cmd_ctrl_ptr->cont_msg_opcode;
   sub_graph_list_state = apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_proc_info.curr_sg_list_state;

   switch (apm_cmd_ctrl_ptr->cmd_seq.close_all_seq.op_idx)
   {
      case APM_CLOSE_ALL_CMD_OP_CLOSE_SG:
      {
         result = apm_populate_graph_close_proc_seq(apm_cmd_ctrl_ptr, sub_graph_list_state, cont_msg_opcode_ptr);
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_populate_spf_set_cfg_cmd_seq(): Unexpected op_idx[0x%lX], cmd_opcode[0x%08lX]",
                apm_cmd_ctrl_ptr->cmd_seq.close_all_seq.op_idx,
                apm_cmd_ctrl_ptr->cmd_opcode);
         break;
      }
   }
   return result;
}

/* Completes APM_CMD_CLOSE_ALL by deregistering and unloading AMDB and unmapping memory */
static ar_result_t apm_finish_graph_close_all_cmd(apm_t *apm_info_ptr)
{
   ar_result_t result                = AR_EOK;
   bool_t      IS_FLUSH_NEEDED_FALSE = FALSE;
   bool_t      IS_RESET_NEEDED_TRUE  = TRUE;

   /** IRM reset */
   if (AR_DID_FAIL(result = irm_reset(IS_FLUSH_NEEDED_FALSE, IS_RESET_NEEDED_TRUE)))
   {
      AR_MSG(DBG_ERROR_PRIO, "APM_CMD_CLOSE_ALL: Unable to reset IRM with result: %lu", result);
      return result;
   }

   /** AMDB unload and reset */
   if (AR_DID_FAIL(result = amdb_reset(IS_FLUSH_NEEDED_FALSE, IS_RESET_NEEDED_TRUE)))
   {
      AR_MSG(DBG_ERROR_PRIO, "APM_CMD_CLOSE_ALL: Unable to reset amdb with result: %lu", result);
      return result;
   }

   // core_drv_reset(): to call asps_reset() as asps is moved to platform/modules/
   if (AR_DID_FAIL(result = core_drv_reset()))
   {
      AR_MSG(DBG_ERROR_PRIO, "APM_CMD_CLOSE_ALL: Unable to reset ASPS with result: %lu", result);
      return result;
   }

   // Reset the offload memory manager
   if (apm_info_ptr->ext_utils.offload_vtbl_ptr &&
       apm_info_ptr->ext_utils.offload_vtbl_ptr->apm_offload_mem_mgr_reset_fptr)
   {
      result = apm_info_ptr->ext_utils.offload_vtbl_ptr->apm_offload_mem_mgr_reset_fptr();
   }

   /** Memory unmap */
   posal_memorymap_global_unmap_all();

   AR_MSG(DBG_HIGH_PRIO, "APM_CMD_CLOSE_ALL completed");

   return result;
}

/* Create list of subgraph ids to be closed */
static ar_result_t apm_prepare_cmd_close_all_payload(apm_t *apm_info_ptr)
{

   ar_result_t      result = AR_EOK;
   apm_cmd_ctrl_t * cmd_ctrl_ptr;
   apm_sub_graph_t *sg_obj_ptr;
   if (!apm_info_ptr->graph_info.sub_graph_list_ptr)
   {
      AR_MSG(DBG_HIGH_PRIO, "apm_prepare_cmd_close_all_payload(): Sub-graph list is empty");

      return AR_EOK;
   }

   /** Get the pointer to current cmd control object  */
   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /* Add each subgraph id to the payload */
   spf_list_node_t *curr_ptr = apm_info_ptr->graph_info.sub_graph_list_ptr;

   while (curr_ptr)
   {
      sg_obj_ptr = (apm_sub_graph_t *)curr_ptr->obj_ptr;

      apm_db_add_node_to_list(&cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.reg_sg_list_ptr,
                              sg_obj_ptr,
                              &cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_reg_sub_graphs);

      /** Advance to next node in the list */
      curr_ptr = curr_ptr->next_ptr;
   }

   /** If there are any sub-graphs present managed via proxy
    *  manager, separate them out from the regular sub-graph
    *  list. */
   if (AR_EOK != (result = apm_proxy_util_sort_graph_mgmt_sg_lists(apm_info_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_prepare_cmd_close_all_payload(): Failed to update grph_mgmt lists, cmd_opcode[0x%08lx]",
             cmd_ctrl_ptr->cmd_opcode);

      return result;
   }

   return result;
}

static ar_result_t apm_handle_close_all_pre_process(void)
{
   ar_result_t result                = AR_EOK;
   bool_t      IS_FLUSH_NEEDED_TRUE  = TRUE;
   bool_t      IS_RESET_NEEDED_FALSE = FALSE;

   /** Flush on IRM and AMDB needs to done before handling close all on
       sat amdb/irm to handle following use-case sequence

       master amdb load
       Close all to master apm
       master amdb + irm Flush
       graph close
       satellite APM close all
          sat close all
          sat amdb + irm close all
       Master amdb sends load command to sat amdb (which is wrong)
       master irm + amdb reset (only after the use cases are closed)
    **/

   /** IRM reset */
   if (AR_DID_FAIL(result = irm_reset(IS_FLUSH_NEEDED_TRUE, TRUE)))
   {
      AR_MSG(DBG_ERROR_PRIO, "APM_CMD_CLOSE_ALL: Unable to flush IRM with result: %lu", result);
      return result;
   }

   /** AMDB unload and reset */
   if (AR_DID_FAIL(result = amdb_reset(IS_FLUSH_NEEDED_TRUE, IS_RESET_NEEDED_FALSE)))
   {
      AR_MSG(DBG_ERROR_PRIO, "APM_CMD_CLOSE_ALL: Unable to flush amdb with result: %lu", result);
      return result;
   }

   return result;
}

ar_result_t apm_cmd_close_all_sequencer(apm_t *apm_info_ptr)
{
   ar_result_t     result          = AR_EOK;
   apm_cmd_ctrl_t *cmd_ctrl_ptr    = NULL;
   apm_op_seq_t *  curr_op_seq_ptr = NULL;

   /** Get the pointer to current command control   */
   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Set the current op seq for close all  */
   cmd_ctrl_ptr->cmd_seq.curr_op_seq_ptr = &cmd_ctrl_ptr->cmd_seq.close_all_seq;

   /** Get the pointer to close call cmd op seq   */
   curr_op_seq_ptr = cmd_ctrl_ptr->cmd_seq.curr_op_seq_ptr;

   if (APM_CMD_SEQ_IDX_INVALID == curr_op_seq_ptr->curr_seq_idx)
   {
      curr_op_seq_ptr->curr_seq_idx = APM_CLOSE_ALL_CMD_OP_PRE_PROCESS;
   }

   AR_MSG(DBG_HIGH_PRIO,
          "apm_cmd_close_all_sequencer(): Executing curr_op_idx[0x%lX], curr_seq_idx[%lu], cmd_opcode[0x%lX]",
          cmd_ctrl_ptr->cmd_seq.close_all_seq.op_idx,
          cmd_ctrl_ptr->cmd_seq.close_all_seq.curr_seq_idx,
          cmd_ctrl_ptr->cmd_opcode);

   switch (curr_op_seq_ptr->op_idx)
   {
      case APM_CLOSE_ALL_CMD_OP_PRE_PROCESS:
      {
         if (AR_EOK != (result = apm_prepare_cmd_close_all_payload(apm_info_ptr)))
         {
            AR_MSG(DBG_ERROR_PRIO, "apm_cmd_close_all_sequencer(): Failed to populate sub-graph list");
            return result;
         }

         if (AR_EOK != (result = apm_handle_close_all_pre_process()))
         {
            AR_MSG(DBG_ERROR_PRIO, "apm_cmd_close_all_sequencer(): Failed to handle close all pre process");
            return result;
         }
         break;
      }
      case APM_CLOSE_ALL_CMD_OP_CLOSE_SG:
      {
         result = apm_cmd_graph_mgmt_sequencer(apm_info_ptr);

         /** If graph management sequence is completed, clear the
          *  pending flag for current close all operation */
         if (APM_CMN_CMD_OP_COMPLETED == cmd_ctrl_ptr->cmd_seq.graph_mgmt_seq.op_idx)
         {
            curr_op_seq_ptr->curr_cmd_op_pending = FALSE;
         }
         else
         {
            curr_op_seq_ptr->curr_cmd_op_pending = TRUE;
         }

         break;
      }
      case APM_CLOSE_ALL_CMD_OP_SAT_CLOSE:
      {
         if (apm_info_ptr->ext_utils.offload_vtbl_ptr &&
             apm_info_ptr->ext_utils.offload_vtbl_ptr->apm_send_close_all_to_sat_fptr)
         {
            result = apm_info_ptr->ext_utils.offload_vtbl_ptr->apm_send_close_all_to_sat_fptr(apm_info_ptr);
         }
         break;
      }
      case APM_CLOSE_ALL_CMD_OP_POST_PROCESS:
      {
         result = apm_finish_graph_close_all_cmd(apm_info_ptr);

         break;
      }
      case APM_CLOSE_ALL_CMD_OP_HANDLE_FAILURE:
      case APM_CLOSE_ALL_CMD_OP_COMPLETED:
      {
         apm_end_cmd_op_sequencer(cmd_ctrl_ptr, curr_op_seq_ptr);

         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_cmd_close_all_sequencer(): Un-support cmd operation index[%lu]",
                curr_op_seq_ptr->op_idx);

         /** Abort command sequencer   */
         apm_abort_cmd_op_sequencer(cmd_ctrl_ptr);

         break;
      }

   } /** End of switch() */

   return result;
}

ar_result_t apm_close_all_utils_init(apm_t *apm_info_ptr)
{
   ar_result_t result = AR_EOK;

   apm_info_ptr->ext_utils.close_all_vtbl_ptr = &close_all_util_funcs;

   return result;
}
