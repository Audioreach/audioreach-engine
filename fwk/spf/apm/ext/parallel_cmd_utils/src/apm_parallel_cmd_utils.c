/**
 * \file apm_parallel_cmd_utils.c
 *
 * \brief
 *     This file contains utility functions for APM parallel
 *     command handling
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include "apm_internal.h"
#include "apm_cmd_utils.h"

/**==============================================================================
   Function Declaration
==============================================================================*/

ar_result_t apm_check_and_defer_cmd_processing(apm_t *apm_info_ptr, bool_t *cmd_proc_deferred_ptr);

ar_result_t apm_check_def_cmd_is_ready_to_process(apm_t *apm_info_ptr);

ar_result_t apm_update_deferred_gm_cmd(apm_t *apm_info_ptr, apm_sub_graph_t *closed_sg_ptr);

/**==============================================================================
   Global Defines
==============================================================================*/

apm_parallel_cmd_utils_vtable_t parallel_cmd_util_funcs = {
   .apm_check_and_defer_cmd_processing_fptr = apm_check_and_defer_cmd_processing,

   .apm_check_def_cmd_is_ready_to_process_fptr = apm_check_def_cmd_is_ready_to_process,

   .apm_update_deferred_gm_cmd_fptr = apm_update_deferred_gm_cmd
};

static ar_result_t apm_gm_cmd_check_overlap_for_single_sg_id(apm_cmd_ctrl_t *curr_cmd_ctrl_ptr,
                                                             apm_cmd_ctrl_t *list_cmd_ctrl_ptr,
                                                             uint32_t        tgt_sg_id,
                                                             bool_t *        sg_list_overlap_ptr)
{
   ar_result_t         result = AR_EOK;
   apm_sub_graph_id_t *list_sg_list_ptr;
   uint32_t            list_num_sg_id;
   spf_list_node_t *   curr_list_sg_node_ptr;
   apm_sub_graph_t *   curr_list_sg_obj_ptr;

   if (!curr_cmd_ctrl_ptr || !list_cmd_ctrl_ptr || !tgt_sg_id)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_gm_cmd_check_overlap_for_single_sg_id(): Invalid i/p arg curr_cmd_ctrl[0x%lX], "
             "list_cmd_ctrl[0x%lX], tgt SG_ID[0x%lX]",
             curr_cmd_ctrl_ptr,
             list_cmd_ctrl_ptr,
             tgt_sg_id);

      return AR_EFAILED;
   }

   /** Get the pointer to list of sub-graph ID for concurrent graph
    *  managment commands under process */
   list_sg_list_ptr = list_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.cmd_sg_id_list_ptr;
   list_num_sg_id   = list_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_cmd_sg_id;

   for (uint32_t list_idx = 0; list_idx < list_num_sg_id; list_idx++)
   {
      if (tgt_sg_id == list_sg_list_ptr[list_idx].sub_graph_id)
      {
         /** If the match is found, return   */
         *sg_list_overlap_ptr = TRUE;

         AR_MSG(DBG_HIGH_PRIO,
                "apm_gm_cmd_check_overlap_for_single_sg_id(): SG_ID[0x%lX] in cmd_list_idx[%lu], cmd_opcode[0x%lX] , "
                "directly matches "
                "with cmd_list_idx[%lu], cmd_opcode[0x%lX] ",
                tgt_sg_id,
                curr_cmd_ctrl_ptr->list_idx,
                curr_cmd_ctrl_ptr->cmd_opcode,
                list_cmd_ctrl_ptr->list_idx,
                list_cmd_ctrl_ptr->cmd_opcode);

         return AR_EOK;

      } /** End of if (sub-graph ID match ) */

   } /** End of for(list sub-graph id arr) */

   /** Execution falls through if current target sub-graph ID
    *  received directly as part of GM command is not present in
    *  other commands. Now check if this target sub-graph ID
    *  matches with sub-graph ID being processed due to link
    *  operation as part of other concurrently active commands */

   /** Get the list of sub-graph ID's being processed due to GM
    *  operation on links */
   curr_list_sg_node_ptr = list_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.cont_port_hdl_sg_list_ptr;

   /** Iterate over the list of sub-graph ID's. If any of the
    *  sub-graph matches with target sub-graph ID, break the
    *  loop */
   while (curr_list_sg_node_ptr)
   {
      curr_list_sg_obj_ptr = (apm_sub_graph_t *)curr_list_sg_node_ptr->obj_ptr;

      /** If the match is found, break  */
      if (tgt_sg_id == curr_list_sg_obj_ptr->sub_graph_id)
      {
         *sg_list_overlap_ptr = TRUE;

         AR_MSG(DBG_HIGH_PRIO,
                "apm_gm_cmd_check_overlap_for_single_sg_id(): SG_ID[0x%lX] in cmd_list_idx[%lu], cmd_opcode[0x%lX], "
                "indirectly matches "
                "with cmd_list_idx[%lu], cmd_opcode[0x%lX] ",
                tgt_sg_id,
                curr_cmd_ctrl_ptr->list_idx,
                curr_cmd_ctrl_ptr->cmd_opcode,
                list_cmd_ctrl_ptr->list_idx,
                list_cmd_ctrl_ptr->cmd_opcode);

         return AR_EOK;
      }

      /** Advance to next node in the list   */
      curr_list_sg_node_ptr = curr_list_sg_node_ptr->next_ptr;

   } /** End of while(list sub-graph ID arr) */

   return result;
}

static ar_result_t apm_gm_cmd_check_sg_list_overlap(apm_t *         apm_info_ptr,
                                                    apm_cmd_ctrl_t *curr_cmd_ctrl_ptr,
                                                    apm_cmd_ctrl_t *list_cmd_ctrl_ptr,
                                                    bool_t *        sg_list_overlap_ptr)
{
   ar_result_t                   result = AR_EOK;
   apm_sub_graph_id_t *          tgt_sg_list_ptr;
   uint32_t                      tgt_num_sg_id;
   spf_list_node_t *             curr_tgt_sg_node_ptr, *curr_node_ptr;
   apm_sub_graph_t *             curr_tgt_sg_obj_ptr;
   apm_cont_port_connect_info_t *curr_port_conn_info_obj_ptr;

   /** Validate input arguments   */
   if (!curr_cmd_ctrl_ptr || !list_cmd_ctrl_ptr || !sg_list_overlap_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_gm_cmd_check_sg_list_overlap(): Invalid i/p arg, curr_cmd_ctrl_ptr[0x%lX], list_cmd_ctrl_ptr[0x%lX], "
             "sg_list_overlap_ptr[0x%lX] ",
             curr_cmd_ctrl_ptr,
             list_cmd_ctrl_ptr,
             sg_list_overlap_ptr);

      return AR_EFAILED;
   }

   AR_MSG(DBG_HIGH_PRIO,
          "apm_gm_cmd_check_sg_list_overlap(): Checking overlap across, curr_cmd_ctrl_idx[%lu], "
          "list_cmd_ctrl_idx[%lu] ",
          curr_cmd_ctrl_ptr->list_idx,
          list_cmd_ctrl_ptr->list_idx);

   /** Init the return value  */
   *sg_list_overlap_ptr = FALSE;

   /** Get the pointer to the list of sub-graph IDs for current
    *  graph managment command under process */
   tgt_sg_list_ptr = curr_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.cmd_sg_id_list_ptr;
   tgt_num_sg_id   = curr_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_cmd_sg_id;

   /** Check if there is at least 1 sub-graph ID matches for the
    *  current command with other active commands */
   for (uint32_t tgt_idx = 0; tgt_idx < tgt_num_sg_id; tgt_idx++)
   {
      if (AR_EOK != (result = apm_gm_cmd_check_overlap_for_single_sg_id(curr_cmd_ctrl_ptr,
                                                                        list_cmd_ctrl_ptr,
                                                                        tgt_sg_list_ptr[tgt_idx].sub_graph_id,
                                                                        sg_list_overlap_ptr)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_gm_cmd_check_sg_list_overlap(): Failed to check overlap for SG_ID[0x%lX] "
                "cmd_opcode[0x%lX] ",
                tgt_sg_list_ptr[tgt_idx].sub_graph_id,
                curr_cmd_ctrl_ptr->cmd_opcode);

         return result;
      }

      /** If overlap found, break the loop */
      if (*sg_list_overlap_ptr)
      {
         break;
      }

      /** Execution falls through  if no overlap found for the
       *  current sub-graph ID */

      /** For current sub-graph ID, fetch its peer sub-graph ID and
       *  check if that is getting processed as part of a parallel
       *  command execution */
      curr_node_ptr = apm_info_ptr->graph_info.sub_graph_conn_list_ptr;

      /** Iterate over the list of all the connected sub-graphs   */
      while (curr_node_ptr)
      {
         curr_port_conn_info_obj_ptr = (apm_cont_port_connect_info_t *)curr_node_ptr->obj_ptr;

         /** Get the data connection where the target sub-graph ID is
          *  self sub-graph ID */
         if (curr_port_conn_info_obj_ptr->self_sg_obj_ptr && curr_port_conn_info_obj_ptr->peer_sg_obj_ptr &&
             (tgt_sg_list_ptr[tgt_idx].sub_graph_id == curr_port_conn_info_obj_ptr->self_sg_obj_ptr->sub_graph_id))
         {
            if (AR_EOK != (result = apm_gm_cmd_check_overlap_for_single_sg_id(curr_cmd_ctrl_ptr,
                                                                              list_cmd_ctrl_ptr,
                                                                              curr_port_conn_info_obj_ptr
                                                                                 ->peer_sg_obj_ptr->sub_graph_id,
                                                                              sg_list_overlap_ptr)))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "apm_gm_cmd_check_sg_list_overlap(): Failed to check overlap for SG_ID[0x%lX] "
                      "cmd_opcode[0x%lX] ",
                      tgt_sg_list_ptr[tgt_idx].sub_graph_id,
                      curr_cmd_ctrl_ptr->cmd_opcode);

               return result;
            }
         }

         /** If overlap found, break the loop */
         if (*sg_list_overlap_ptr)
         {
            break;
         }

         /** Advance to next node in the list   */
         curr_node_ptr = curr_node_ptr->next_ptr;

      } /** End of while(global sub-graph connection list) */

      /** If overlap found, break the outer for-loop */
      if (*sg_list_overlap_ptr)
      {
         break;
      }

   } /** End of for(target sub-graph ID arr) */

   /** Execution falls through if no match found in the list of
    *  sub-graph ID's received directly as part of graph mgmt
    *  commands. Next to check overlap with sub-graph id's
    *  getting processed indirectly e.g. as part of link closure */

   if (!(*sg_list_overlap_ptr))
   {
      /** Get the pointer to list of sub-graphs getting parsed as
       *  part of data/control link operation */
      curr_tgt_sg_node_ptr = curr_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.cont_port_hdl_sg_list_ptr;

      /** Iterate over this sub-graph list  */
      while (curr_tgt_sg_node_ptr)
      {
         /** Get the pointer to current sub-graph object   */
         curr_tgt_sg_obj_ptr = (apm_sub_graph_t *)curr_tgt_sg_node_ptr->obj_ptr;

         if (AR_EOK != (result = apm_gm_cmd_check_overlap_for_single_sg_id(curr_cmd_ctrl_ptr,
                                                                           list_cmd_ctrl_ptr,
                                                                           curr_tgt_sg_obj_ptr->sub_graph_id,
                                                                           sg_list_overlap_ptr)))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "apm_gm_cmd_check_sg_list_overlap(): Failed to check overlap for SG_ID[0x%lX] "
                   "cmd_opcode[0x%lX] ",
                   curr_tgt_sg_obj_ptr->sub_graph_id,
                   curr_cmd_ctrl_ptr->cmd_opcode);

            return result;
         }

         /** Advance to next node in the list   */
         curr_tgt_sg_node_ptr = curr_tgt_sg_node_ptr->next_ptr;

      } /** End of wehile (cont port hdl sgid list) */

   } /** End of if(no overlap with direct sg id list) */

   return result;
}

ar_result_t apm_check_sg_id_overlap_across_parallel_cmds(apm_t *         apm_info_ptr,
                                                         apm_cmd_ctrl_t *curr_cmd_ctrl_ptr,
                                                         bool_t *        sg_list_overlap_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *list_cmd_ctrl_ptr;

   /** Init return value   */
   *sg_list_overlap_ptr = FALSE;

   /** Iterate over the list of all the concurrently active
    *  commands  */
   for (uint32_t idx = 0; idx < APM_NUM_MAX_PARALLEL_CMD; idx++)
   {
      list_cmd_ctrl_ptr = &apm_info_ptr->cmd_ctrl_list[idx];

      if (list_cmd_ctrl_ptr->cmd_pending && (list_cmd_ctrl_ptr != curr_cmd_ctrl_ptr))
      {
         /** In case atleast one command is pending (which is checked by
          *  above check) in the assigned list and the current commands
          *  evaluated for resume is close all. CLOSE_ALL resume is
          *  deferred. */

         if (APM_CMD_CLOSE_ALL == curr_cmd_ctrl_ptr->cmd_opcode)
         {
            AR_MSG(DBG_HIGH_PRIO,
                   "apm_check_sg_id_overlap_across_parallel_cmds(): Did not resume already deferred CLOSE_ALL ");

            *sg_list_overlap_ptr = TRUE;
            break;
         }

         result =
            apm_gm_cmd_check_sg_list_overlap(apm_info_ptr, curr_cmd_ctrl_ptr, list_cmd_ctrl_ptr, sg_list_overlap_ptr);

         /** If CLOSE_ALL cmd is already in progress, break the loop. As part of processing CLOSE_ALL cmd,
           * APM will initiate the STOP sequence. Hence APM can ignore the current PROXY graph mgmt cmd from VCPM.   */
         if ( ( (curr_cmd_ctrl_ptr->cmd_opcode == SPF_MSG_CMD_PROXY_GRAPH_STOP  ) ||
                 (curr_cmd_ctrl_ptr->cmd_opcode == SPF_MSG_CMD_PROXY_GRAPH_START  ) ||
                 (curr_cmd_ctrl_ptr->cmd_opcode == SPF_MSG_CMD_PROXY_GRAPH_PREPARE  ) ) && 
               (list_cmd_ctrl_ptr->cmd_opcode == APM_CMD_CLOSE_ALL ))
         {
            curr_cmd_ctrl_ptr->cmd_status  = AR_EBUSY;
            curr_cmd_ctrl_ptr->cmd_pending = FALSE;
            result                         = AR_EBUSY;

            AR_MSG(DBG_HIGH_PRIO,
                   "apm_check_sg_id_overlap_across_parallel_cmds():  found overlap of CLOSE_ALL and PROXY STOP");
            break;
         }
         /** If sub-graph list overlap is found, break the loop   */
         if (*sg_list_overlap_ptr)
         {
            break;
         }
      }
   }

   return result;
}

static ar_result_t apm_resume_deferred_cmd_proc(apm_t *apm_info_ptr, spf_list_node_t **def_cmd_ctrl_list_node_pptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *def_cmd_ctrl_ptr;

   /** Get the pointer to command control object  */
   def_cmd_ctrl_ptr = (apm_cmd_ctrl_t *)((*def_cmd_ctrl_list_node_pptr)->obj_ptr);

   /** Remove this node from deferred command  */
   spf_list_delete_node_update_head(def_cmd_ctrl_list_node_pptr,
                                    &apm_info_ptr->def_cmd_list.deferred_cmd_list.list_ptr,
                                    TRUE /** Pool used*/);

   /** Decrement deferred cmd count */
   apm_info_ptr->def_cmd_list.deferred_cmd_list.num_nodes--;

   /** If the current command being made active is CLOSE ALL,
    *  clear the corresponding flag */
   apm_info_ptr->def_cmd_list.close_all_deferred = FALSE;

   /** Set this deferred cmd ctrl object as current */
   apm_info_ptr->curr_cmd_ctrl_ptr = def_cmd_ctrl_ptr;

   AR_MSG(DBG_HIGH_PRIO,
          "apm_resume_deferred_cmd_proc(): Resume deferred cmd processing, cmd_opcode[0x%lX], "
          "cmd_ctrl_idx[%lu]",
          def_cmd_ctrl_ptr->cmd_opcode,
          def_cmd_ctrl_ptr->list_idx);

   /** Call the command sequencer, corresponding to current
    *  opcode under process */
   if (AR_EOK != (result = apm_cmd_sequencer_cmn_entry(apm_info_ptr)))
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_resume_deferred_cmd_proc(), Cmd resume failed, cmd_opcode[0x%lX], result[%lu]",
             def_cmd_ctrl_ptr->cmd_opcode,
             result);
   }

   return result;
}

static uint32_t apm_get_num_available_cmd(apm_t *apm_info_ptr)
{
   uint32_t num_available_cmd = 0;

   for (uint32_t list_idx = 0; list_idx < APM_NUM_MAX_PARALLEL_CMD; list_idx++)
   {
      if (apm_info_ptr->cmd_ctrl_list[list_idx].cmd_pending)
      {
         num_available_cmd++;
      }
   }

   return num_available_cmd;
}

ar_result_t apm_check_def_cmd_is_ready_to_process(apm_t *apm_info_ptr)
{
   ar_result_t      result = AR_EOK;
   spf_list_node_t *curr_node_ptr, *next_node_ptr;
   apm_cmd_ctrl_t * def_cmd_ctrl_ptr;
   bool_t           sg_list_overlap = FALSE;
   uint32_t         num_def_cmds;

   /** If there are no deferred commands, then return   */
   if (!apm_info_ptr->def_cmd_list.deferred_cmd_list.num_nodes)
   {
      AR_MSG(DBG_LOW_PRIO, "apm_resume_deferred_cmd_proc(): Deferred cmd list is empty");
      return AR_EOK;
   }

   /** Get the pointer to list of deferred commands */
   curr_node_ptr = apm_info_ptr->def_cmd_list.deferred_cmd_list.list_ptr;

   /** Get number of deferred commands  */
   num_def_cmds = apm_info_ptr->def_cmd_list.deferred_cmd_list.num_nodes;

   /** If number of deferred command(s) is(are) equal to number of
    *  available commands to process, then resume the first
    *  available deferred command since there are no
    *  other active commands executing.
    *  Note that the execution can reach here if a secondary
    *  command ends in context of a primary command in which case
    *  we need to re-evalulate if a deferred command can resume
    *  processing and cannot implicitly resume the first deferred
    *  command.
    *  */
   if (num_def_cmds == apm_get_num_available_cmd(apm_info_ptr))
   {
      result = apm_resume_deferred_cmd_proc(apm_info_ptr, &curr_node_ptr);

      return result;
   }

   /** Execution falls through if we need to further evalute if any
    *  deferred command can resume processing  */

   /** Iterate over list of deferred commands   */
   while (curr_node_ptr)
   {
      /** Get the pointer to current deferred command control
       *  object */
      def_cmd_ctrl_ptr = (apm_cmd_ctrl_t *)curr_node_ptr->obj_ptr;

      /** Get the next list node ptr  */
      next_node_ptr = curr_node_ptr->next_ptr;

      /** Check current command control against all the active and
       *  deferred commands */
      if (AR_EOK !=
          (result = apm_check_sg_id_overlap_across_parallel_cmds(apm_info_ptr, def_cmd_ctrl_ptr, &sg_list_overlap)))
      {
         return result;
      }

      /** If no overlap found, the current deferred command is
       *  ready to process */
      if (!sg_list_overlap)
      {
         result = apm_resume_deferred_cmd_proc(apm_info_ptr, &curr_node_ptr);

         /** Break the loop */
         break;

      } /** End of if (sub graph list overlap) */

      /** Advance to next node in the list  */
      curr_node_ptr = next_node_ptr;
   }

   return result;
}

static bool_t apm_check_if_graph_close_cmd_active(apm_t *apm_info_ptr)
{
   apm_cmd_ctrl_t *curr_cmd_ctrl_ptr;

   for (uint32_t idx = 0; idx < APM_NUM_MAX_PARALLEL_CMD; idx++)
   {
      curr_cmd_ctrl_ptr = &apm_info_ptr->cmd_ctrl_list[idx];

      if (curr_cmd_ctrl_ptr->cmd_pending && (APM_CMD_GRAPH_CLOSE == curr_cmd_ctrl_ptr->cmd_opcode))
      {
         return TRUE;
      }
   }

   return FALSE;
}

ar_result_t apm_check_and_defer_cmd_processing(apm_t *apm_info_ptr, bool_t *cmd_proc_deferred_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *curr_cmd_ctrl_ptr;
   bool_t          sg_id_overlapped = FALSE;

   /** Validate input arguments */
   if (!apm_info_ptr || !cmd_proc_deferred_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_check_and_defer_cmd_processing(), Invalid i/p arg apm_info_ptr[0x%lX], cmd_proc_deferred_ptr[%lu]",
             apm_info_ptr,
             cmd_proc_deferred_ptr);

      return AR_EFAILED;
   }

   /** Init return pointer  */
   *cmd_proc_deferred_ptr = FALSE;

   /** Get the pointer to current command control object  */
   curr_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** For the command deferrable, check if the current command is
    *  graph management command and there are other parallel
    *  commands being executed */
   if (!apm_is_graph_mgmt_cmd_opcode(curr_cmd_ctrl_ptr->cmd_opcode) ||
       !apm_check_if_multiple_cmds_executing(apm_info_ptr->active_cmd_mask))
   {
      /** Nothing to do  */
      return AR_EOK;
   }

   /** If the current command is not CLOSE ALL, check if the
    *  sub-graph ID overlaps with concurrently active gm
    *  commands */
   if (APM_CMD_CLOSE_ALL != curr_cmd_ctrl_ptr->cmd_opcode)
   {
      if (AR_EOK !=
          (result = apm_check_sg_id_overlap_across_parallel_cmds(apm_info_ptr, curr_cmd_ctrl_ptr, &sg_id_overlapped)))
      {
         return result;
      }
   }

   /** Check if close all needs to be deferred or if the sub-graph
    *  lists overlap across non-close all gm commands */
   if (apm_defer_close_all_cmd(apm_info_ptr) || sg_id_overlapped)
   {
      /** If overlap, mark this command as deferred. Linked list is
       *  required to maintain the ordering of deferred commands
       *  when they are ready to execute */
      if (AR_EOK != (result = apm_db_add_node_to_list(&apm_info_ptr->def_cmd_list.deferred_cmd_list.list_ptr,
                                                      curr_cmd_ctrl_ptr,
                                                      &apm_info_ptr->def_cmd_list.deferred_cmd_list.num_nodes)))
      {
         return result;
      }

      /** Set the flag if CLOSE ALL command has been deferred  */
      if (APM_CMD_CLOSE_ALL == curr_cmd_ctrl_ptr->cmd_opcode)
      {
         apm_info_ptr->def_cmd_list.close_all_deferred = TRUE;
      }

      AR_MSG(DBG_HIGH_PRIO,
             "apm_check_and_defer_cmd_processing(), Deferred processing cmd_opcode: 0x%8lX, "
             "cmd_ctrl_list_idx[%lu]",
             curr_cmd_ctrl_ptr->cmd_opcode,
             curr_cmd_ctrl_ptr->list_idx);

      /** Set the flag to defer the current command processing  */
      *cmd_proc_deferred_ptr = TRUE;

   } /** End of If command processing deferred */

   /** For a proxy graph mgmt command, if this command is
    *  overlapping, then deferral is done only if any close
    *  command is running in parallel. If no close
    *  cmd, then skip deferral. */
   if (apm_is_proxy_graph_mgmt_cmd_opcode(curr_cmd_ctrl_ptr->cmd_opcode) && *cmd_proc_deferred_ptr &&
       !apm_check_if_graph_close_cmd_active(apm_info_ptr))
   {
      /** Clear deferred flag   */
      *cmd_proc_deferred_ptr = FALSE;

      /** Remove this command from list of pending commands  */
      if (AR_EOK != (result = apm_db_remove_node_from_list(&apm_info_ptr->def_cmd_list.deferred_cmd_list.list_ptr,
                                                           curr_cmd_ctrl_ptr,
                                                           &apm_info_ptr->def_cmd_list.deferred_cmd_list.num_nodes)))
      {
         return result;
      }

      AR_MSG(DBG_HIGH_PRIO,
             "apm_check_and_defer_cmd_processing(), Reverted deferral of cmd_opcode: 0x%8lX, "
             "cmd_ctrl_list_idx[%lu]",
             curr_cmd_ctrl_ptr->cmd_opcode,
             curr_cmd_ctrl_ptr->list_idx);
   }

   return result;
}

static ar_result_t apm_remove_closed_sg_from_def_gm_cmd(apm_cmd_ctrl_t * def_cmd_ctrl_ptr,
                                                        apm_sub_graph_t *closed_sg_ptr)
{
   ar_result_t      result = AR_EOK;
   spf_list_node_t *curr_node_ptr, *next_node_ptr;
   apm_sub_graph_t *curr_sg_obj_ptr;

   curr_node_ptr = def_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.reg_sg_list_ptr;

   while (curr_node_ptr)
   {
      next_node_ptr = curr_node_ptr->next_ptr;

      curr_sg_obj_ptr = (apm_sub_graph_t *)curr_node_ptr->obj_ptr;

      if (curr_sg_obj_ptr->sub_graph_id == closed_sg_ptr->sub_graph_id)
      {
         spf_list_find_delete_node(&def_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.reg_sg_list_ptr,
                                   curr_sg_obj_ptr,
                                   TRUE);

         /** Decrement the number of regular sub-graphs */
         def_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_reg_sub_graphs--;

         AR_MSG(DBG_HIGH_PRIO,
                "apm_remove_closed_sg_from_def_gm_cmd(), Removed SG_ID[0x%lX] for cmd_opcode: 0x%8lX, "
                "def_cmd_ctrl_list_idx[%lu]",
                curr_sg_obj_ptr->sub_graph_id,
                def_cmd_ctrl_ptr->cmd_opcode,
                def_cmd_ctrl_ptr->list_idx);
      }

      /** Advance to next node in the list */
      curr_node_ptr = next_node_ptr;
   }

   return result;
}

ar_result_t apm_update_deferred_gm_cmd(apm_t *apm_info_ptr, apm_sub_graph_t *closed_sg_ptr)
{
   ar_result_t      result = AR_EOK;
   spf_list_node_t *curr_node_ptr;
   apm_cmd_ctrl_t * def_cmd_ctrl_ptr;

   curr_node_ptr = apm_info_ptr->def_cmd_list.deferred_cmd_list.list_ptr;

   /** Iterate over the list of defered commands, if present.
    *  Only proxy commands are being handled because for apps
    *  commands, client is expected to ensure that operation on
    *  same sub-graph ID is done in synchronous manner */
   while (curr_node_ptr)
   {
      def_cmd_ctrl_ptr = (apm_cmd_ctrl_t *)curr_node_ptr->obj_ptr;

      switch (def_cmd_ctrl_ptr->cmd_opcode)
      {
         case SPF_MSG_CMD_PROXY_GRAPH_PREPARE:
         case SPF_MSG_CMD_PROXY_GRAPH_START:
         case SPF_MSG_CMD_PROXY_GRAPH_STOP:
         {
            result = apm_remove_closed_sg_from_def_gm_cmd(def_cmd_ctrl_ptr, closed_sg_ptr);

            break;
         }
         default:
         {
            AR_MSG(DBG_HIGH_PRIO,
                   "apm_update_deferred_gm_cmd(), Unexpected cmd_opcode: 0x%8lX, list_idx[%lu] ",
                   def_cmd_ctrl_ptr->cmd_opcode,
                   def_cmd_ctrl_ptr->list_idx);

            result = AR_EOK;

            break;
         }
      }

      /** Advance to next node in the list   */
      curr_node_ptr = curr_node_ptr->next_ptr;

   } /** End of while(def'd cmd list) */

   return result;
}

ar_result_t apm_parallel_cmd_utils_init(apm_t *apm_info_ptr)
{
   ar_result_t result = AR_EOK;

   apm_info_ptr->ext_utils.parallel_cmd_utils_vtbl_ptr = &parallel_cmd_util_funcs;

   return result;
}
