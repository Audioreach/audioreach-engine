/**
 * \file wear_cntr_cmd_handler.c
 * \brief
 *   This file contains functions for command hanlder code for WCNTR
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "wear_cntr_i.h"
#include "wear_cntr_utils.h"
#include "posal_queue.h"
#include "apm.h"

#define WR_CNTR_PROPAGATION_RECURSE_MAX_DEPTH 10

static ar_result_t wcntr_set_get_cfgs_packed_loop(wcntr_base_t *      me_ptr,
                                                  uint8_t *           data_ptr,
                                                  uint32_t            miid,
                                                  uint32_t            payload_size,
                                                  bool_t              is_wcntr_set_cfg,
                                                  bool_t              is_oob,
                                                  bool_t              is_deregister,
                                                  spf_cfg_data_type_t cfg_type);

ar_result_t wcntr_handle_prepare(wcntr_base_t *base_ptr, spf_msg_cmd_graph_mgmt_t *cmd_gmgmt_ptr);

static void wcntr_update_module_process_flag(wcntr_base_t *me_ptr, wcntr_gu_sg_t *sg_ptr);
static ar_result_t wcntr_update_data_port_states(wcntr_base_t *me_ptr, wcntr_gu_sg_t *sg_ptr);
static void wcntr_update_boundary_out_port_states(wcntr_base_t *          me_ptr,
                                                  wcntr_gu_sg_t *         sg_ptr,
                                                  wcntr_topo_port_state_t self_port_state);
ar_result_t wcntr_topo_propagate_port_state_forwards(void *    vtopo_ptr,
                                                     void *    vin_port_ptr,
                                                     uint32_t  propagated_value,
                                                     uint32_t *recurse_depth_ptr);
wcntr_topo_port_state_t wcntr_get_downgraded_state(wcntr_topo_port_state_t self_port_state,
                                                   wcntr_topo_port_state_t connected_port_state);

ar_result_t wcntr_print_port_state(wcntr_base_t *me_ptr, wcntr_gu_sg_t *sg_ptr)
{
   ar_result_t result = AR_EOK;

   for (wcntr_gu_module_list_t *module_list_ptr = sg_ptr->module_list_ptr; (NULL != module_list_ptr);
        LIST_ADVANCE(module_list_ptr))
   {
      wcntr_gu_module_t *module_ptr = (wcntr_gu_module_t *)module_list_ptr->module_ptr;

      for (wcntr_gu_input_port_list_t *in_port_list_ptr = module_ptr->input_port_list_ptr; (NULL != in_port_list_ptr);
           LIST_ADVANCE(in_port_list_ptr))
      {
         wcntr_topo_input_port_t *in_port_ptr = (wcntr_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;
         WCNTR_MSG(me_ptr->gu_ptr->log_id,
                   DBG_HIGH_PRIO,
                   "print_port_state "
                   "miid,input_port_id (0x%lX,0x%lx) state %u ",
                   in_port_ptr->gu.cmn.module_ptr->module_instance_id,
                   in_port_ptr->gu.cmn.id,
                   in_port_ptr->common.state);
      }

      for (wcntr_gu_output_port_list_t *out_port_list_ptr = module_ptr->output_port_list_ptr;
           (NULL != out_port_list_ptr);
           LIST_ADVANCE(out_port_list_ptr))
      {
         wcntr_topo_output_port_t *out_port_ptr = (wcntr_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
         WCNTR_MSG(me_ptr->gu_ptr->log_id,
                   DBG_HIGH_PRIO,
                   "print_port_state "
                   "miid,output_port_id (0x%lX,0x%lx)  state %u ",
                   out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                   out_port_ptr->gu.cmn.id,
                   out_port_ptr->common.state);
      }

      for (wcntr_gu_ctrl_port_list_t *ctrl_port_list_ptr = module_ptr->ctrl_port_list_ptr; (NULL != ctrl_port_list_ptr);
           LIST_ADVANCE(ctrl_port_list_ptr))
      {
         wcntr_topo_ctrl_port_t *ctrl_port_ptr = (wcntr_topo_ctrl_port_t *)ctrl_port_list_ptr->ctrl_port_ptr;

         WCNTR_MSG(me_ptr->gu_ptr->log_id,
                   DBG_HIGH_PRIO,
                   "print_port_state "
                   "miid,control_port_id (0x%lX,0x%lx) state %u ",
                   ctrl_port_ptr->gu.module_ptr->module_instance_id,
                   ctrl_port_ptr->gu.id,
                   ctrl_port_ptr->state);
      }
   }

   return result;
}

static ar_result_t wcntr_update_ctrl_port_states(wcntr_base_t *me_ptr, wcntr_gu_sg_t *sg_ptr)
{
   ar_result_t result = AR_EOK;

   for (wcntr_gu_module_list_t *module_list_ptr = sg_ptr->module_list_ptr; (NULL != module_list_ptr);
        LIST_ADVANCE(module_list_ptr))
   {
      wcntr_gu_module_t *module_ptr = (wcntr_gu_module_t *)module_list_ptr->module_ptr;

      for (wcntr_gu_ctrl_port_list_t *ctrl_port_list_ptr = module_ptr->ctrl_port_list_ptr; (NULL != ctrl_port_list_ptr);
           LIST_ADVANCE(ctrl_port_list_ptr))
      {

         wcntr_topo_ctrl_port_t *ctrl_port_ptr        = (wcntr_topo_ctrl_port_t *)ctrl_port_list_ptr->ctrl_port_ptr;
         wcntr_topo_ctrl_port_t *peer_ctrl_port_ptr   = (wcntr_topo_ctrl_port_t *)ctrl_port_ptr->gu.peer_ctrl_port_ptr;
         wcntr_topo_module_t *   this_module_ptr      = (wcntr_topo_module_t *)ctrl_port_ptr->gu.module_ptr;
         wcntr_topo_port_state_t peer_ctrl_port_state = WCNTR_TOPO_PORT_STATE_INVALID;
         wcntr_topo_port_state_t self_port_state = wcntr_topo_sg_state_to_port_state(wcntr_topo_get_sg_state(sg_ptr));

         if (peer_ctrl_port_ptr)
         {
            peer_ctrl_port_state = peer_ctrl_port_ptr->state;
         }

         wcntr_topo_port_state_t state_to_be_set = wcntr_get_downgraded_state(self_port_state, peer_ctrl_port_state);
         intf_extn_imcl_port_opcode_t op_code    = wcntr_topo_port_state_to_ctrl_port_opcode(state_to_be_set);

         if (op_code != INTF_EXTN_IMCL_PORT_STATE_INVALID)
         {
            ctrl_port_ptr->state = state_to_be_set;
            result =
               wcntr_topo_set_ctrl_port_operation(&ctrl_port_ptr->gu, op_code, this_module_ptr->topo_ptr->heap_id);
         }
      }
   }
   return result;
}



ar_result_t wcntr_handle_current_sg_mgmt_cmd(wcntr_base_t *        me_ptr,
                                             wcntr_gu_sg_t *       sg_ptr,
                                             uint32_t              sg_ops,
                                             wcntr_topo_sg_state_t sg_state)
{
   ar_result_t result = AR_EOK;
   if (sg_ptr == NULL)
   {

      WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_LOW_PRIO, "wcntr_handle_current_sg_mgmt_cmd  sg_ptr cannot be NULL");
      return AR_EFAILED;
   }
   wcntr_topo_port_state_t self_port_state = wcntr_topo_sg_state_to_port_state(sg_state);

   WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_LOW_PRIO,
             "handle_current_sg_mgmt_cmd: sg_id 0x%lX,sg_ops 0x%lX,sg_state 0x%lX, self_port_state 0x%lX ",
             sg_ptr->id,
             sg_ops,
             sg_state,
             self_port_state);

   for (wcntr_gu_module_list_t *module_list_ptr = sg_ptr->module_list_ptr; (NULL != module_list_ptr);
        LIST_ADVANCE(module_list_ptr))
   {
      wcntr_gu_module_t *  module_ptr      = (wcntr_gu_module_t *)module_list_ptr->module_ptr;
      wcntr_topo_module_t *topo_module_ptr = (wcntr_topo_module_t *)module_ptr;

      for (wcntr_gu_output_port_list_t *out_port_list_ptr = module_ptr->output_port_list_ptr;
           (NULL != out_port_list_ptr);
           LIST_ADVANCE(out_port_list_ptr))
      {
         wcntr_topo_output_port_t *out_port_ptr = (wcntr_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
         out_port_ptr->common.state_updated_during_propagation = FALSE;

         if (sg_ops & WCNTR_TOPO_SG_OP_STOP)
         {
            wcntr_topo_reset_output_port(topo_module_ptr->topo_ptr, (void *)out_port_ptr);
         }

         if (out_port_ptr->gu.conn_in_port_ptr)
         {
            if (sg_ops & WCNTR_TOPO_SG_OP_CLOSE)
            {
               if (out_port_ptr->gu.cmn.boundary_port)
               {
                  wcntr_topo_input_port_t *conn_in_port_ptr =
                     (wcntr_topo_input_port_t *)out_port_ptr->gu.conn_in_port_ptr;
                  wcntr_topo_module_t *connected_module_ptr =
                     (wcntr_topo_module_t *)conn_in_port_ptr->gu.cmn.module_ptr;

                  if (connected_module_ptr->capi_ptr)
                  {

                     result = wcntr_topo_capi_set_data_port_op_from_sg_ops(connected_module_ptr,
                                                                           sg_ops,
                                                                           &conn_in_port_ptr->common.last_issued_opcode,
                                                                           TRUE,
                                                                           conn_in_port_ptr->gu.cmn.index,
                                                                           conn_in_port_ptr->gu.cmn.id);
                  }

                  wcntr_topo_destroy_cmn_port(connected_module_ptr,
                                              &conn_in_port_ptr->common,
                                              me_ptr->gu_ptr->log_id,
                                              conn_in_port_ptr->gu.cmn.id,
                                              TRUE);
               }
               out_port_ptr->gu.conn_in_port_ptr->conn_out_port_ptr = NULL;
               out_port_ptr->gu.conn_in_port_ptr                    = NULL;
            }
         }
         out_port_ptr->common.state = self_port_state;
         wcntr_topo_capi_set_data_port_op_from_data_port_state((wcntr_topo_module_t *)out_port_ptr->gu.cmn.module_ptr,
                                                               out_port_ptr->common.state,
                                                               &out_port_ptr->common.last_issued_opcode,
                                                               FALSE,
                                                               out_port_ptr->gu.cmn.index,
                                                               out_port_ptr->gu.cmn.id);
      }

      for (wcntr_gu_input_port_list_t *in_port_list_ptr = module_ptr->input_port_list_ptr; (NULL != in_port_list_ptr);
           LIST_ADVANCE(in_port_list_ptr))
      {
         wcntr_topo_input_port_t *in_port_ptr = (wcntr_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;

         if (sg_ops & WCNTR_TOPO_SG_OP_STOP)
         {
            wcntr_topo_reset_input_port(topo_module_ptr->topo_ptr, (void *)in_port_ptr);
         }

         if (in_port_ptr->gu.conn_out_port_ptr)
         {
            if (sg_ops & WCNTR_TOPO_SG_OP_CLOSE)
            {
               if (in_port_ptr->gu.cmn.boundary_port)
               {
                  wcntr_topo_output_port_t *conn_out_port_ptr =
                     (wcntr_topo_output_port_t *)in_port_ptr->gu.conn_out_port_ptr;

                  wcntr_topo_module_t *connected_module_ptr =
                     (wcntr_topo_module_t *)conn_out_port_ptr->gu.cmn.module_ptr;

                  if (connected_module_ptr->capi_ptr)
                  {

                     result =
                        wcntr_topo_capi_set_data_port_op_from_sg_ops(connected_module_ptr,
                                                                     sg_ops,
                                                                     &conn_out_port_ptr->common.last_issued_opcode,
                                                                     FALSE,
                                                                     conn_out_port_ptr->gu.cmn.index,
                                                                     conn_out_port_ptr->gu.cmn.id);
                  }

                  wcntr_topo_destroy_cmn_port(connected_module_ptr,
                                              &conn_out_port_ptr->common,
                                              me_ptr->gu_ptr->log_id,
                                              conn_out_port_ptr->gu.cmn.id,
                                              FALSE);
               }
               in_port_ptr->gu.conn_out_port_ptr->conn_in_port_ptr = NULL;
               in_port_ptr->gu.conn_out_port_ptr                   = NULL;
            }
         }
         in_port_ptr->common.state = self_port_state;
         wcntr_topo_capi_set_data_port_op_from_data_port_state((wcntr_topo_module_t *)in_port_ptr->gu.cmn.module_ptr,
                                                               in_port_ptr->common.state,
                                                               &in_port_ptr->common.last_issued_opcode,
                                                               TRUE,
                                                               in_port_ptr->gu.cmn.index,
                                                               in_port_ptr->gu.cmn.id);
      }

      for (wcntr_gu_ctrl_port_list_t *ctrl_port_list_ptr = module_ptr->ctrl_port_list_ptr; (NULL != ctrl_port_list_ptr);
           LIST_ADVANCE(ctrl_port_list_ptr))
      {
         wcntr_topo_ctrl_port_t *ctrl_port_ptr      = (wcntr_topo_ctrl_port_t *)ctrl_port_list_ptr->ctrl_port_ptr;
         wcntr_topo_ctrl_port_t *peer_ctrl_port_ptr = (wcntr_topo_ctrl_port_t *)ctrl_port_ptr->gu.peer_ctrl_port_ptr;

         if (sg_ops & WCNTR_TOPO_SG_OP_PREPARE)
         {
            result |= wcntr_check_and_recreate_int_ctrl_port_buffers(me_ptr, (wcntr_gu_ctrl_port_t *)ctrl_port_ptr);
         }
         if (sg_ops & (WCNTR_TOPO_SG_OP_CLOSE | WCNTR_TOPO_SG_OP_STOP))
         {
            result |= wcntr_check_and_retrieve_bufq_msgs_from_cmn_ctrlq(me_ptr, (wcntr_gu_ctrl_port_t *)ctrl_port_ptr);
         }

         if (peer_ctrl_port_ptr)
         {
            wcntr_topo_module_t *peer_module_ptr = (wcntr_topo_module_t *)peer_ctrl_port_ptr->gu.module_ptr;
            if (sg_ops & WCNTR_TOPO_SG_OP_CLOSE)
            {
               if (!ctrl_port_ptr->gu.is_peer_port_in_same_sg)
               {
                  if (peer_module_ptr->capi_ptr)
                  {
                     wcntr_topo_set_ctrl_port_operation(&peer_ctrl_port_ptr->gu,
                                                        INTF_EXTN_IMCL_PORT_CLOSE,
                                                        peer_module_ptr->topo_ptr->heap_id);
                  }
               }

               ctrl_port_ptr->gu.peer_ctrl_port_ptr->peer_ctrl_port_ptr = NULL;
               ctrl_port_ptr->gu.peer_ctrl_port_ptr                     = NULL;
            }
         }

         ctrl_port_ptr->state = self_port_state;
      }
   }

   ((wcntr_topo_sg_t *)sg_ptr)->state = sg_state;

   WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_LOW_PRIO,
             "handle_current_sg_mgmt_cmd: sg_id 0x%lX, sg_ops 0x%lX complete with result %u",
             sg_ptr->id,sg_ops,
             result);

   return result;
}

wcntr_topo_port_state_t wcntr_get_downgraded_state(wcntr_topo_port_state_t self_port_state,
                                                   wcntr_topo_port_state_t connected_port_state)
{

   if (self_port_state == WCNTR_TOPO_PORT_STATE_STOPPED || self_port_state == WCNTR_TOPO_PORT_STATE_PREPARED)
   {
      return self_port_state;
   }
   else if (self_port_state == WCNTR_TOPO_PORT_STATE_STARTED)
   {
      return connected_port_state;
   }
   else
   {
      return WCNTR_TOPO_PORT_STATE_INVALID;
   }
}


static ar_result_t wcntr_update_data_port_states(wcntr_base_t *me_ptr, wcntr_gu_sg_t *sg_ptr)
{
   ar_result_t             result          = AR_EOK;
   wcntr_topo_port_state_t self_port_state = wcntr_topo_sg_state_to_port_state(wcntr_topo_get_sg_state(sg_ptr));

   WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_LOW_PRIO,
             "update_data_port_states: for sg_id 0x%lX num_boundary_in_ports %u num_boundary_out_ports %u ",
             sg_ptr->id,
             sg_ptr->num_boundary_in_ports,
             sg_ptr->num_boundary_out_ports);

   // Subgraphs with boundary input only and boundary input and output ports are handled below
   if (sg_ptr->num_boundary_in_ports)
   {
      for (wcntr_gu_input_port_list_t *in_port_list_ptr = sg_ptr->boundary_in_port_list_ptr; (NULL != in_port_list_ptr);
           LIST_ADVANCE(in_port_list_ptr))
      {

         wcntr_topo_input_port_t * in_port_ptr = (wcntr_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;
         wcntr_topo_output_port_t *connected_out_port_ptr =
            (wcntr_topo_output_port_t *)in_port_ptr->gu.conn_out_port_ptr;
         if (connected_out_port_ptr)
         {

         //use subgraph state to know correct state of the port
		 wcntr_gu_sg_t *connected_sg_ptr=  connected_out_port_ptr->gu.cmn.module_ptr->sg_ptr;
		 wcntr_topo_port_state_t connected_port_state = wcntr_topo_sg_state_to_port_state(wcntr_topo_get_sg_state(connected_sg_ptr)); 
			 
            wcntr_topo_port_state_t input_port_state_to_be_propagated =
               wcntr_get_downgraded_state(self_port_state, connected_port_state);

            WCNTR_MSG(me_ptr->gu_ptr->log_id,
                      DBG_HIGH_PRIO,
                      "miid,port_id (0x%lX,0x%lx) self_port_state %X connected_op state %X "
                      "in_port_state_to_be_propagated %X ",
                      in_port_ptr->gu.cmn.module_ptr->module_instance_id,
                      in_port_ptr->gu.cmn.id,
                      self_port_state,
                      connected_port_state,
                      input_port_state_to_be_propagated);

            // Current state is different. Hence propagate
            if (in_port_ptr->common.state != input_port_state_to_be_propagated &&
                input_port_state_to_be_propagated != WCNTR_TOPO_PORT_STATE_INVALID)
            {
               uint32_t recurse_depth = 0;
               result                 = wcntr_topo_propagate_port_state_forwards((void *)me_ptr->topo_ptr,
                                                                 (void *)in_port_ptr,
                                                                 input_port_state_to_be_propagated,
                                                                 &recurse_depth);
            }
         }
         else
         {
            WCNTR_MSG(me_ptr->gu_ptr->log_id,
                      DBG_HIGH_PRIO,
                      "update_data_port_states  "
                      "miid,input_port_id (0x%lX,0x%lx)  "
                      "No connected_out_port ",
                      in_port_ptr->gu.cmn.module_ptr->module_instance_id,
                      in_port_ptr->gu.cmn.id);
         }
      }
      wcntr_update_boundary_out_port_states(me_ptr, sg_ptr, self_port_state);
   }
   else if (sg_ptr->num_boundary_out_ports) // Subgraphs with only boundary output
   {
      wcntr_update_boundary_out_port_states(me_ptr, sg_ptr, self_port_state);
   }
   else // No boundary ports. Single subgraph
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_LOW_PRIO, "update_data_port_states: No boundary ports found");
   }

   WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_LOW_PRIO,
             "update_data_port_states: for sg_id 0x%lX with result 0x%lX ",
             sg_ptr->id,result);

   return result;
}

static void wcntr_update_boundary_out_port_states(wcntr_base_t *          me_ptr,
                                                  wcntr_gu_sg_t *         sg_ptr,
                                                  wcntr_topo_port_state_t self_port_state)
{

   if (sg_ptr->num_boundary_out_ports)
   {
      for (wcntr_gu_output_port_list_t *out_port_list_ptr = sg_ptr->boundary_out_port_list_ptr;
           (NULL != out_port_list_ptr);
           LIST_ADVANCE(out_port_list_ptr))
      {
         wcntr_topo_output_port_t *out_port_ptr          = (wcntr_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
         wcntr_topo_input_port_t * connected_in_port_ptr = (wcntr_topo_input_port_t *)out_port_ptr->gu.conn_in_port_ptr;
         wcntr_topo_port_state_t   output_port_state     = self_port_state;

         if (connected_in_port_ptr)
         {

         wcntr_gu_sg_t *connected_sg_ptr=  connected_in_port_ptr->gu.cmn.module_ptr->sg_ptr;
		 wcntr_topo_port_state_t connected_port_state = wcntr_topo_sg_state_to_port_state(wcntr_topo_get_sg_state(connected_sg_ptr)); 


            wcntr_topo_port_state_t output_port_state_to_be_set =
               wcntr_get_downgraded_state(output_port_state, connected_port_state);

            WCNTR_MSG(me_ptr->gu_ptr->log_id,
                      DBG_HIGH_PRIO,
                      "update_boundary_out_port_states: "
                      "miid,port_id (0x%lX,0x%lx)  output_port_state %X connected_in_port state %X "
                      "output_port_state_to_be_set %X ",
                      out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                      out_port_ptr->gu.cmn.id,output_port_state,
                      connected_port_state,
                      output_port_state_to_be_set);

            if (out_port_ptr->common.state != output_port_state_to_be_set &&
                output_port_state_to_be_set != WCNTR_TOPO_PORT_STATE_INVALID)
            {
               out_port_ptr->common.state = output_port_state_to_be_set;
               wcntr_topo_capi_set_data_port_op_from_data_port_state((wcntr_topo_module_t *)
                                                                        out_port_ptr->gu.cmn.module_ptr,
                                                                     out_port_ptr->common.state,
                                                                     &out_port_ptr->common.last_issued_opcode,
                                                                     FALSE,
                                                                     out_port_ptr->gu.cmn.index,
                                                                     out_port_ptr->gu.cmn.id);
            }

            WCNTR_MSG(me_ptr->gu_ptr->log_id,
                      DBG_HIGH_PRIO,
                      "update_boundary_out_port_states num_boundary_out_ports >0 "
                      "miid,output_port_id (0x%lX,0x%lx)  state %u boundary_port %u ",
                      out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                      out_port_ptr->gu.cmn.id,
                      out_port_ptr->common.state,
                      out_port_ptr->gu.cmn.boundary_port);
         }
         else
         {
            WCNTR_MSG(me_ptr->gu_ptr->log_id,
                      DBG_HIGH_PRIO,
                      "update_boundary_out_port_states  "
                      "miid,port_id (0x%lX,0x%lx)  "
                      "No connected_in_port_ptr ",
                      out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                      out_port_ptr->gu.cmn.id);
         }
      }
   }
   else
   {
   }

   return;
}

/**
 * Minimal recursive state propagation (forwards in graph shape).
 * Simple recursive propagation based on state of boundary input state
 * The current approach works mainly for single-in-multi-out modules
 * State to be propagated from boundary input state would be propagated
 * to all the output ports of that module(same state) until we reach a sink module
 * And propagate further from next modules output port to next_next module's input.
 */
ar_result_t wcntr_topo_propagate_port_state_forwards(void *    vtopo_ptr,
                                                     void *    vin_port_ptr,
                                                     uint32_t  propagated_value,
                                                     uint32_t *recurse_depth_ptr)
{
   ar_result_t              result      = AR_EOK;
   wcntr_topo_t *           topo_ptr    = (wcntr_topo_t *)vtopo_ptr;
   wcntr_topo_input_port_t *in_port_ptr = (wcntr_topo_input_port_t *)vin_port_ptr;

   if (!in_port_ptr || !vtopo_ptr)
   {
      return AR_EOK;
   }
   RECURSION_ERROR_CHECK_ON_FN_ENTRY(topo_ptr->gu.log_id, recurse_depth_ptr, WR_CNTR_PROPAGATION_RECURSE_MAX_DEPTH);

   WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                  DBG_HIGH_PRIO,
                  "wcntr_topo_propagate_port_property_forwards START  propagated_value=0x%x at "
                  "module-id,input_port_id (0x%lX,0x%lx) recurse_depth_ptr %u ",
                  propagated_value,
                  in_port_ptr->gu.cmn.module_ptr->module_instance_id,
                  in_port_ptr->gu.cmn.id,
                  *recurse_depth_ptr);

   // Set next input ports state.
   bool_t   can_propagate_further = FALSE;
   uint32_t before_state          = WCNTR_TOPO_PORT_STATE_INVALID;
   if (in_port_ptr->common.state != propagated_value)
   {
      before_state              = in_port_ptr->common.state;
      in_port_ptr->common.state = propagated_value;

      if (WCNTR_TOPO_PORT_STATE_STOPPED == in_port_ptr->common.state)
      {
         wcntr_topo_reset_input_port(topo_ptr, (void *)in_port_ptr);
      }

      wcntr_topo_capi_set_data_port_op_from_data_port_state((wcntr_topo_module_t *)in_port_ptr->gu.cmn.module_ptr,
                                                            in_port_ptr->common.state,
                                                            &in_port_ptr->common.last_issued_opcode,
                                                            TRUE,
                                                            in_port_ptr->gu.cmn.index,
                                                            in_port_ptr->gu.cmn.id);
      can_propagate_further = TRUE;
   }

   if (!can_propagate_further)
   {
      WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                     DBG_HIGH_PRIO,
                     "wcntr_topo_propagate_port_property_forwards STOP and RETURNING as can_propagate_further is "
                     "FALSE "
                     "propagated_value=0x%x at "
                     "module-id,input_port_id (0x%lX,0x%lx) recurse_depth_ptr %u ",
                     propagated_value,
                     in_port_ptr->gu.cmn.module_ptr->module_instance_id,
                     in_port_ptr->gu.cmn.id,
                     *recurse_depth_ptr);
      return AR_EOK;
   }

   WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                  DBG_HIGH_PRIO,
                  "wcntr_topo_propagate_port_property_forwards  "
                  "propagated_value=0x%x at "
                  "module-id,input_port_id (0x%lX,0x%lx) recurse_depth_ptr %u before_state %u current_state %u ",
                  propagated_value,
                  in_port_ptr->gu.cmn.module_ptr->module_instance_id,
                  in_port_ptr->gu.cmn.id,
                  *recurse_depth_ptr,
                  before_state,
                  in_port_ptr->common.state);

   // Propagate across current module to which input port is connected
   wcntr_topo_module_t *current_module_ptr = (wcntr_topo_module_t *)in_port_ptr->gu.cmn.module_ptr;
   // Sink module. Cannot be propagated further from sinka module which dont have any output ports
   if (current_module_ptr->gu.num_output_ports == 0)
   {
      RECURSION_ERROR_CHECK_ON_FN_EXIT(topo_ptr->gu.log_id, recurse_depth_ptr);

      WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                     DBG_HIGH_PRIO,
                     "wcntr_topo_propagate_port_property_forwards RETURNING as module has ZERO OUTPUT PORTS "
                     "propagated_value=0x%x at "
                     "module-id,input_port_id (0x%lX,0x%lx) recurse_depth_ptr %u ",
                     propagated_value,
                     in_port_ptr->gu.cmn.module_ptr->module_instance_id,
                     in_port_ptr->gu.cmn.id,
                     *recurse_depth_ptr);
      return AR_EOK;
   }

   for (wcntr_gu_output_port_list_t *out_port_list_ptr = current_module_ptr->gu.output_port_list_ptr;
        (NULL != out_port_list_ptr);
        LIST_ADVANCE(out_port_list_ptr))
   {
      wcntr_topo_output_port_t *out_port_ptr          = (wcntr_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
      bool_t                    can_propagate_further = FALSE;
      if (out_port_ptr->common.state != propagated_value)
      {

         out_port_ptr->common.state = propagated_value;

         if (WCNTR_TOPO_PORT_STATE_STOPPED == out_port_ptr->common.state)
         {
            wcntr_topo_reset_output_port(topo_ptr, (void *)out_port_ptr);
         }

         wcntr_topo_capi_set_data_port_op_from_data_port_state((wcntr_topo_module_t *)out_port_ptr->gu.cmn.module_ptr,
                                                               out_port_ptr->common.state,
                                                               &out_port_ptr->common.last_issued_opcode,
                                                               FALSE,
                                                               out_port_ptr->gu.cmn.index,
                                                               out_port_ptr->gu.cmn.id);

         // Need to do data_port operation
         can_propagate_further = TRUE;

         if (out_port_ptr->gu.cmn.boundary_port)
         {
            can_propagate_further                                 = FALSE;
         }
      }

      if (!can_propagate_further)
      {
         WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                        DBG_HIGH_PRIO,
                        "wcntr_topo_propagate_port_property_forwards CONTINUE "
                        "module-id,output_port_id (0x%lX,0x%lx) can_propagate_further %u state %u boundary_port %u",
                        out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                        out_port_ptr->gu.cmn.id,
                        can_propagate_further,
                        out_port_ptr->common.state,
                        out_port_ptr->gu.cmn.boundary_port);

         continue;
      }

      WCNTR_TOPO_MSG(topo_ptr->gu.log_id,
                     DBG_HIGH_PRIO,
                     "wcntr_topo_propagate_port_property_forwards "
                     "module-id,output_port_id (0x%lX,0x%lx) propagated_value %u can_propagate_further %u state %u boundary_port %u ",
                     out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                     out_port_ptr->gu.cmn.id, propagated_value,
                     can_propagate_further,
                     out_port_ptr->common.state,
                     out_port_ptr->gu.cmn.boundary_port);

      {
         wcntr_topo_input_port_t *next_in_port_ptr = (wcntr_topo_input_port_t *)out_port_ptr->gu.conn_in_port_ptr;

         wcntr_topo_propagate_port_state_forwards(vtopo_ptr,
                                                  (void *)next_in_port_ptr,
                                                  propagated_value,
                                                  recurse_depth_ptr);
      }
   }
   RECURSION_ERROR_CHECK_ON_FN_EXIT(topo_ptr->gu.log_id, recurse_depth_ptr);
   return result;
}


static void wcntr_update_module_process_flag(wcntr_base_t *me_ptr, wcntr_gu_sg_t *sg_ptr)
{

   for (wcntr_gu_module_list_t *module_list_ptr = sg_ptr->module_list_ptr; (NULL != module_list_ptr);
        LIST_ADVANCE(module_list_ptr))
   {
      wcntr_topo_module_t *module_ptr                      = (wcntr_topo_module_t *)module_list_ptr->module_ptr;
      bool_t               atleast_one_input_port_started  = FALSE;
      bool_t               atleast_one_output_port_started = FALSE;
      bool_t               can_process_be_called           = FALSE;
      uint32_t             input_ports_created             = 0;
      uint32_t             output_ports_created            = 0;

	  if(MODULE_ID_RD_SHARED_MEM_EP == module_ptr->gu.module_id)
	  {
	   module_ptr->can_process_be_called = FALSE;
	   continue;
	  }
	  

      if (((wcntr_topo_sg_t *)sg_ptr)->state != WCNTR_TOPO_SG_STATE_STARTED)
      {
         module_ptr->can_process_be_called = FALSE;
         continue;
      }

      for (wcntr_gu_input_port_list_t *in_port_list_ptr = module_ptr->gu.input_port_list_ptr;
           (NULL != in_port_list_ptr);
           LIST_ADVANCE(in_port_list_ptr))
      {
         wcntr_topo_input_port_t *in_port_ptr = (wcntr_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;
         input_ports_created++;
         if (in_port_ptr->common.state == WCNTR_TOPO_PORT_STATE_STARTED)
         {
            atleast_one_input_port_started = TRUE;
            break;
         }
      }

      for (wcntr_gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr;
           (NULL != out_port_list_ptr);
           LIST_ADVANCE(out_port_list_ptr))
      {
         wcntr_topo_output_port_t *out_port_ptr = (wcntr_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
         // Ports are created only during open command and removed during close command
         output_ports_created++;
         if (out_port_ptr->common.state == WCNTR_TOPO_PORT_STATE_STARTED)
         {
            atleast_one_output_port_started = TRUE;
            break;
         }
      }

      if (input_ports_created && output_ports_created)
      {
         can_process_be_called = atleast_one_input_port_started & atleast_one_output_port_started;
      }
      else if (input_ports_created)
      {
         can_process_be_called = atleast_one_input_port_started;

         // Module is not sink module.
         // Connected subgraph might not be opened.
         // So num_output_ports will be 0
         if (module_ptr->gu.max_output_ports)
         {
            can_process_be_called = FALSE;
         }
      }
      else if (output_ports_created)
      {
         can_process_be_called = atleast_one_output_port_started;
      }
      else
      {
      }

      module_ptr->can_process_be_called = can_process_be_called;

      WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_LOW_PRIO,
                "update_module_process_flag sg_id 0x%lX ,mid 0x%lX, "
                "atleast_one_input_port_started %u,atleast_one_output_port_started %u,"
                "can_process_be_called %u ",
                sg_ptr->id,
                module_ptr->gu.module_instance_id,
                atleast_one_input_port_started,
                atleast_one_output_port_started,
                module_ptr->can_process_be_called);
   }

   return;
}

bool_t wcntr_cmd_graph_mgmt_contains_ctrl_only(spf_msg_cmd_graph_mgmt_t *cmd_gmgmt_ptr)
{
   if (!cmd_gmgmt_ptr)
   {
      return FALSE;
   }

   spf_cntr_sub_graph_list_t *sg_list_ptr = &cmd_gmgmt_ptr->sg_id_list;

   return (0 == (cmd_gmgmt_ptr->cntr_port_hdl_list.num_ip_port_handle +
                 cmd_gmgmt_ptr->cntr_port_hdl_list.num_op_port_handle + sg_list_ptr->num_sub_graph));
}

static bool_t wcntr_is_atleast_one_sg_started(wcntr_base_t *me_ptr)
{
   for (wcntr_gu_sg_list_t *sg_list_ptr = me_ptr->gu_ptr->sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      wcntr_gu_sg_t *sg_ptr = (wcntr_gu_sg_t *)sg_list_ptr->sg_ptr;
      if (WCNTR_TOPO_SG_STATE_STARTED == wcntr_topo_get_sg_state(sg_ptr))
      {
         return TRUE;
      }
   }
   return FALSE;
}

/**
 * handles events after commands.
 * typically required for set-cfg, but also in other scenarios:
 *  - A SG is opened and running and another new one joins. new module raises threshold event.
 *    If we don't handle right after command, then after some data processing, events might be handled causing data
 *       drops (threshold/MF prop can cause data drop).
 */
static ar_result_t wcntr_handle_events_after_cmds(wcntr_t *me_ptr)
{
   ar_result_t   result   = AR_EOK;
   wcntr_base_t *base_ptr = &me_ptr->cu;

   /** no need to handle event when handle rest is pending and some events occur when container is not started. */
   if (!wcntr_is_any_handle_rest_pending(base_ptr) &&
       (me_ptr->cu.fwk_evt_flags.sg_state_change ||
        (me_ptr->cu.flags.is_cntr_started && (me_ptr->cu.fwk_evt_flags.word || me_ptr->topo.capi_event_flag.word))))
   {
      wcntr_handle_fwk_events(me_ptr);
   }

   return result;
}

/**
 * For external ports, gen_cntr_operate_on_ext_in_port & gen_cntr_operate_on_ext_out_port is called
 * in 2 contexts:
 * 1. in the context of subgraph command: both ends of the connection belongs to the same SG.
 * 2. in the context of handle list of subgraph: this is an inter-SG connection.
 *
 * the distinction is made in the caller.
 * This is a common utility.
 *
 */
static void wcntr_handle_failure_at_graph_open(wcntr_t *                 me_ptr,
                                               spf_msg_cmd_graph_open_t *open_cmd_ptr,
                                               ar_result_t               result)
{
   /** destroy subgraphs one by one */
   for (uint32_t i = 0; i < open_cmd_ptr->num_sub_graphs; i++)
   {
      typedef struct cmd_graph_mgmt_t
      {
         spf_msg_cmd_graph_mgmt_t c;
         uint32_t                 sg_id;
      } cmd_graph_mgmt_t;
      cmd_graph_mgmt_t cmd_mgmt;

      memset(&cmd_mgmt, 0, sizeof(cmd_graph_mgmt_t));
      cmd_mgmt.c.sg_id_list.num_sub_graph  = 1;
      cmd_mgmt.c.sg_id_list.sg_id_list_ptr = &cmd_mgmt.sg_id;
      cmd_mgmt.sg_id                       = open_cmd_ptr->sg_cfg_list_pptr[i]->sub_graph_id;

      wcntr_deinit_internal_ctrl_ports(&me_ptr->cu, &cmd_mgmt.c);

      wcntr_topo_destroy_modules(&me_ptr->topo, &cmd_mgmt.c.sg_id_list);

      // if this container has a port connected to another SG, then the port alone might be destroyed in this cmd.
      // Also, very important to call wcntr_topo_destroy_modules before wcntr_deinit_external_ports.
      // This ensures gpr dereg happens before ext port is deinit'ed. If not, crash may occur if HLOS pushes buf when we
      // deinit.
      //    wcntr_deinit_external_ports(&me_ptr->cu, &cmd_mgmt.c);

      wcntr_gu_destroy_graph(me_ptr->cu.gu_ptr, &cmd_mgmt.c, me_ptr->cu.heap_id);
   }

   spf_msg_ack_msg(&me_ptr->cu.cmd_msg, result);
}

/**
 * context ctx_ptr is not used.
 *
 * This function is entered in below ways:
 * 1. directly from wcntr_graph_open
 * 2. from thread relaunch originating from wcntr_graph_open
 * 3. from gen_cntr_handle_rest_of_wcntr_set_cfg_after_real_module_cfg, which in turn originates from thread relaunch
 * due
 *    to set-cfg containing real-module-id of a placeholder module (in graph open).
 */
ar_result_t wcntr_handle_rest_of_graph_open(wcntr_base_t *base_ptr, void *ctx_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   wcntr_t *                 me_ptr = (wcntr_t *)base_ptr;
   spf_msg_cmd_graph_open_t *open_cmd_ptr;

   spf_msg_header_t *header_ptr = (spf_msg_header_t *)me_ptr->cu.cmd_msg.payload_ptr;
   open_cmd_ptr                 = (spf_msg_cmd_graph_open_t *)&header_ptr->payload_start;

   bool_t pm_already_registered = posal_power_mgr_is_registered(me_ptr->cu.pm_info.pm_handle_ptr);

   // do not clear handle_rest because below func needs it to know where it left off in the set-cfg payload.
   TRY(result,
       wcntr_set_get_cfgs_fragmented(base_ptr,
                                     (apm_module_param_data_t **)open_cmd_ptr->param_data_pptr,
                                     open_cmd_ptr->num_param_id_cfg,
                                     TRUE,                        /*is_wcntr_set_cfg*/
                                     FALSE,                       /*is_deregister (Dont care in this case)*/
                                     SPF_CFG_DATA_TYPE_DEFAULT)); // no module will be in start state during open

   // real-module-id present in the set-cfg payload could recreate thread. after relaunching the thread,
   // gen_cntr_handle_rest_of_wcntr_set_cfg_after_real_module_cfg calls this func

   if (wcntr_is_any_handle_rest_pending(base_ptr))
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_HIGH_PRIO,
                "CMD:GRAPH_OPEN: handle_rest fn set during set-get-cfg of graph open");
      return result;
   }

   // in success case, ack to APM is sent from this func.
   TRY(result, wcntr_gu_respond_to_graph_open(me_ptr->cu.gu_ptr, &me_ptr->cu.cmd_msg, me_ptr->cu.heap_id));

   if (!pm_already_registered)
   {
      // Register container with pm
      wcntr_register_with_pm(&me_ptr->cu);
   }

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
      if (pm_already_registered)
      {
         wcntr_deregister_with_pm(&me_ptr->cu);
      }

      // handle rest of graph-open is null, hence this handle rest must be set-cfg's
      if (base_ptr->handle_rest_ctx_ptr)
      {
         wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *set_get_ptr =
            (wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *)base_ptr->handle_rest_ctx_ptr;
         result |= set_get_ptr->overall_result;
      }

      wcntr_handle_failure_at_graph_open(me_ptr, open_cmd_ptr, result);

      wcntr_reset_handle_rest(base_ptr);
   }

   wcntr_handle_events_after_cmds(me_ptr);

   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_HIGH_PRIO,
             "CMD:GRAPH_OPEN: Done executing graph open command. current channel mask=0x%x. result=0x%lx.",
             me_ptr->cu.curr_chan_mask,
             result);

   return result;
}
ar_result_t wcntr_gpr_cmd(wcntr_base_t *base_ptr)
{
   ar_result_t result = AR_EOK;
   wcntr_t *   me_ptr = (wcntr_t *)base_ptr;
   INIT_EXCEPTION_HANDLING
   gpr_packet_t *packet_ptr = (gpr_packet_t *)me_ptr->cu.cmd_msg.payload_ptr;

   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_HIGH_PRIO,
             "CMD:GPR cmd: Executing GPR command, opCode(%lX) token(%lx), handle_rest %u",
             packet_ptr->opcode,
             packet_ptr->token,
             wcntr_is_any_handle_rest_pending(base_ptr));

   switch (packet_ptr->opcode)
   {
      case APM_CMD_SET_CFG:
      case APM_CMD_GET_CFG:
      {
         wcntr_set_get_cfgs_packed(base_ptr, packet_ptr, SPF_CFG_DATA_TYPE_DEFAULT);
         break;
      }

      case APM_CMD_REGISTER_CFG:
      case APM_CMD_DEREGISTER_CFG:
      {
         wcntr_set_get_cfgs_packed(base_ptr, packet_ptr, SPF_CFG_DATA_PERSISTENT);
         break;
      }

      case APM_CMD_REGISTER_SHARED_CFG:
      case APM_CMD_DEREGISTER_SHARED_CFG:
      {
         wcntr_set_get_cfgs_packed(base_ptr, packet_ptr, SPF_CFG_DATA_SHARED_PERSISTENT);
         break;
      }
      case APM_CMD_REGISTER_MODULE_EVENTS:
      {
         result = wcntr_register_module_events(base_ptr, packet_ptr);
         if (AR_EOK != result)
         {
            WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "Failed to register events with modules");
         }
         __gpr_cmd_end_command(packet_ptr, result);

         break;
      }
      default:
      {
         WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "CMD:GPR cmd: unsupported GPR opcode", packet_ptr->opcode);

         // TRY(result, cu_gpr_cmd(base_ptr));
         break;
      }
   }

   TRY(result, wcntr_handle_events_after_cmds(me_ptr));

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   if (!wcntr_is_any_handle_rest_pending(base_ptr) || AR_DID_FAIL(result))
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_HIGH_PRIO,
                "CMD:GPR cmd: Done executing GPR command, result=0x%lx, handle_rest %u",
                result,
                wcntr_is_any_handle_rest_pending(base_ptr));

      if (base_ptr->handle_rest_ctx_ptr)
      {
         wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *set_get_ptr =
            (wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *)base_ptr->handle_rest_ctx_ptr;
         result |= set_get_ptr->overall_result;
      }

      wcntr_reset_handle_rest(base_ptr);
   }

   return result;
}

ar_result_t wcntr_graph_open(wcntr_base_t *base_ptr)
{
   ar_result_t result = AR_EOK;
   wcntr_t *   me_ptr = (wcntr_t *)base_ptr;

   INIT_EXCEPTION_HANDLING
   wcntr_topo_graph_init_t   graph_init_data = { 0 };
   spf_msg_cmd_graph_open_t *open_cmd_ptr    = NULL;
   uint32_t                  stack_size      = 0;
   posal_thread_prio_t       thread_priority = 0;
   char_t                    thread_name[POSAL_DEFAULT_NAME_LEN];
   bool_t                    thread_launched = FALSE;

   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_HIGH_PRIO,
             "CMD:GRAPH_OPEN: Executing graph open command. current channel mask=0x%x",
             me_ptr->cu.curr_chan_mask);

   spf_msg_header_t *header_ptr = (spf_msg_header_t *)me_ptr->cu.cmd_msg.payload_ptr;

   if (header_ptr->payload_size < sizeof(spf_msg_cmd_graph_open_t))
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_ERROR_PRIO,
                "CMD:GRAPH_OPEN: Wrong Payload size, Req = %lu, Given = %lu",
                sizeof(spf_msg_cmd_graph_open_t),
                header_ptr->payload_size);
      return AR_EBADPARAM;
   }

   open_cmd_ptr = (spf_msg_cmd_graph_open_t *)&header_ptr->payload_start;

   TRY(result, wcntr_parse_container_cfg(me_ptr, open_cmd_ptr->container_cfg_ptr));

   {
      wcntr_gu_sizes_t sz = {

         .in_port_size   = sizeof(wcntr_topo_input_port_t),
         .out_port_size  = sizeof(wcntr_topo_output_port_t),
         .ctrl_port_size = WNTR_INT_CTRL_PORT_SIZE_W_QS,
         .sg_size        = sizeof(wcntr_topo_sg_t),
         .module_size    = sizeof(wcntr_module_t)
      };

      TRY(result, wcntr_gu_create_graph(&me_ptr->topo.gu, open_cmd_ptr, &sz, me_ptr->cu.heap_id));
      me_ptr->cu.gu_ptr = &me_ptr->topo.gu;
      // wcntr_gu_print_graph(&me_ptr->topo.gu);
   }

   graph_init_data.spf_handle_ptr = &me_ptr->cu.spf_handle;
   graph_init_data.gpr_cb_fn      = wcntr_gpr_callback;
   graph_init_data.capi_cb        = wcntr_topo_capi_callback;

   
   TRY(result, wcntr_init_internal_ctrl_ports(&me_ptr->cu));

   TRY(result, wcntr_topo_create_modules(&me_ptr->topo, &graph_init_data));
   stack_size = graph_init_data.max_stack_size;

   //  TRY(result, wcntr_init_external_ports(&me_ptr->cu));

   // to update the trigger policy related flags.
   wcntr_topo_reset_top_level_flags(&me_ptr->topo);

   TRY(result,
       wcntr_prepare_to_launch_thread(me_ptr, &stack_size, &thread_priority, thread_name, POSAL_DEFAULT_NAME_LEN));

   TRY(result, wcntr_check_launch_thread(&me_ptr->cu, stack_size, thread_priority, thread_name, &thread_launched));

   if (thread_launched)
   {
      me_ptr->cu.handle_rest_fn      = wcntr_handle_rest_of_graph_open;
      me_ptr->cu.handle_rest_ctx_ptr = NULL;
      // do set cfg etc later as thread has re-launched.

      return AR_EOK;
   }

   // clean-up is handled in handle_rest func.
   return wcntr_handle_rest_of_graph_open(&me_ptr->cu, NULL /*ctx_ptr*/);

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
      wcntr_handle_failure_at_graph_open(me_ptr, open_cmd_ptr, result);
   }

   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_HIGH_PRIO,
             "CMD:GRAPH_OPEN: Done executing graph open command. current channel mask=0x%x. result=0x%lx.",
             me_ptr->cu.curr_chan_mask,
             result);

   return result;
}

ar_result_t wcntr_set_get_cfg(wcntr_base_t *base_ptr)
{
   ar_result_t result = AR_EOK;
   wcntr_t *   me_ptr = (wcntr_t *)base_ptr;
   INIT_EXCEPTION_HANDLING
   spf_msg_cmd_param_data_cfg_t *cfg_cmd_ptr;
   bool_t                        is_wcntr_set_cfg_msg;
   bool_t                        is_deregister;
   spf_cfg_data_type_t           data_type;

   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_HIGH_PRIO,
             "CMD:SET_GET_CFG: Executing command opcode=0x%lX, current channel mask=0x%x, handle_rest %u",
             me_ptr->cu.cmd_msg.msg_opcode,
             me_ptr->cu.curr_chan_mask,
             wcntr_is_any_handle_rest_pending(base_ptr));

   spf_msg_header_t *header_ptr = (spf_msg_header_t *)me_ptr->cu.cmd_msg.payload_ptr;
   VERIFY(result, header_ptr->payload_size >= sizeof(spf_msg_cmd_param_data_cfg_t));

   cfg_cmd_ptr = (spf_msg_cmd_param_data_cfg_t *)&header_ptr->payload_start;

   switch (me_ptr->cu.cmd_msg.msg_opcode)
   {
      case SPF_MSG_CMD_REGISTER_CFG:
      {
         data_type            = SPF_CFG_DATA_PERSISTENT;
         is_deregister        = FALSE;
         is_wcntr_set_cfg_msg = TRUE;
         break;
      }
      case SPF_MSG_CMD_DEREGISTER_CFG:
      {
         data_type            = SPF_CFG_DATA_PERSISTENT;
         is_deregister        = TRUE;
         is_wcntr_set_cfg_msg = FALSE;
         break;
      }
      case SPF_MSG_CMD_SET_CFG:
      {
         data_type            = SPF_CFG_DATA_TYPE_DEFAULT;
         is_deregister        = FALSE;
         is_wcntr_set_cfg_msg = TRUE;
         break;
      }
      case SPF_MSG_CMD_GET_CFG:
      {
         data_type            = SPF_CFG_DATA_TYPE_DEFAULT;
         is_deregister        = FALSE;
         is_wcntr_set_cfg_msg = FALSE;
         break;
      }
      default:
      {
         THROW(result, AR_EUNSUPPORTED);
         break;
      }
   }
   TRY(result,
       wcntr_set_get_cfgs_fragmented(base_ptr,
                                     (apm_module_param_data_t **)cfg_cmd_ptr->param_data_pptr,
                                     cfg_cmd_ptr->num_param_id_cfg,
                                     is_wcntr_set_cfg_msg,
                                     is_deregister,
                                     data_type));

   TRY(result, wcntr_handle_events_after_cmds(me_ptr));

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   if (!wcntr_is_any_handle_rest_pending(base_ptr) || AR_DID_FAIL(result))
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_HIGH_PRIO,
                "CMD:SET_GET_CFG: Done executing command opcode=0x%lX, current channel mask=0x%x. result=0x%lx, "
                "handle_rest %u",
                me_ptr->cu.cmd_msg.msg_opcode,
                me_ptr->cu.curr_chan_mask,
                result,
                wcntr_is_any_handle_rest_pending(base_ptr));

      if (base_ptr->handle_rest_ctx_ptr)
      {
         wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *set_get_ptr =
            (wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *)base_ptr->handle_rest_ctx_ptr;
         result |= set_get_ptr->overall_result;
      }

      wcntr_reset_handle_rest(base_ptr);

      return spf_msg_ack_msg(&me_ptr->cu.cmd_msg, result);
   }
   return result;
}

ar_result_t wcntr_graph_prepare(wcntr_base_t *base_ptr)
{
   ar_result_t result = AR_EOK;
   wcntr_t *   me_ptr = (wcntr_t *)base_ptr;
   INIT_EXCEPTION_HANDLING
   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_HIGH_PRIO, "CMD:Prepare Graph: Entering prepare graph");
   spf_msg_cmd_graph_mgmt_t *cmd_gmgmt_ptr;
   spf_msg_header_t *        header_ptr = (spf_msg_header_t *)me_ptr->cu.cmd_msg.payload_ptr;
   cmd_gmgmt_ptr                        = (spf_msg_cmd_graph_mgmt_t *)&header_ptr->payload_start;

   TRY(result, wcntr_handle_sg_mgmt_cmd(base_ptr, WCNTR_TOPO_SG_OP_PREPARE, WCNTR_TOPO_SG_STATE_PREPARED));

   // APM issues prepare if graph is started without prepare. Hence we dont need to take care preparing if start is
   // issued without prepare.
   // need to handle last because of dependency on state (for media fmt prop).

  TRY(result, wcntr_handle_prepare(&me_ptr->cu, cmd_gmgmt_ptr));

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   wcntr_handle_events_after_cmds(me_ptr);

   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_HIGH_PRIO,
             "CMD:Prepare Graph: Done executing prepare graph command., current channel mask=0x%x. result=0x%lx",
             me_ptr->cu.curr_chan_mask,
             result);

   return spf_msg_ack_msg(&me_ptr->cu.cmd_msg, result);
}

ar_result_t wcntr_graph_start(wcntr_base_t *base_ptr)
{
   ar_result_t result = AR_EOK;
   wcntr_t *   me_ptr = (wcntr_t *)base_ptr;

   INIT_EXCEPTION_HANDLING
   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_HIGH_PRIO,
             "CMD:START:Executing start command. current channel mask=0x%x",
             me_ptr->cu.curr_chan_mask);

   TRY(result, wcntr_handle_sg_mgmt_cmd(base_ptr, WCNTR_TOPO_SG_OP_START, WCNTR_TOPO_SG_STATE_STARTED));

   wcntr_update_cntr_kpps_bw(me_ptr);

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   wcntr_handle_events_after_cmds(me_ptr);

   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_HIGH_PRIO,
             "CMD:START:Done executing start Command. current channel mask=0x%x. result=0x%lx.",
             me_ptr->cu.curr_chan_mask,
             result);

   return spf_msg_ack_msg(&me_ptr->cu.cmd_msg, result);
}

ar_result_t wcntr_graph_suspend(wcntr_base_t *base_ptr)
{

   wcntr_t *me_ptr = (wcntr_t *)base_ptr;
   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "graph_suspend not supported");

   return spf_msg_ack_msg(&me_ptr->cu.cmd_msg, AR_EOK);
}

ar_result_t wcntr_graph_stop(wcntr_base_t *base_ptr)
{
   ar_result_t result = AR_EOK;
   wcntr_t *   me_ptr = (wcntr_t *)base_ptr;

   INIT_EXCEPTION_HANDLING
   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_HIGH_PRIO,
             "CMD:STOP: Executing stop Command. current channel mask=0x%x",
             me_ptr->cu.curr_chan_mask);

   TRY(result, wcntr_handle_sg_mgmt_cmd(base_ptr, WCNTR_TOPO_SG_OP_STOP, WCNTR_TOPO_SG_STATE_STOPPED));

   wcntr_update_cntr_kpps_bw(me_ptr);

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   wcntr_handle_events_after_cmds(me_ptr);

   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_HIGH_PRIO,
             "CMD:STOP:Done Executing stop command. current channel mask=0x%x. result=0x%lx.",
             me_ptr->cu.curr_chan_mask,
             result);

   spf_msg_ack_msg(&me_ptr->cu.cmd_msg, result);

   return result;
}

ar_result_t wcntr_graph_flush(wcntr_base_t *base_ptr)
{
   wcntr_t *me_ptr = (wcntr_t *)base_ptr;
   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "graph_flush not supported");

   return spf_msg_ack_msg(&me_ptr->cu.cmd_msg, AR_EOK);
}

/**
 * APM follows these steps for close: stop, disconnect, close.
 * Close comes for external port disconnect (E.g. AFE client going away).
 */
ar_result_t wcntr_graph_close(wcntr_base_t *base_ptr)
{
   ar_result_t result = AR_EOK;
   wcntr_t *   me_ptr = (wcntr_t *)base_ptr;

   INIT_EXCEPTION_HANDLING
   spf_msg_t                  cmd_msg        = me_ptr->cu.cmd_msg;
   uint32_t                   log_id         = me_ptr->topo.gu.log_id;
   bool_t                     ret_terminated = FALSE;
   spf_cntr_sub_graph_list_t *sg_list_ptr;
   spf_msg_cmd_graph_mgmt_t * cmd_gmgmt_ptr;

   spf_msg_header_t *header_ptr = (spf_msg_header_t *)me_ptr->cu.cmd_msg.payload_ptr;
   VERIFY(result, header_ptr->payload_size >= sizeof(spf_msg_cmd_graph_mgmt_t));

   cmd_gmgmt_ptr = (spf_msg_cmd_graph_mgmt_t *)&header_ptr->payload_start;

   sg_list_ptr = &cmd_gmgmt_ptr->sg_id_list;

   WCNTR_MSG(me_ptr->topo.gu.log_id,
             DBG_HIGH_PRIO,
             "CMD:CLOSE:Executing close Command, current channel mask=0x%x",
             me_ptr->cu.curr_chan_mask);

   wcntr_deinit_internal_ctrl_ports(&me_ptr->cu, cmd_gmgmt_ptr);

   result = wcntr_handle_sg_mgmt_cmd(base_ptr, WCNTR_TOPO_SG_OP_CLOSE, WCNTR_TOPO_SG_STATE_STOPPED);

   // module destroy happens only with SG destroy
   wcntr_topo_destroy_modules(&me_ptr->topo, sg_list_ptr);

   wcntr_gu_destroy_graph(me_ptr->cu.gu_ptr, cmd_gmgmt_ptr, me_ptr->cu.heap_id);

   // wcntr_topo_reset_top_level_flags(&me_ptr->topo);

   if (me_ptr->topo.gu.num_subgraphs)
   {
      // if part of graph goes away, then LCM threshold might change. We can skip this if the cmd is only for control
      // ports.
      if (!wcntr_cmd_graph_mgmt_contains_ctrl_only(cmd_gmgmt_ptr))
      {
         me_ptr->topo.capi_event_flag.port_thresh = TRUE;
      }
   }
   else // check if any subgraph is pending, if not, destroy this container
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_HIGH_PRIO,
                SPF_LOG_PREFIX "As number of subgraphs is zero, destroying this container");
      wcntr_destroy(me_ptr);
      me_ptr         = NULL;
      ret_terminated = TRUE;
   }

   // Catch here so that we don't catch on AR_ETERMINATED.
   CATCH(result, WCNTR_MSG_PREFIX, log_id)
   {
   }

   if (!ret_terminated)
   {
      wcntr_handle_events_after_cmds(me_ptr);
   }

   if (ret_terminated && AR_EOK == result)
   {
      result = AR_ETERMINATED;
   }
   WCNTR_MSG(log_id,
             DBG_HIGH_PRIO,
             "CMD:CLOSE:Done executing close command, current channel mask=0x%x. result=0x%lx.",
             me_ptr ? me_ptr->cu.curr_chan_mask : 0,
             result);

   spf_msg_ack_msg(&cmd_msg, result); // don't overwrite result as it might be AR_ETERMINATED

   return result;
}

ar_result_t wcntr_graph_connect(wcntr_base_t *base_ptr)
{
   wcntr_t *me_ptr = (wcntr_t *)base_ptr;
   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "graph_connect not supported");
   return spf_msg_ack_msg(&me_ptr->cu.cmd_msg, AR_EOK);
}

/**
 * As part of graph close command, APM sends disconnect first
 * this helps all containers stop accessing peer ports.
 */
ar_result_t wcntr_graph_disconnect(wcntr_base_t *base_ptr)
{
   wcntr_t *me_ptr = (wcntr_t *)base_ptr;
   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "graph_disconnect not supported");
   return spf_msg_ack_msg(&me_ptr->cu.cmd_msg, AR_EOK);
}

/**
 * this is no longer used.
 */
ar_result_t wcntr_destroy_container(wcntr_base_t *base_ptr)
{
   ar_result_t result = AR_EOK;
   wcntr_t *   me_ptr = (wcntr_t *)base_ptr;
   uint32_t    log_id = me_ptr->topo.gu.log_id;

   WCNTR_MSG(log_id,
             DBG_HIGH_PRIO,
             "CMD:DESTROY:destroy received. current channel mask=0x%x",
             me_ptr->cu.curr_chan_mask);

   spf_msg_t cmd_msg = me_ptr->cu.cmd_msg;

   result = wcntr_destroy(me_ptr);

   spf_msg_ack_msg(&cmd_msg, result);

   WCNTR_MSG(log_id,
             DBG_HIGH_PRIO,
             "CMD:DESTROY:Done destroy with current channel mask=0x%x. result=0x%lx.",
             me_ptr->cu.curr_chan_mask,
             result);

   // send AR_ETERMINATED so calling routine knows the destroyer has been invoked.
   return AR_ETERMINATED;
}

ar_result_t wcntr_ctrl_path_media_fmt_cmd(wcntr_base_t *base_ptr)
{
   wcntr_t *me_ptr = (wcntr_t *)base_ptr;
   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "ctrl_path_media_fmt_cmd not supported");
   return spf_msg_ack_msg(&me_ptr->cu.cmd_msg, AR_EOK);
}

ar_result_t wcntr_cmd_icb_info_from_downstream(wcntr_base_t *base_ptr)
{
   wcntr_t *me_ptr = (wcntr_t *)base_ptr;
   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "icb_info_from_downstream not supported");
   return spf_msg_ack_msg(&me_ptr->cu.cmd_msg, AR_EOK);
}

ar_result_t wcntr_handle_ctrl_port_trigger_cmd(wcntr_base_t *base_ptr)
{
   ar_result_t result = AR_EOK;

   wcntr_base_t *me_ptr = (wcntr_base_t *)base_ptr;

   // Get the ctrl port handle from the message header.
   spf_msg_header_t *       header_ptr    = (spf_msg_header_t *)me_ptr->cmd_msg.payload_ptr;
   spf_msg_ctrl_port_msg_t *ctrl_msg_ptr  = (spf_msg_ctrl_port_msg_t *)&header_ptr->payload_start;
   wcntr_gu_ctrl_port_t *   ctrl_port_ptr = NULL;

   if (ctrl_msg_ptr->is_intra_cntr)
   {
      ctrl_port_ptr = (wcntr_gu_ctrl_port_t *)ctrl_msg_ptr->dst_intra_cntr_port_hdl;
   }
   else
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "handle_ctrl_port_trigger_cmd Only intra container supported. Returning EOK");
      return AR_EOK;
   }

   wcntr_topo_ctrl_port_t *topo_ctrl_port_ptr = (wcntr_topo_ctrl_port_t *)ctrl_port_ptr;

   if (topo_ctrl_port_ptr->state != WCNTR_TOPO_SG_OP_START)
   {

      // Send the incoming intent to the capi module.
      result = wcntr_topo_handle_incoming_ctrl_intent(ctrl_port_ptr,
                                                      &ctrl_msg_ptr->data_buf[0],
                                                      ctrl_msg_ptr->max_size,
                                                      ctrl_msg_ptr->actual_size);
   }
   else
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "miid,port_id (0x%lX,0x%lX) not in start state.Returning EOK",
                ctrl_port_ptr->module_ptr->module_instance_id,
                ctrl_port_ptr->id);
      return AR_EOK;
   }
   if (AR_DID_FAIL(result))
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_HIGH_PRIO,
                "CMD:CTRL_PORT_TRIGGER: Handling incoming control message failed.");
   }
   else
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_LOW_PRIO,
                "CMD:CTRL_PORT_TRIGGER: mid: 0x%x ctrl port id 0x%lx Set incoming ctrl message on done.",
                ctrl_port_ptr->module_ptr->module_instance_id,
                ctrl_port_ptr->id);
   }

   return spf_msg_return_msg(&me_ptr->cmd_msg);
}

/* Handles Peer port property update command in WCNTR contianer.*/
ar_result_t wcntr_handle_peer_port_property_update_cmd(wcntr_base_t *base_ptr)
{
   wcntr_t *me_ptr = (wcntr_t *)base_ptr;
   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "handle_peer_port_property_update_cmd not supported");

   return spf_msg_ack_msg(&me_ptr->cu.cmd_msg, AR_EOK);
}

ar_result_t wcntr_handle_upstream_stop_cmd(wcntr_base_t *base_ptr)
{
   wcntr_t *me_ptr = (wcntr_t *)base_ptr;
   WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "handle_upstream_stop_cmd not supported");

   return spf_msg_ack_msg(&me_ptr->cu.cmd_msg, AR_EOK);
}

ar_result_t wcntr_set_get_cfg_util(wcntr_base_t *      base_ptr,
                                   void *              mod_ptr,
                                   uint32_t            pid,
                                   int8_t *            param_payload_ptr,
                                   uint32_t *          param_size_ptr,
                                   uint32_t *          error_code_ptr,
                                   bool_t              is_wcntr_set_cfg,
                                   bool_t              is_deregister,
                                   spf_cfg_data_type_t cfg_type)
{
   ar_result_t result = AR_EOK;
   wcntr_t *me_ptr = (wcntr_t *)base_ptr;

   wcntr_topo_module_t *module_ptr = (wcntr_topo_module_t *)mod_ptr;

   if (NULL == module_ptr)
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "gen_cntr_wcntr_set_cfg_util: Module not found");
      return AR_EUNSUPPORTED;
   }

   // For placeholder module, once a real module is replaced, all params except RESET_PLACEHOLDER go to CAPI
   if (module_ptr->capi_ptr && (pid != PARAM_ID_RESET_PLACEHOLDER_MODULE))
   {
      if (is_wcntr_set_cfg)
      {
         // If this is a low power container, we will not inform the module about persistence, so module will treat it
         // as non-persistent set-param
         if (((SPF_CFG_DATA_PERSISTENT == cfg_type) || (SPF_CFG_DATA_SHARED_PERSISTENT == cfg_type)) &&
             (FALSE == POSAL_IS_ISLAND_HEAP_ID(me_ptr->cu.heap_id)))
         {
            wcntr_topo_capi_set_persistence_prop(me_ptr->topo.gu.log_id, module_ptr, pid, is_deregister, cfg_type);
         }

         // when deregistering a persistent payload, we shouldn't call set param
         if (!is_deregister)
         {
            result |= wcntr_topo_capi_set_param(me_ptr->topo.gu.log_id,
                                                module_ptr->capi_ptr,
                                                pid,
                                                param_payload_ptr,
                                                *param_size_ptr);
         }
      }
      else /* GET_CFG */
      {
         result |= wcntr_topo_capi_get_param(me_ptr->topo.gu.log_id,
                                             module_ptr->capi_ptr,
                                             pid,
                                             param_payload_ptr,
                                             param_size_ptr);
      }
   }
   else if(MODULE_ID_RD_SHARED_MEM_EP == module_ptr->gu.module_id)
   {
	   WCNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_HIGH_PRIO,
                " set param for MODULE_ID_RD_SHARED_MEM_EP Setting result to EOK ");
      result = AR_EOK;   
   }
   else
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id,
                DBG_ERROR_PRIO,
                " Unsupported framework module-id 0x%lX",
                module_ptr->gu.module_id);
      result = AR_EUNSUPPORTED;
   }

   /*Return Error code */
   *error_code_ptr = result;

   return result;
}

ar_result_t wcntr_register_events_utils(wcntr_base_t *          base_ptr,
                                        wcntr_gu_module_t *     gu_module_ptr,
                                        wcntr_topo_reg_event_t *reg_event_payload_ptr,
                                        bool_t                  is_register,
                                        bool_t *                capi_supports_v1_event_ptr)
{
   ar_result_t result = AR_EOK;
   wcntr_t *me_ptr = (wcntr_t *)base_ptr;

   wcntr_topo_module_t *module_ptr = (wcntr_topo_module_t *)gu_module_ptr;

   if (module_ptr->capi_ptr)
   {
      result = wcntr_topo_set_event_reg_prop_to_capi_modules(me_ptr->topo.gu.log_id,
                                                             module_ptr->capi_ptr,
                                                             module_ptr,
                                                             reg_event_payload_ptr,
                                                             is_register,
                                                             capi_supports_v1_event_ptr);
   }
   else
   {
      WCNTR_MSG(me_ptr->topo.gu.log_id, DBG_ERROR_PRIO, "fwk module not supported");
      result = AR_EUNSUPPORTED;
   }

   return result;
}

/**
 * For external ports, gen_cntr_operate_on_ext_in_port & gen_cntr_operate_on_ext_out_port is called
 * in 2 contexts:
 * 1. in the context of subgraph command: both ends of the connection belongs to the same SG.
 * 2. in the context of handle list of subgraph: this is an inter-SG connection.
 *
 * CLOSE, FLUSH, RESET, STOP are handled in this context.
 * START is not handled here, its handled based on downgraded state in gen_cntr_set_downgraded_state_on_output_port.
 */

/**
 *
 * sg_ptr->state is changed in the caller only after all operations are successful.
 *    1. Operate on all the modules in the current subgraph and current container.
 *    2. Operate on the connected ports which are in the peer sub graph but in the
 *       same container.
 *    3. Operate on all the external ports belonging to the same sub graph.
 */
ar_result_t wcntr_operate_on_subgraph(void *                     base_ptr,
                                      uint32_t                   sg_ops,
                                      wcntr_topo_sg_state_t      sg_state,
                                      wcntr_gu_sg_t *            gu_sg_ptr,
                                      spf_cntr_sub_graph_list_t *spf_sg_list_ptr)
{
   return AR_EOK;
}

ar_result_t wcntr_handle_sg_mgmt_cmd(wcntr_base_t *me_ptr, uint32_t sg_ops, wcntr_topo_sg_state_t sg_state)
{

   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   spf_cntr_sub_graph_list_t *sg_list_ptr;
   spf_msg_cmd_graph_mgmt_t * cmd_gmgmt_ptr;

   spf_msg_header_t *header_ptr = (spf_msg_header_t *)me_ptr->cmd_msg.payload_ptr;

   // VERIFY(result, header_ptr->payload_size >= sizeof(spf_msg_cmd_graph_mgmt_t));
   // VERIFY(result, topo_vtbl_ptr->set_sg_state )

   cmd_gmgmt_ptr = (spf_msg_cmd_graph_mgmt_t *)&header_ptr->payload_start;
   sg_list_ptr   = &cmd_gmgmt_ptr->sg_id_list;

   WCNTR_MSG(me_ptr->gu_ptr->log_id,
             DBG_LOW_PRIO,
             "handle_sg_mgmt_cmd: num_ip_port_handle %lu, num_op_port_handle %lu, num_ctrl_port_handle %lu, "
             "num_sub_graph %lu",
             cmd_gmgmt_ptr->cntr_port_hdl_list.num_ip_port_handle,
             cmd_gmgmt_ptr->cntr_port_hdl_list.num_op_port_handle,
             cmd_gmgmt_ptr->cntr_port_hdl_list.num_ctrl_port_handle,
             sg_list_ptr->num_sub_graph);

   for (uint32_t i = 0; i < sg_list_ptr->num_sub_graph; i++)
   {
      uint32_t *sg_id_base_ptr = sg_list_ptr->sg_id_list_ptr;
      uint32_t  sg_id          = *(sg_id_base_ptr + i);

      wcntr_gu_sg_t *  sg_ptr            = wcntr_gu_find_subgraph(me_ptr->gu_ptr, sg_id);
    

      if (sg_ptr)
      {
         wcntr_topo_sg_t *wcntr_topo_sg_ptr = (wcntr_topo_sg_t *)sg_ptr;
         // This will block mf propagation during open during multi VA session
         wcntr_topo_sg_ptr->can_mf_be_propagated = TRUE;
         TRY(result, wcntr_handle_current_sg_mgmt_cmd(me_ptr, sg_ptr, sg_ops, sg_state));
         WCNTR_MSG(me_ptr->gu_ptr->log_id,
                   DBG_LOW_PRIO,
                   "handle_sg_mgmt_cmd changed state to %u for sg_id 0x%lX  ",
                   sg_state,
                   wcntr_topo_sg_ptr->gu.id);

         if (sg_ops & WCNTR_TOPO_SG_OP_START)
         {
            result =
               wcntr_fwk_extn_handle_at_start((wcntr_t *)me_ptr, me_ptr->gu_ptr->sorted_module_list_ptr, sg_ptr->id);
         }

         if (sg_ops & WCNTR_TOPO_SG_OP_STOP)
         {
            result =
               wcntr_fwk_extn_handle_at_stop((wcntr_t *)me_ptr, me_ptr->gu_ptr->sorted_module_list_ptr, sg_ptr->id);
         }
      }
      else
      {
         WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "handle_sg_mgmt_cmd. sg_id (%08X) not found", sg_id);
	  }

   }

   for (wcntr_gu_sg_list_t *all_sgs_list_ptr = me_ptr->gu_ptr->sg_list_ptr; (NULL != all_sgs_list_ptr);
        LIST_ADVANCE(all_sgs_list_ptr))
   {
      result |= wcntr_update_data_port_states(me_ptr, all_sgs_list_ptr->sg_ptr);
      result |= wcntr_update_ctrl_port_states(me_ptr, all_sgs_list_ptr->sg_ptr);
      result |= wcntr_print_port_state(me_ptr, all_sgs_list_ptr->sg_ptr);
      wcntr_update_module_process_flag(me_ptr, all_sgs_list_ptr->sg_ptr);
   }

   if (sg_ops & (WCNTR_TOPO_SG_OP_START | WCNTR_TOPO_SG_OP_STOP | WCNTR_TOPO_SG_OP_SUSPEND))
   {
      me_ptr->fwk_evt_flags.sg_state_change       = TRUE;
      bool_t prev_cntr_run_state                  = me_ptr->flags.is_cntr_started;
      me_ptr->flags.is_cntr_started               = wcntr_is_atleast_one_sg_started(me_ptr);
      me_ptr->fwk_evt_flags.cntr_run_state_change = (prev_cntr_run_state != me_ptr->flags.is_cntr_started);
   }

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->gu_ptr->log_id)
   {
   }

   return result;
}

ar_result_t wcntr_set_get_cfg_wrapper(wcntr_base_t *      base_ptr,
                                      uint32_t            miid,
                                      uint32_t            pid,
                                      int8_t *            param_payload_ptr,
                                      uint32_t *          param_size_ptr,
                                      uint32_t *          error_code_ptr,
                                      bool_t              is_wcntr_set_cfg,
                                      bool_t              is_deregister,
                                      spf_cfg_data_type_t cfg_type)
{
   ar_result_t result = AR_EOK;

   {
      wcntr_gu_module_t *module_ptr = (wcntr_gu_module_t *)wcntr_gu_find_module(base_ptr->gu_ptr, miid);
      if (NULL == module_ptr)
      {
         WCNTR_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Module 0x%08lX not found", miid);
         result = AR_EFAILED;
      }
      else
      {
         result = wcntr_set_get_cfg_util(base_ptr,
                                         (void *)module_ptr,
                                         pid,
                                         param_payload_ptr,
                                         param_size_ptr,
                                         error_code_ptr,
                                         is_wcntr_set_cfg,
                                         is_deregister,
                                         cfg_type);
      }
   }

   return result;
}

ar_result_t wcntr_set_get_cfgs_packed(wcntr_base_t *me_ptr, gpr_packet_t *packet_ptr, spf_cfg_data_type_t cfg_type)
{
   ar_result_t result            = AR_EOK;
   bool_t      free_gpr_pkt_flag = FALSE;
   INIT_EXCEPTION_HANDLING

   apm_cmd_header_t *in_apm_cmd_header = GPR_PKT_GET_PAYLOAD(apm_cmd_header_t, packet_ptr);
   bool_t            is_out_of_band    = (0 != in_apm_cmd_header->mem_map_handle);
   bool_t            is_deregister     = FALSE; /*for setting persistence prop*/
   uint8_t *         param_data_ptr    = NULL;
   uint32_t          alignment_size    = 0;

   if (!is_out_of_band &&
       ((packet_ptr->opcode == APM_CMD_REGISTER_CFG) || (packet_ptr->opcode == APM_CMD_DEREGISTER_CFG) ||
        (packet_ptr->opcode == APM_CMD_REGISTER_SHARED_CFG) || (packet_ptr->opcode == APM_CMD_DEREGISTER_SHARED_CFG)))
   {
      result = AR_EFAILED;
      WCNTR_MSG(me_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "Expected out-of-band params, got in-band params, opcode 0x%X",
                packet_ptr->opcode);
      __gpr_cmd_end_command(packet_ptr, result);
      return result;
   }

   switch (packet_ptr->opcode)
   {
      case APM_CMD_SET_CFG:
      case APM_CMD_REGISTER_CFG:
      case APM_CMD_DEREGISTER_CFG:
      case APM_CMD_REGISTER_SHARED_CFG:
      case APM_CMD_DEREGISTER_SHARED_CFG:
      {
         WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_MED_PRIO, "Set cfg received from GPR opcode 0x%lX", packet_ptr->opcode);
         if (is_out_of_band)
         {
            result = spf_svc_get_cmd_payload_addr(me_ptr->gu_ptr->log_id,
                                                  packet_ptr,
                                                  NULL,
                                                  (uint8_t **)&param_data_ptr,
                                                  &alignment_size,
                                                  NULL,
                                                  apm_get_mem_map_client());

            if (AR_EOK != result)
            {
               WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Failed to get payload ptr");
               THROW(result, AR_EFAILED);
            }
         }
         else
         {
            param_data_ptr = (uint8_t *)(in_apm_cmd_header + 1);
         }

         if ((APM_CMD_DEREGISTER_CFG == packet_ptr->opcode) || (APM_CMD_DEREGISTER_SHARED_CFG == packet_ptr->opcode))
         {
            is_deregister = TRUE;
         }

         result = wcntr_set_get_cfgs_packed_loop(me_ptr,
                                                 param_data_ptr,
                                                 packet_ptr->dst_port,
                                                 in_apm_cmd_header->payload_size,
                                                 TRUE /* is_wcntr_set_cfg */,
                                                 is_out_of_band,
                                                 is_deregister,
                                                 cfg_type);

         if (!wcntr_is_any_handle_rest_pending(me_ptr))
         {
            if (is_out_of_band)
            {
               posal_cache_flush((uint32_t)param_data_ptr, in_apm_cmd_header->payload_size);
            }
         }
         break;
      }
      case APM_CMD_GET_CFG:
      {
         free_gpr_pkt_flag = TRUE;

         apm_cmd_rsp_get_cfg_t *cmd_get_cfg_rsp_ptr = NULL;
         gpr_packet_t *         gpr_rsp_pkt_ptr     = NULL;

         result = spf_svc_get_cmd_payload_addr(me_ptr->gu_ptr->log_id,
                                               packet_ptr,
                                               &gpr_rsp_pkt_ptr,
                                               (uint8_t **)&param_data_ptr,
                                               &alignment_size,
                                               NULL,
                                               apm_get_mem_map_client());
         if (AR_EOK != result)
         {
            WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Failed to get payload ptr");
            /* Send IBASIC with error */
            free_gpr_pkt_flag = FALSE;
            THROW(result, AR_EFAILED);
         }

         result = wcntr_set_get_cfgs_packed_loop(me_ptr,
                                                 param_data_ptr,
                                                 packet_ptr->dst_port,
                                                 in_apm_cmd_header->payload_size,
                                                 FALSE,          // is_set_param
                                                 is_out_of_band, // is_oob
                                                 is_deregister,
                                                 cfg_type);

         // doesn't support handle-rest

         if (!is_out_of_band)
         {
            cmd_get_cfg_rsp_ptr         = GPR_PKT_GET_PAYLOAD(apm_cmd_rsp_get_cfg_t, gpr_rsp_pkt_ptr);
            cmd_get_cfg_rsp_ptr->status = result;

            result = __gpr_cmd_async_send(gpr_rsp_pkt_ptr);
            if (AR_EOK != result)
            {
               __gpr_cmd_free(gpr_rsp_pkt_ptr);
               THROW(result, AR_EFAILED);
            }
         }
         else
         {
            posal_cache_flush((uint32_t)param_data_ptr, in_apm_cmd_header->payload_size);
            apm_cmd_rsp_get_cfg_t cmd_get_cfg_rsp = { 0 };
            cmd_get_cfg_rsp.status                = result;

            gpr_cmd_alloc_send_t args;
            args.src_domain_id = packet_ptr->dst_domain_id;
            args.dst_domain_id = packet_ptr->src_domain_id;
            args.src_port      = packet_ptr->dst_port;
            args.dst_port      = packet_ptr->src_port;
            args.token         = packet_ptr->token;
            args.opcode        = APM_CMD_RSP_GET_CFG;
            args.payload       = &cmd_get_cfg_rsp;
            args.payload_size  = sizeof(apm_cmd_rsp_get_cfg_t);
            args.client_data   = 0;
            TRY(result, __gpr_cmd_alloc_send(&args));
         }
         break;
      }
      default:
      {
         break;
      }
   }

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->gu_ptr->log_id)
   {
   }

   if (!wcntr_is_any_handle_rest_pending(me_ptr))
   {
      if (TRUE == free_gpr_pkt_flag)
      {
         __gpr_cmd_free(packet_ptr);
      }
      else
      {
         __gpr_cmd_end_command(packet_ptr, result);
      }
   }

   return result;
}

/**
 * Parse through multiple set or get configs which are packed in memory, and
 * call set_get_cfg() individually.
 */
static ar_result_t wcntr_set_get_cfgs_packed_loop(wcntr_base_t *      me_ptr,
                                                  uint8_t *           data_ptr,
                                                  uint32_t            miid,
                                                  uint32_t            payload_size,
                                                  bool_t              is_wcntr_set_cfg,
                                                  bool_t              is_oob,
                                                  bool_t              is_deregister,
                                                  spf_cfg_data_type_t cfg_type)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result            = AR_EOK;
   uint32_t    num_param_counter = 0;

   void *                    prev_handle_rest_ctx_ptr = me_ptr->handle_rest_ctx_ptr;
   wcntr_handle_rest_of_fn_t prev_handle_rest_fn      = me_ptr->handle_rest_fn;
   me_ptr->handle_rest_ctx_ptr                        = NULL;
   me_ptr->handle_rest_fn                             = NULL;

   // VERIFY(result, me_ptr->cntr_vtbl_ptr && me_ptr->cntr_vtbl_ptr->set_get_cfg);

   while (0 < payload_size)
   {
      uint32_t one_param_size           = 0;
      uint32_t min_param_size           = 0;
      uint32_t param_module_instance_id = 0;
      uint32_t param_id                 = 0;
      uint32_t param_size               = 0;
      uint32_t param_header_size        = 0;
      uint32_t error_code               = 0;

      if (SPF_CFG_DATA_SHARED_PERSISTENT == cfg_type)
      {
         apm_module_param_shared_data_t *param_shared_data_ptr = (apm_module_param_shared_data_t *)data_ptr;
         param_header_size                                     = sizeof(apm_module_param_shared_data_t);
         if (payload_size < param_header_size)
         {
            break;
         }
         param_module_instance_id = miid;
         param_id                 = param_shared_data_ptr->param_id;
         param_size               = param_shared_data_ptr->param_size;
         min_param_size           = param_header_size + param_shared_data_ptr->param_size;
         one_param_size           = param_header_size + WCNTR_ALIGN_8_BYTES(param_shared_data_ptr->param_size);
      }
      else
      {
         apm_module_param_data_t *param_data_ptr = (apm_module_param_data_t *)data_ptr;
         param_header_size                       = sizeof(apm_module_param_data_t);
         if (payload_size < param_header_size)
         {
            break;
         }
         if (cfg_type == SPF_CFG_DATA_PERSISTENT)
         {
            param_module_instance_id = miid;
         }
         else
         {
            param_module_instance_id = param_data_ptr->module_instance_id;
         }
         param_id   = param_data_ptr->param_id;
         param_size = param_data_ptr->param_size;
         error_code = param_data_ptr->error_code;

         min_param_size = param_header_size + param_data_ptr->param_size;
         one_param_size = param_header_size + WCNTR_ALIGN_8_BYTES(param_data_ptr->param_size);
      }

      bool_t skip_set   = FALSE;
      bool_t break_loop = FALSE;

      if (payload_size >= min_param_size)
      {
         VERIFY(result, param_module_instance_id == miid);

         int8_t *param_data_ptr = (int8_t *)(data_ptr + param_header_size);

         if (prev_handle_rest_ctx_ptr && prev_handle_rest_fn)
         {
            wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *set_get_ptr =
               (wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *)prev_handle_rest_ctx_ptr;

            if (set_get_ptr->param_payload_ptr == (int8_t *)param_data_ptr)
            {
               result |= set_get_ptr->overall_result;
               MFREE_NULLIFY(prev_handle_rest_ctx_ptr);
               prev_handle_rest_fn = NULL;
            }
            skip_set = TRUE;
            WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_MED_PRIO, "Warning: skipping set param");
         }

         if (!skip_set)
         {
            result |= wcntr_set_get_cfg_wrapper(me_ptr,
                                                miid,
                                                param_id,
                                                param_data_ptr,
                                                &param_size,
                                                &error_code,
                                                is_wcntr_set_cfg,
                                                is_deregister,
                                                cfg_type);

            if (SPF_CFG_DATA_SHARED_PERSISTENT != cfg_type)
            {
               apm_module_param_data_t *param_data_ptr = (apm_module_param_data_t *)data_ptr;
               param_data_ptr->param_size              = param_size;
               param_data_ptr->error_code              = error_code;
            }

            // after set-cfg if any handle rest if pending, break
            if (wcntr_is_any_handle_rest_pending(me_ptr))
            {
               if (me_ptr->handle_rest_ctx_ptr)
               {
                  wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *set_get_ptr =
                     (wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *)me_ptr->handle_rest_ctx_ptr;
                  set_get_ptr->overall_result = result;
               }
               break_loop = TRUE;
            }
         }
      }
      else
      {
         break_loop = TRUE;
      }

      if (min_param_size > one_param_size)
      {
         break;
      }
      else
      {
         num_param_counter++;
         data_ptr = (uint8_t *)(data_ptr + one_param_size);
         if (one_param_size > payload_size)
         {
            break;
         }
         else
         {
            payload_size = payload_size - one_param_size;
         }
      }

      if (break_loop)
      {
         break;
      }
   }

   if (!num_param_counter)
   {
      WCNTR_MSG(me_ptr->gu_ptr->log_id, DBG_MED_PRIO, "Warning: entering set/get cfg with zero set/get cfgs applied.");
   }

   CATCH(result, WCNTR_MSG_PREFIX, me_ptr->gu_ptr->log_id)
   {
   }

   if (prev_handle_rest_fn)
   {
      MFREE_NULLIFY(prev_handle_rest_ctx_ptr);
      prev_handle_rest_fn = NULL;
   }

   return result;
}

/**
 * Parse through multiple set or get configs which are not packed in memory, and
 * call set_get_cfg() individually.
 */
ar_result_t wcntr_set_get_cfgs_fragmented(wcntr_base_t *            me_ptr,
                                          apm_module_param_data_t **param_data_pptr,
                                          uint32_t                  num_param_id_cfg,
                                          bool_t                    is_wcntr_set_cfg,
                                          bool_t                    is_deregister,
                                          spf_cfg_data_type_t       cfg_type)
{
   ar_result_t result = AR_EOK;

   void *                    prev_handle_rest_ctx_ptr = me_ptr->handle_rest_ctx_ptr;
   wcntr_handle_rest_of_fn_t prev_handle_rest_fn      = me_ptr->handle_rest_fn;
   me_ptr->handle_rest_ctx_ptr                        = NULL;
   me_ptr->handle_rest_fn                             = NULL;

   // INIT_EXCEPTION_HANDLING
   // VERIFY(result, me_ptr->cntr_vtbl_ptr && me_ptr->cntr_vtbl_ptr->set_get_cfg);

   for (uint32_t i = 0; i < num_param_id_cfg; i++)
   {
      apm_module_param_data_t *param_data_ptr = (apm_module_param_data_t *)param_data_pptr[i];

      int8_t *param_payload_ptr = (int8_t *)(param_data_ptr + 1);

      /**
       * Some params like PARAM_ID_REAL_MODULE_ID involve thread re-launch.
       * When thread re-launch is needed, we cannot continue set-cfg as subsequent params may need higher stack.
       * Due to this reason, we let the thread re-launch, come back for the rest of the cfg params.
       * We need to skip all earlier params. When handling rest, we need to skip until (& including) we reach the
       * place where we left off
       *
       * this handle_rest happens only for set-cfg (not get, as get doesn't need thread relaunch)
       */
      if (prev_handle_rest_ctx_ptr && prev_handle_rest_fn)
      {
         wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *set_get_ptr =
            (wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *)prev_handle_rest_ctx_ptr;
         if (set_get_ptr->param_payload_ptr == (int8_t *)param_payload_ptr)
         {
            result |= set_get_ptr->overall_result;
            MFREE_NULLIFY(prev_handle_rest_ctx_ptr);
            prev_handle_rest_fn = NULL;
         }
         continue;
      }

      result |= wcntr_set_get_cfg_wrapper(me_ptr,
                                          param_data_ptr->module_instance_id,
                                          param_data_ptr->param_id,
                                          param_payload_ptr,
                                          &param_data_ptr->param_size,
                                          &param_data_ptr->error_code,
                                          is_wcntr_set_cfg,
                                          is_deregister,
                                          cfg_type);

      // after set-cfg if any new handle rest if pending, break
      if (wcntr_is_any_handle_rest_pending(me_ptr))
      {
         if (me_ptr->handle_rest_ctx_ptr)
         {
            wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *set_get_ptr =
               (wcntr_handle_rest_ctx_for_wcntr_set_cfg_t *)me_ptr->handle_rest_ctx_ptr;
            set_get_ptr->overall_result = result;
         }
         break;
      }
   }

   if (prev_handle_rest_fn)
   {
      MFREE_NULLIFY(prev_handle_rest_ctx_ptr);
      prev_handle_rest_fn = NULL;
   }

   return result;
}

ar_result_t wcntr_handle_prepare(wcntr_base_t *base_ptr, spf_msg_cmd_graph_mgmt_t *cmd_gmgmt_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

   if (!base_ptr->gu_ptr->sorted_module_list_ptr)
   {
      // if sorted module list doesn't exist, sort now.
      // For first SG, sorting happens here. For subsequent SG, sorting happens in graph-open, close (& shouldn't happen
      // here).

      WCNTR_MSG(base_ptr->gu_ptr->log_id, DBG_MED_PRIO, "handle_prepare sorting modules now ");
      TRY(result, wcntr_gu_update_sorted_list(base_ptr->gu_ptr, base_ptr->heap_id));

      // wcntr_gu_print_graph(base_ptr->gu_ptr);
   }

   /**
    * If media fmt came before prepare, then it would be sitting at input
    *
    * When prepare is given for downstream SG, upstream ext ports also get prepare.
    *  Ideally, upstream shouldn't go through media fmt through all modules, but only start from ext out port.
    *  However, calling propagate_media_fmt shouldn't be an issue as port state checks are made appropriately in
    *  propagate_media_fmt. Similarly for multi-SG container cases (where one of the SG receives prepare).
    *  It's not possible to go through only connected list of modules as we don't know how they are connected.
    *  We have to go through sorted module list.
    */
   WCNTR_MSG(base_ptr->gu_ptr->log_id, DBG_MED_PRIO, "handle_prepare calling propagate_media_fmt now ");

   TRY(result, wcntr_topo_propagate_media_fmt(base_ptr->topo_ptr, FALSE /* is_data_path*/));

   WCNTR_MSG(base_ptr->gu_ptr->log_id,
             DBG_MED_PRIO,
             "handle_prepare calling port_data_thresh_change now");

   TRY(result, wcntr_handle_port_data_thresh_change_event(base_ptr));

   CATCH(result, WCNTR_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }

   return result;
}
