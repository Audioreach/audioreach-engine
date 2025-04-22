/**
 * \file apm_proxy_vcpm_utils.c
 *
 * \brief
 *
 *     This file contains APM's VCPM proxy manager utlity function implementations
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include "apm_graph_db.h"
#include "apm_msg_utils.h"
#include "apm_cmd_sequencer.h"
#include "apm_proxy_def.h"
#include "apm_cmd_utils.h"
#include "apm_proxy_utils.h"
#include "apm_proxy_vcpm_utils_i.h"
#include "apm_proxy_vcpm_utils.h"
#include "vcpm_api.h"
#include "vcpm.h"
#include "posal_intrinsics.h"
#include "irm_api.h"

/****************************************************************************
 * GLOBALS                                                                  *
 ****************************************************************************/

/****************************************************************************
 * Function Declarations
 ****************************************************************************/

bool_t apm_proxy_util_find_proxy_manager(spf_list_node_t      *proxy_mgr_list_ptr,
                                         apm_proxy_manager_t **proxy_mgr_pptr,
                                         apm_sub_graph_t      *sg_node_ptr)
{
   bool_t match_found = FALSE;

   /** Validate input params. */
   if (NULL == proxy_mgr_pptr || NULL == sg_node_ptr)
   {
      return FALSE;
   }

   uint32_t scenario_id = sg_node_ptr->prop.scenario_id;

   while (proxy_mgr_list_ptr)
   {
      apm_proxy_manager_t *proxy_mgr = (apm_proxy_manager_t *)proxy_mgr_list_ptr->obj_ptr;

      if (scenario_id == proxy_mgr->scenario_id)
      {
         /** This check is specific to VCPM.
             For other Proxies, when/if required, we need to change this match finding logic slightly. */
         if (sg_node_ptr->prop.vsid == proxy_mgr->vcpm_properties.vsid)
         {
            match_found     = TRUE;
            *proxy_mgr_pptr = proxy_mgr;
            break;
         }
      }
      proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;
   }

   return match_found;
}

ar_result_t apm_proxy_util_add_sg_to_proxy_cmd_ctrl(apm_cmd_ctrl_t   *apm_cmd_ctrl_ptr,
                                                    spf_list_node_t **proxy_manager_list_pptr,
                                                    uint32_t         *num_proxy_mgrs_ptr,
                                                    apm_sub_graph_t  *sg_node_ptr)
{
   ar_result_t           result              = AR_EOK;
   bool_t                match_found         = FALSE;
   apm_proxy_manager_t  *proxy_mgr_ptr       = NULL;
   apm_proxy_cmd_ctrl_t *curr_proxy_cmd_ctrl = NULL;

   /** Validate input params. */
   if (NULL == proxy_manager_list_pptr || NULL == num_proxy_mgrs_ptr || NULL == sg_node_ptr || NULL == apm_cmd_ctrl_ptr)
   {
      return AR_EBADPARAM;
   }

   uint32_t scenario_id = sg_node_ptr->prop.scenario_id;

   spf_list_node_t *proxy_mgr_list_ptr = *proxy_manager_list_pptr;

   /** Check if there is a Proxy Manager already existing for this scenario ID. */
   match_found = apm_proxy_util_find_proxy_manager(proxy_mgr_list_ptr, &proxy_mgr_ptr, sg_node_ptr);

   if (FALSE == match_found)
   {
      /** Create Proxy manager node */

      /** ALlocate memory for Proxy manager node for APM graph DB */
      if (NULL == (proxy_mgr_ptr = (apm_proxy_manager_t *)posal_memory_malloc(sizeof(apm_proxy_manager_t),
                                                                              APM_INTERNAL_STATIC_HEAP_ID)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_add_sg_to_proxy_cmd_ctrl: Failed to allocat Proxy Manager node mem, SG_ID: [0x%lX]",
                sg_node_ptr->sub_graph_id);

         return AR_ENOMEMORY;
      }

      /** Clear the allocated struct */
      memset(proxy_mgr_ptr, 0, sizeof(apm_proxy_manager_t));
      proxy_mgr_ptr->scenario_id = scenario_id;

      /** Initiate Proxy Specific properties.*/
      switch (scenario_id)
      {
         case APM_SUB_GRAPH_SID_VOICE_CALL:
         {
            proxy_mgr_ptr->vcpm_properties.vsid = sg_node_ptr->prop.vsid;
            proxy_mgr_ptr->proxy_instance_id    = VCPM_MODULE_INSTANCE_ID;

            if (AR_EOK != (result = vcpm_get_spf_handle(&proxy_mgr_ptr->proxy_handle_ptr)))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "apm_proxy_util_add_sg_to_proxy_cmd_ctrl:VCPM handle not available, SG_ID: [0x%lX]",
                      sg_node_ptr->sub_graph_id);
               return AR_ENOTREADY;
            }
            break;
         }

         default:
            return AR_EBADPARAM;
      }

      AR_MSG(DBG_HIGH_PRIO,
             "apm_proxy_util_add_sg_to_proxy_cmd_ctrl:created proxy manager"
             " Scenario ID [0x%lX] , VSID 0x%08X, SG_ID: [0x%lX]",
             scenario_id,
             proxy_mgr_ptr->vcpm_properties.vsid,
             sg_node_ptr->sub_graph_id);

      /** Add vcpm module node to the vcpm graphs list of vcpm info structure. */
      if (AR_EOK != (result = apm_db_add_node_to_list(proxy_manager_list_pptr, proxy_mgr_ptr, num_proxy_mgrs_ptr)))
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_add_sg_to_proxy_cmd_ctrl: Failed to add proxy manager"
                " to the list of APM graph info,"
                "Scenario ID [0x%lX] , SG_ID: [0x%lX]",
                scenario_id,
                sg_node_ptr->sub_graph_id);

         return AR_EFAILED;
      }
   }

   /** Add subgraph node to the Proxy Manager's cached cmd ctrl. */
   if (proxy_mgr_ptr)
   {

      result = apm_proxy_util_get_cmd_ctrl_obj(proxy_mgr_ptr, apm_cmd_ctrl_ptr, &curr_proxy_cmd_ctrl);

      /** Get/allocate a cmd contrl object from this Proxy Manager for processing current command. */
      if (AR_EOK != result || NULL == curr_proxy_cmd_ctrl)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_add_sg_to_proxy_cmd_ctrl: Failed to allocate proxy_cmd_ctrl. result %d",
                result);
         return result;
      }

      /** Add apm sg node to the proxy_cmd_ctrl's cached cfg params*/

      apm_db_add_node_to_list(&curr_proxy_cmd_ctrl->cached_cfg_params.graph_open_params.sg_list_ptr,
                              (void *)sg_node_ptr,
                              &curr_proxy_cmd_ctrl->cached_cfg_params.graph_open_params.num_proxy_sub_graphs);
      /** Set the pending flag  */

      /** Add Proxy manger to apm cmd-Rsp ctrl inactive list. Since this proxy is added during parse time before any
       * set-cfg is executed, we wait until the vcpm proxy is ready. It will be used while executing the graph open op
       * id for proxy obj in the graph open cmd sequecner.*/
      if (!curr_proxy_cmd_ctrl->rsp_ctrl.pending_msg_proc)
      {
         apm_db_add_node_to_list(&apm_cmd_ctrl_ptr->rsp_ctrl.inactive_proxy_mgr_list_ptr,
                                 (void *)proxy_mgr_ptr,
                                 &apm_cmd_ctrl_ptr->rsp_ctrl.num_inactive_proxy_mgrs);
         /** Set the pending flag  */
         curr_proxy_cmd_ctrl->rsp_ctrl.pending_msg_proc = TRUE;
      }

      AR_MSG(DBG_HIGH_PRIO,
             "apm_proxy_util_add_sg_to_proxy_cmd_ctrl: Scenario ID [0x%lX] , num subgraphs: %d",
             scenario_id,
             proxy_mgr_ptr->num_proxy_subgraphs);
   }

   return result;
}

bool_t apm_proxy_merge_proxy_objects(apm_graph_info_t *graph_info_ptr, uint32_t vsid1, uint32_t vsid2)
{
   apm_proxy_manager_t     *proxy_mgr1 = NULL, *proxy_mgr2 = NULL;
   apm_proxysg_list_node_t *temp_proxy_sg_node = NULL;
   spf_list_node_t         *proxy_mgr_list_ptr = graph_info_ptr->proxy_manager_list_ptr;

   while (proxy_mgr_list_ptr)
   {
      apm_proxy_manager_t *proxy_mgr = (apm_proxy_manager_t *)proxy_mgr_list_ptr->obj_ptr;

      if (vsid1 == proxy_mgr->vcpm_properties.vsid)
      {
         proxy_mgr1 = proxy_mgr;
      }

      if (vsid2 == proxy_mgr->vcpm_properties.vsid)
      {
         proxy_mgr2 = proxy_mgr;
      }
      proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;
   }

   /** If VSID is valid and no proxy manager found, then flag
    *  error */
   if (((APM_SUB_GRAPH_VSID_DONT_CARE != vsid1) && !proxy_mgr1) ||
       ((APM_SUB_GRAPH_VSID_DONT_CARE != vsid2) && !proxy_mgr2))
   {
      AR_MSG(DBG_HIGH_PRIO,
             "apm_proxy_util_add_sg_to_proxy_cmd_ctrl: No proxy found for VSID1[0x%lX], proxy1[0x%lX], or "
             "VSID2[0x%lX], proxy2[0x%lX]",
             vsid1,
             proxy_mgr1,
             vsid2,
             proxy_mgr2);

      return false;
   }

   /** If no proxy found for VSID =
    *  APM_SUB_GRAPH_VSID_DONT_CARE, then nothing to merge and
    *  return */
   if ((NULL == proxy_mgr1) || (NULL == proxy_mgr2))
   {
      AR_MSG(DBG_HIGH_PRIO,
             "apm_proxy_util_add_sg_to_proxy_cmd_ctrl::WARNING:: No proxy found for VSID1[0x%lX], proxy1[0x%lX], or "
             "VSID2[0x%lX], proxy2[0x%lX]",
             vsid1,
             proxy_mgr1,
             vsid2,
             proxy_mgr2);

      return true;
   }

   temp_proxy_sg_node = proxy_mgr1->proxy_sg_list_ptr;

   while (NULL != temp_proxy_sg_node->next_ptr)
   {
      temp_proxy_sg_node = temp_proxy_sg_node->next_ptr;
   }
   /* Concatenate the SG lists. */
   temp_proxy_sg_node->next_ptr            = proxy_mgr2->proxy_sg_list_ptr;
   proxy_mgr2->proxy_sg_list_ptr->prev_ptr = temp_proxy_sg_node;
   proxy_mgr1->num_proxy_subgraphs += proxy_mgr2->num_proxy_subgraphs;

   /** Remove Proxy Manager node from apm graph info list. */
   apm_db_remove_node_from_list(&graph_info_ptr->proxy_manager_list_ptr,
                                proxy_mgr2,
                                &graph_info_ptr->num_proxy_managers);

   /** Free the memory allocated to proxy manager node.*/
   posal_memory_free(proxy_mgr2);

   return true;
}

bool_t apm_proxy_util_find_vcpm_proxy_mgr(apm_graph_info_t        *apm_graph_info_ptr,
                                          apm_proxy_manager_t    **proxy_mgr_pptr,
                                          apm_module_param_data_t *param_data)
{
   bool_t      match_found = FALSE;
   uint32_t    vsid        = 0;
   uint32_t    real_vsid   = 0;
   ar_result_t result      = AR_EOK;
   /** Validate input params. */
   if (NULL == apm_graph_info_ptr || NULL == proxy_mgr_pptr || NULL == param_data)
   {
      return FALSE;
   }

   spf_list_node_t *proxy_mgr_list_ptr = apm_graph_info_ptr->proxy_manager_list_ptr;

   /* Parse the payload partially to find the VSID. */
   switch (param_data->param_id)
   {
      case VCPM_PARAM_ID_CAL_KEYS:
      {
         vcpm_param_cal_keys_payload_t *payload_ptr = (vcpm_param_cal_keys_payload_t *)(param_data + 1);
         vsid                                       = payload_ptr->vsid;
         break;
      }

      case VCPM_PARAM_ID_TTY_MODE:
      {
         vcpm_param_id_tty_mode_t *payload_ptr = (vcpm_param_id_tty_mode_t *)(param_data + 1);
         vsid                                  = payload_ptr->vsid;
         break;
      }

      case VCPM_PARAM_ID_NUM_VOL_STEPS:
      {
         vcpm_param_num_vol_steps_payload_t *payload_ptr = (vcpm_param_num_vol_steps_payload_t *)(param_data + 1);
         vsid                                            = payload_ptr->vsid;
         break;
      }

      case VCPM_PARAM_ID_VOC_PKT_LOOPBACK_DELAY:
      {
         vcpm_param_id_voc_pkt_loopback_delay_t *payload_ptr =
            (vcpm_param_id_voc_pkt_loopback_delay_t *)(param_data + 1);
         vsid = payload_ptr->vsid;
         break;
      }

      case VCPM_PARAM_ID_VSID:
      {
         /* This Param ID is intended to the graph which was created with a dont care value
            APM_SUB_GRAPH_VSID_DONT_CARE.
            Hence check for dont care VSID, APM_SUB_GRAPH_VSID_DONT_CARE to push to the list. */

         vcpm_param_vsid_payload_t *payload_ptr = (vcpm_param_vsid_payload_t *)(param_data + 1);

         /** Get the real VSID */
         real_vsid = payload_ptr->vsid;

         /* During device switch, a new proxy object will be created (with default_vsid )for the newly opened PP SG's.
          * When the real VSID is set on the respective object, it shall be merged with the parent object
          * associated with the real VSID.
          */
         if (apm_proxy_merge_proxy_objects(apm_graph_info_ptr, real_vsid, APM_SUB_GRAPH_VSID_DONT_CARE))
         {
            vsid = payload_ptr->vsid;
         }
         else
         {
            vsid = APM_SUB_GRAPH_VSID_DONT_CARE;
         }
         break;
      }

      case VCPM_PARAM_ID_VOICE_CONFIG:
      {
         vcpm_param_voice_config_payload_t *payload_ptr = (vcpm_param_voice_config_payload_t *)(param_data + 1);

         uint8_t *tmp_ptr = (uint8_t *)(payload_ptr + 1);

         for (uint32_t i = 0; i < payload_ptr->num_sub_graphs; i++)
         {
            vcpm_cfg_subgraph_properties_t *sg_prop_ptr = (vcpm_cfg_subgraph_properties_t *)tmp_ptr;
            apm_sub_graph_t                *sg_node_ptr = NULL;

            tmp_ptr += sizeof(vcpm_cfg_subgraph_properties_t);

            result =
               apm_db_get_sub_graph_node(apm_graph_info_ptr, sg_prop_ptr->sub_graph_id, &sg_node_ptr, APM_DB_OBJ_QUERY);

            if (AR_EOK != result)
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "apm_proxy_util_find_vcpm_proxy_mgr: failed to get sg node 0x%lx. Bailing out.",
                      sg_prop_ptr->sub_graph_id);
               break;
            }

            if (NULL != sg_node_ptr)
            {
               if (APM_SUB_GRAPH_SID_VOICE_CALL == sg_node_ptr->prop.scenario_id)
               {
                  vsid = sg_node_ptr->prop.vsid;
                  break;
               }
            }

            for (uint32_t j = 0; j < sg_prop_ptr->num_vcpm_properties; j++)
            {
               vcpm_property_config_struct_t *prop_cfg_ptr = (vcpm_property_config_struct_t *)tmp_ptr;
               tmp_ptr += sizeof(vcpm_property_config_struct_t) + prop_cfg_ptr->property_size;
            }
         }
         break;
      }

      case VCPM_PARAM_ID_CAL_TABLE:
      {
         vcpm_param_cal_table_payload_t *payload_ptr  = (vcpm_param_cal_table_payload_t *)(param_data + 1);
         vcpm_sgid_cal_table_t          *sg_cal_table = (vcpm_sgid_cal_table_t *)(payload_ptr + 1);
         apm_sub_graph_t                *sg_node_ptr  = NULL;

         result =
            apm_db_get_sub_graph_node(apm_graph_info_ptr, sg_cal_table->sub_graph_id, &sg_node_ptr, APM_DB_OBJ_QUERY);

         if (AR_EOK != result)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "apm_proxy_util_find_vcpm_proxy_mgr: failed to get sg node 0x%lx. Bailing out.",
                   sg_cal_table->sub_graph_id);
            break;
         }

         if (NULL != sg_node_ptr)
         {
            if (APM_SUB_GRAPH_SID_VOICE_CALL == sg_node_ptr->prop.scenario_id)
            {
               vsid = sg_node_ptr->prop.vsid;
               break;
            }
         }
         break;
      }
      case VCPM_PARAM_ID_TX_DEV_PP_CHANNEL_INFO:
      {
         vcpm_param_id_tx_dev_pp_channel_info_t *payload_ptr =
            (vcpm_param_id_tx_dev_pp_channel_info_t *)(param_data + 1);
         vsid = payload_ptr->vsid;
         break;
      }

      case VCPM_PARAM_ID_ACTIVE_CAL_KEYS:
      {
         vcpm_param_active_cal_keys_payload_t *payload_ptr = (vcpm_param_active_cal_keys_payload_t *)(param_data + 1);
         uint32_t                              sg_id       = payload_ptr->sg_id;
         apm_sub_graph_t                      *sg_node_ptr = NULL;

         result = apm_db_get_sub_graph_node(apm_graph_info_ptr, sg_id, &sg_node_ptr, APM_DB_OBJ_QUERY);

         if (AR_EOK != result)
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "apm_proxy_util_find_vcpm_proxy_mgr: failed to get sg node 0x%lx. Bailing out.",
                   sg_id);
            break;
         }

         if (NULL != sg_node_ptr)
         {
            if (APM_SUB_GRAPH_SID_VOICE_CALL == sg_node_ptr->prop.scenario_id)
            {
               vsid = sg_node_ptr->prop.vsid;
               break;
            }
         }
         break;
      }

      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_find_vcpm_proxy_mgr: "
                "uknown VCPM Param ID [0x%lX]",
                param_data->param_id);
         result = AR_EUNEXPECTED;
      }
   }

   if (AR_EOK != result)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_proxy_util_find_vcpm_proxy_mgr: getting the VSID failed for VCPM Param ID 0x%lx",
             param_data->param_id);
      return FALSE;
   }

   AR_MSG(DBG_HIGH_PRIO,
          "apm_proxy_util_find_vcpm_proxy_mgr: "
          "Received VCPM Param ID [0x%lX] for VSID 0x%08X",
          param_data->param_id,
          vsid);

   while (proxy_mgr_list_ptr)
   {
      apm_proxy_manager_t *proxy_mgr = (apm_proxy_manager_t *)proxy_mgr_list_ptr->obj_ptr;

      if (vsid == proxy_mgr->vcpm_properties.vsid)
      {
         match_found     = TRUE;
         *proxy_mgr_pptr = proxy_mgr;

         /** Update the real VSID once set from client */
         if ((VCPM_PARAM_ID_VSID == param_data->param_id) && real_vsid)
         {
            proxy_mgr->vcpm_properties.vsid = real_vsid;

            /* Update vsid in subgraphs */
            apm_proxysg_list_node_t *curr_sg_node_ptr = proxy_mgr->proxy_sg_list_ptr;
            while (curr_sg_node_ptr)
            {
               apm_sub_graph_t *curr_sg_ptr = curr_sg_node_ptr->apm_sg_node;
               curr_sg_ptr->prop.vsid       = real_vsid;
               curr_sg_node_ptr             = curr_sg_node_ptr->next_ptr;
            }
         }
         break;
      }

      proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;
   }

   return match_found;
}

// ==============================================================================================================

ar_result_t apm_proxy_util_send_graph_info_to_proxy_managers(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr,
                                                             spf_handle_t   *apm_handle_ptr)
{
   ar_result_t           result             = AR_EOK;
   spf_list_node_t      *proxy_mgr_list_ptr = NULL;
   spf_msg_t             proxy_msg;
   uint32_t              msg_payload_size    = 0;
   apm_proxy_cmd_ctrl_t *curr_proxy_cmd_ctrl = NULL;
   uint32_t              num_proxy_subgraphs = 0;
   uint32_t              num_mod_param_cfg   = 0;
   spf_msg_token_t       token;

   /** Validate input params. */
   if (NULL == apm_cmd_ctrl_ptr || NULL == apm_handle_ptr)
   {
      return AR_EBADPARAM;
   }

   apm_proxy_msg_opcode_t *proxy_msg_opcode_ptr = &apm_cmd_ctrl_ptr->proxy_msg_opcode;
   uint32_t                msg_opcode = proxy_msg_opcode_ptr->proxy_opcode_list[proxy_msg_opcode_ptr->proxy_opcode_idx];

   proxy_mgr_list_ptr = apm_cmd_ctrl_ptr->rsp_ctrl.active_proxy_mgr_list_ptr;
   while (proxy_mgr_list_ptr)
   {
      apm_proxy_manager_t *proxy_mgr_node = (apm_proxy_manager_t *)proxy_mgr_list_ptr->obj_ptr;

      result = apm_proxy_util_get_cmd_ctrl_obj(proxy_mgr_node, apm_cmd_ctrl_ptr, &curr_proxy_cmd_ctrl);

      if (AR_EOK != result || NULL == curr_proxy_cmd_ctrl)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_send_graph_info_to_proxy_managers: Failed to allocate proxy_cmd_ctrl. result %d",
                result);
         return result;
      }

      num_proxy_subgraphs = curr_proxy_cmd_ctrl->cached_cfg_params.graph_open_params.num_proxy_sub_graphs;
      spf_list_node_t *proxy_sg_list_ptr = curr_proxy_cmd_ctrl->cached_cfg_params.graph_open_params.sg_list_ptr;

      num_mod_param_cfg = curr_proxy_cmd_ctrl->cached_cfg_params.graph_open_params.num_mod_param_cfg;
      spf_list_node_t *param_data_list_ptr =
         curr_proxy_cmd_ctrl->cached_cfg_params.graph_open_params.param_data_list_ptr;

      if (0 == num_proxy_subgraphs || NULL == proxy_sg_list_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_send_graph_info_to_proxy_managers: unexpected. num_proxy_subgraphs %d,"
                " proxy_sg_list_ptr 0x%08X",
                num_proxy_subgraphs,
                proxy_sg_list_ptr);

         /** No Proxy SubGraphs in this container, to send command. */
         proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;
         continue;
      }

      msg_payload_size = sizeof(spf_msg_header_t) + sizeof(spf_msg_cmd_proxy_graph_info_t) +
                         (sizeof(apm_sub_graph_t *) * num_proxy_subgraphs) + (SIZE_OF_PTR() * num_mod_param_cfg);

      token.token_ptr = curr_proxy_cmd_ctrl;

      /** Send the init message to container */
      if (AR_EOK != (result = spf_msg_create_msg(&proxy_msg,                       /** MSG Ptr */
                                                 &msg_payload_size,                /** MSG payload size */
                                                 msg_opcode,                       /** MSG opcode */
                                                 apm_handle_ptr,                   /** APM response handle */
                                                 &token,                           /** MSG Token */
                                                 proxy_mgr_node->proxy_handle_ptr, /** Destination handle */
                                                 APM_INTERNAL_STATIC_HEAP_ID)))
      {
         AR_MSG(DBG_HIGH_PRIO,
                "apm_proxy_util_send_graph_info_to_proxy_managers :FAILED to create msg payload,"
                " opcode: 0x%lx, proxy instance ID[0x%lX], result: 0x%lx",
                msg_opcode,
                proxy_mgr_node->scenario_id,
                result);

         return result;
      }

      spf_msg_header_t *msg_header_ptr = (spf_msg_header_t *)proxy_msg.payload_ptr;

      /** Get the start pointer to graph mgmt msg header */
      spf_msg_cmd_proxy_graph_info_t *proxy_graph_info =
         (spf_msg_cmd_proxy_graph_info_t *)&msg_header_ptr->payload_start;

      proxy_graph_info->num_proxy_sub_graphs = num_proxy_subgraphs;
      proxy_graph_info->sg_node_ptr_array    = (void **)(proxy_graph_info + 1);

      apm_sub_graph_t **apm_sg_ptr = (apm_sub_graph_t **)(proxy_graph_info->sg_node_ptr_array);

      proxy_graph_info->num_param_id_cfg = num_mod_param_cfg;
      proxy_graph_info->param_data_pptr  = (void **)((uint8_t *)apm_sg_ptr + (num_proxy_subgraphs * SIZE_OF_PTR()));

      AR_MSG(DBG_HIGH_PRIO,
             "apm_proxy_util_send_graph_info_to_proxy_managers:  num_sgs %d num_param_cfg %d"
             " apm_sg_ptr 0x%08X, param_data_pptr 0x%08X ",
             num_proxy_subgraphs,
             num_mod_param_cfg,
             apm_sg_ptr,
             proxy_graph_info->param_data_pptr);

      /* Fill the SubGraph node pointers in the payload. */
      for (uint32_t i = 0; i < num_proxy_subgraphs; i++)
      {
         apm_sg_ptr[i]     = (apm_sub_graph_t *)proxy_sg_list_ptr->obj_ptr;
         proxy_sg_list_ptr = proxy_sg_list_ptr->next_ptr;
      }

      /* Fill the param data pointers in the payload. */
      apm_module_param_data_t **param_data_pptr = (apm_module_param_data_t **)proxy_graph_info->param_data_pptr;

      for (uint32_t i = 0; i < num_mod_param_cfg; i++)
      {
         param_data_pptr[i]  = (apm_module_param_data_t *)param_data_list_ptr->obj_ptr;
         param_data_list_ptr = param_data_list_ptr->next_ptr;
      }

      /** Push the message packet to container command Q */
      if (AR_EOK != (result = spf_msg_send_cmd(&proxy_msg, proxy_mgr_node->proxy_handle_ptr)))
      {
         AR_MSG(DBG_ERROR_PRIO, "Failed to send msg  to Proxy manager cmdQ, result: 0x%lx", result);

         spf_msg_return_msg(&proxy_msg);
         return result;
      }

      /** Update command ctronl and response control. */
      curr_proxy_cmd_ctrl->rsp_ctrl.rsp_pending       = TRUE;
      curr_proxy_cmd_ctrl->rsp_ctrl.rsp_result        = AR_EOK;
      curr_proxy_cmd_ctrl->rsp_ctrl.reuse_rsp_msg_buf = FALSE;

      /** Increment the number of commands issues  */
      apm_cmd_ctrl_ptr->rsp_ctrl.num_proxy_cmd_issued++;
      apm_cmd_ctrl_ptr->rsp_ctrl.cmd_rsp_pending = CMD_RSP_PENDING;

      proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;
   }

   return result;
}
ar_result_t apm_proxy_util_send_mgmt_command_to_proxy_managers(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr,
                                                               spf_handle_t   *apm_handle_ptr)
{
   ar_result_t           result             = AR_EOK;
   spf_list_node_t      *proxy_mgr_list_ptr = NULL;
   spf_msg_t             proxy_msg;
   uint32_t              msg_payload_size    = 0;
   apm_proxy_cmd_ctrl_t *curr_proxy_cmd_ctrl = NULL;
   spf_msg_token_t       token;

   /** Validate input params. */
   if (NULL == apm_cmd_ctrl_ptr || NULL == apm_handle_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "apm_proxy_util_send_mgmt_command_to_proxy_managers: Invalid i/p args apm_cmd_ctrl_ptr[0x%lX], "
             "apm_handle_ptr[0x%lX]",
             apm_cmd_ctrl_ptr,
             apm_handle_ptr);

      return AR_EBADPARAM;
   }

   uint32_t *proxy_sg_list_ptr = (uint32_t *)apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.proxy_sg_id_list_ptr;

   apm_proxy_msg_opcode_t *proxy_msg_opcode_ptr = &apm_cmd_ctrl_ptr->proxy_msg_opcode;
   uint32_t                msg_opcode = proxy_msg_opcode_ptr->proxy_opcode_list[proxy_msg_opcode_ptr->proxy_opcode_idx];

   proxy_mgr_list_ptr = apm_cmd_ctrl_ptr->rsp_ctrl.active_proxy_mgr_list_ptr;

   AR_MSG(DBG_HIGH_PRIO,
          "apm_proxy_util_send_mgmt_command_to_proxy_managers: "
          "Send command to all Proxy Managers. cmd ID 0x%08X",
          msg_opcode);

   while (proxy_mgr_list_ptr)
   {
      apm_proxy_manager_t *proxy_mgr_node = (apm_proxy_manager_t *)proxy_mgr_list_ptr->obj_ptr;

      result = apm_proxy_util_get_cmd_ctrl_obj(proxy_mgr_node, apm_cmd_ctrl_ptr, &curr_proxy_cmd_ctrl);

      if (AR_EOK != result || NULL == curr_proxy_cmd_ctrl)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_send_mgmt_command_to_proxy_managers: Failed to allocate proxy_cmd_ctrl. result %d",
                result);
         return result;
      }

      apm_proxy_graph_mgmt_params_t *proxy_cached_mgmt_params =
         &curr_proxy_cmd_ctrl->cached_cfg_params.graph_mgmt_params;

      uint32_t start_index = proxy_cached_mgmt_params->input_start_index;
      uint32_t end_index   = proxy_cached_mgmt_params->input_end_index;

      if (0 == (end_index - start_index))
      {
         /** No payload to be sent to this Proxy Manager. */
         proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;
         continue;
      }

      msg_payload_size =
         sizeof(spf_msg_header_t) + sizeof(spf_msg_cmd_graph_mgmt_t) + (sizeof(uint32_t) * (end_index - start_index));

      token.token_ptr = curr_proxy_cmd_ctrl;

      /** Send the init message to container */
      if (AR_EOK != (result = spf_msg_create_msg(&proxy_msg,                       /** MSG Ptr */
                                                 &msg_payload_size,                /** MSG payload size */
                                                 msg_opcode,                       /** MSG opcode */
                                                 apm_handle_ptr,                   /** APM response handle */
                                                 &token,                           /** MSG Token */
                                                 proxy_mgr_node->proxy_handle_ptr, /** Destination handle */
                                                 APM_INTERNAL_STATIC_HEAP_ID)))
      {
         AR_MSG(DBG_HIGH_PRIO,
                "FAILED to create msg payload, opcode: 0x%lx, proxy instance ID[0x%lX], result: 0x%lx",
                msg_opcode,
                proxy_mgr_node->scenario_id,
                result);

         return result;
      }

      spf_msg_header_t *msg_header_ptr = (spf_msg_header_t *)proxy_msg.payload_ptr;

      /** Get the start pointer to graph mgmt msg header */
      spf_msg_cmd_graph_mgmt_t *graph_mgmt_msg_ptr = (spf_msg_cmd_graph_mgmt_t *)&msg_header_ptr->payload_start;

      memset((void *)graph_mgmt_msg_ptr, 0, (msg_payload_size - sizeof(spf_msg_header_t)));

      graph_mgmt_msg_ptr->sg_id_list.num_sub_graph = end_index - start_index;

      /** Get the start pointer to graph mgmt msg payload */
      uint8_t *msg_payload_start_ptr = (uint8_t *)(graph_mgmt_msg_ptr + 1);

      uint32_t sg_list_size_bytes = sizeof(uint32_t) * (end_index - start_index);

      uint8_t *sg_list_start = (uint8_t *)&proxy_sg_list_ptr[start_index];

      memscpy((void *)msg_payload_start_ptr, sg_list_size_bytes, (void *)sg_list_start, sg_list_size_bytes);

      graph_mgmt_msg_ptr->sg_id_list.sg_id_list_ptr = (uint32_t *)msg_payload_start_ptr;

      /** Push the message packet to container command Q */
      if (AR_EOK != (result = spf_msg_send_cmd(&proxy_msg, proxy_mgr_node->proxy_handle_ptr)))
      {
         AR_MSG(DBG_ERROR_PRIO, "Failed to send msg  to cont cmdQ, result: 0x%lx", result);

         spf_msg_return_msg(&proxy_msg);
         return result;
      }

      AR_MSG(DBG_HIGH_PRIO,
             "apm_proxy_util_send_mgmt_command_to_proxy_managers: Sent command to Proxy Manager."
             " cmd ID 0x%08X, proxy_instance  0x%08X",
             msg_opcode,
             proxy_mgr_node->proxy_instance_id);

      /** Update command control and response control. */
      curr_proxy_cmd_ctrl->rsp_ctrl.rsp_pending       = TRUE;
      curr_proxy_cmd_ctrl->rsp_ctrl.rsp_result        = AR_EOK;
      curr_proxy_cmd_ctrl->rsp_ctrl.reuse_rsp_msg_buf = FALSE;

      /** Increment the number of commands issues */
      apm_cmd_ctrl_ptr->rsp_ctrl.num_proxy_cmd_issued++;
      apm_cmd_ctrl_ptr->rsp_ctrl.cmd_rsp_pending    = CMD_RSP_PENDING;
      apm_cmd_ctrl_ptr->rsp_ctrl.proxy_resp_pending = CMD_RSP_PENDING;

      proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;
   }

   return result;
}

static ar_result_t apm_populate_proxy_mgr_cmd_close_seq(apm_proxy_msg_opcode_t *proxy_msg_opcode_ptr,
                                                        apm_sub_graph_state_t   sub_graph_list_state)
{
   ar_result_t result = AR_EOK;
   if (APM_SG_STATE_STOPPED == sub_graph_list_state)
   {
      proxy_msg_opcode_ptr->proxy_opcode_list[0] = SPF_MSG_CMD_GRAPH_CLOSE;
   }
   else if ((APM_SG_STATE_STARTED == sub_graph_list_state) || (APM_SG_STATE_PREPARED == sub_graph_list_state))
   {
      proxy_msg_opcode_ptr->num_proxy_msg_opcode = 2;
      proxy_msg_opcode_ptr->proxy_opcode_list[0] = SPF_MSG_CMD_GRAPH_STOP;
      proxy_msg_opcode_ptr->proxy_opcode_list[1] = SPF_MSG_CMD_GRAPH_CLOSE;
   }
   else
   {
      result = AR_EFAILED;
   }
   return result;
}

static ar_result_t apm_populate_proxy_mgr_cmd_spf_set_cfg_seq(apm_cmd_ctrl_t         *apm_cmd_ctrl_ptr,
                                                              apm_proxy_msg_opcode_t *proxy_msg_opcode_ptr,
                                                              apm_sub_graph_state_t   sub_graph_list_state)
{
   ar_result_t result = AR_EOK;
   switch (apm_cmd_ctrl_ptr->cmd_seq.set_get_cfg_cmd_seq.op_idx)
   {
      case APM_SET_GET_CFG_CMD_OP_SEND_CONT_MSG:
      case APM_SET_GET_CFG_CMD_OP_SEND_PROXY_MGR_MSG:
      {
         break;
      }
      case APM_SET_GET_CFG_CMD_OP_CLOSE_ALL:
      {
         result = apm_populate_proxy_mgr_cmd_close_seq(proxy_msg_opcode_ptr, sub_graph_list_state);
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_populate_proxy_mgr_cmd_spf_set_cfg_seq(): Un-support set get cmd operation index[%lu]",
                apm_cmd_ctrl_ptr->cmd_seq.set_get_cfg_cmd_seq.op_idx);
      }
   }
   return result;
}

ar_result_t apm_populate_proxy_mgr_cmd_seq(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr)
{
   ar_result_t             result = AR_EOK;
   apm_proxy_msg_opcode_t *proxy_msg_opcode_ptr;
   apm_sub_graph_state_t   sub_graph_list_state = APM_SG_STATE_INVALID;

   /** Validate input params. */
   if (NULL == apm_cmd_ctrl_ptr)
   {
      return AR_EBADPARAM;
   }

   /** Get the pointer to APM command control */
   proxy_msg_opcode_ptr = &apm_cmd_ctrl_ptr->proxy_msg_opcode;

   /** Clear the proxy cmd seq struct */
   memset(proxy_msg_opcode_ptr, 0, sizeof(apm_proxy_msg_opcode_t));

   /** For graph management command, get the current sub-graph
    *  list overall state */
   if (apm_is_graph_mgmt_cmd_opcode(apm_cmd_ctrl_ptr->cmd_opcode))
   {
      /** Get the state of sub-graph list being processed */
      sub_graph_list_state = apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.proxy_sg_list_state;
   }

   /** Set the default num opcode */
   proxy_msg_opcode_ptr->num_proxy_msg_opcode = 1;

   /** Reset the current command index being procesesd */
   proxy_msg_opcode_ptr->proxy_opcode_idx = 0;

   switch (apm_cmd_ctrl_ptr->cmd_opcode)
   {
      case APM_CMD_GRAPH_OPEN:
      {
         switch (apm_cmd_ctrl_ptr->cmd_seq.graph_open_seq.op_idx)
         {
            case APM_OPEN_CMD_OP_HDL_DB_QUERY_PREPROCESS:
            case APM_OPEN_CMD_OP_HDL_DB_QUERY_SEND_INFO:
            {

               break;
            }
            default:
            {
               proxy_msg_opcode_ptr->proxy_opcode_list[0] = SPF_MSG_CMD_PROXY_GRAPH_INFO;
               break;
            }
         }
         break;
      }
      case APM_CMD_GRAPH_PREPARE:
      {
         proxy_msg_opcode_ptr->proxy_opcode_list[0] = SPF_MSG_CMD_GRAPH_PREPARE;
         break;
      }
      case APM_CMD_GRAPH_STOP:
      {
         proxy_msg_opcode_ptr->proxy_opcode_list[0] = SPF_MSG_CMD_GRAPH_STOP;
         break;
      }
      case APM_CMD_GRAPH_START:
      {
         if (APM_SG_STATE_PREPARED == sub_graph_list_state)
         {
            proxy_msg_opcode_ptr->proxy_opcode_list[0] = SPF_MSG_CMD_GRAPH_START;
         }
         else if (APM_SG_STATE_STOPPED == sub_graph_list_state)
         {
            proxy_msg_opcode_ptr->num_proxy_msg_opcode = 2;
            proxy_msg_opcode_ptr->proxy_opcode_list[0] = SPF_MSG_CMD_GRAPH_PREPARE;
            proxy_msg_opcode_ptr->proxy_opcode_list[1] = SPF_MSG_CMD_GRAPH_START;
         }
         else
         {
            result = AR_EFAILED;
         }
         break;
      }
      case APM_CMD_GRAPH_CLOSE:
      case APM_CMD_CLOSE_ALL:
      {
         switch (apm_cmd_ctrl_ptr->cmd_seq.graph_mgmt_seq.op_idx)
         {
            case APM_GM_CMD_OP_DB_QUERY_SEND_INFO:
            {
               break;
            }
            default:
            {
               result = apm_populate_proxy_mgr_cmd_close_seq(proxy_msg_opcode_ptr, sub_graph_list_state);
               break;
            }
         }
         break;
      }
      case SPF_MSG_CMD_SET_CFG: // Close all can be called in this context for apm satellite
      {
         result =
            apm_populate_proxy_mgr_cmd_spf_set_cfg_seq(apm_cmd_ctrl_ptr, proxy_msg_opcode_ptr, sub_graph_list_state);
         break;
      }
      case SPF_MSG_CMD_GET_CFG:
      {
         break;
      }
      case APM_CMD_SET_CFG:
      {
         proxy_msg_opcode_ptr->proxy_opcode_list[0] = SPF_MSG_CMD_SET_CFG;
         break;
      }
      case APM_CMD_REGISTER_CFG:
      {
         proxy_msg_opcode_ptr->proxy_opcode_list[0] = SPF_MSG_CMD_REGISTER_CFG;
         break;
      }
      case APM_CMD_DEREGISTER_CFG:
      {
         proxy_msg_opcode_ptr->proxy_opcode_list[0] = SPF_MSG_CMD_DEREGISTER_CFG;
         break;
      }
      case APM_CMD_GET_CFG:
      {
         proxy_msg_opcode_ptr->proxy_opcode_list[0] = SPF_MSG_CMD_GET_CFG;
         break;
      }
      default:
      {
         result = AR_EFAILED;
         break;
      }
   } /** End of switch (cmd_opcode)*/

   return result;
}

ar_result_t apm_proxy_util_check_if_proxy_required(apm_cmd_ctrl_t   *apm_cmd_ctrl_ptr,
                                                   apm_graph_info_t *graph_info_ptr,
                                                   apm_sub_graph_t  *sg_node_ptr)
{
   ar_result_t result = AR_EOK;

   /** Validate input params. */
   if (NULL == graph_info_ptr || NULL == sg_node_ptr || NULL == apm_cmd_ctrl_ptr)
   {
      return AR_EBADPARAM;
   }

   switch (sg_node_ptr->prop.scenario_id)
   {
      case APM_SUB_GRAPH_SID_VOICE_CALL:
      {
         AR_MSG(DBG_HIGH_PRIO,
                "apm_proxy_util_check_if_proxy_required(): "
                " identified Voice Proxy for SG[0x%lx], scenario ID [0x%lx] ",
                sg_node_ptr->sub_graph_id,
                sg_node_ptr->prop.scenario_id);

         (void)apm_proxy_util_add_sg_to_proxy_cmd_ctrl(apm_cmd_ctrl_ptr,
                                                       &graph_info_ptr->proxy_manager_list_ptr,
                                                       &graph_info_ptr->num_proxy_managers,
                                                       sg_node_ptr);
      }
      break;

      default:
         AR_MSG(DBG_LOW_PRIO,
                "apm_proxy_util_check_if_proxy_required(): "
                " NO Proxy identified for SG[0x%lx]",
                sg_node_ptr->sub_graph_id);
         break;
   }

   return result;
}

ar_result_t apm_proxy_get_updated_sg_list_state(apm_sub_graph_state_t *curr_sg_list_state_ptr,
                                                apm_sub_graph_state_t  curr_sg_state,
                                                uint32_t               cmd_opcode)
{
   ar_result_t result = AR_EOK;

   switch (cmd_opcode)
   {
      case APM_CMD_GRAPH_START:
      {
         /** For START command, the expected current sub-graph state is
          *  PREPARED. However, start command could be issued to
          *  sub-graphs in STOPPED state as well. In this case, the state
          *  for the overall sub-graph list is set to STOPPED. The
          *  overall state is used by the container command sequencer to
          *  determine the lowest state from which to execute the
          *  overall START command sequence */

         *curr_sg_list_state_ptr = MIN(*curr_sg_list_state_ptr, curr_sg_state);

         break;
      }
      case APM_CMD_GRAPH_CLOSE:
      case APM_CMD_CLOSE_ALL:
      case SPF_MSG_CMD_SET_CFG:
      {
         /** For CLOSE command, the expected current sub-graph state is
          *  STOPPED. However, close command could be issued to
          *  sub-graphs in STARTED state as well. In this case, the state
          *  for the overall sub-graph list is set to STARTED.  The
          *  overall state is used by the container command sequencer to
          *  determine the highest state from which to execute the
          *  overall CLOSE command sequence */

         /* for SPF_MSG_CMD_SET_CFG, close all might be called in the satellite
          * side. This command can come when subgraphs are in STARTED state as well.
          * So we take the highest state */
         *curr_sg_list_state_ptr = MAX(*curr_sg_list_state_ptr, curr_sg_state);

         break;
      }
      default:
      {
         result = AR_EOK;
         break;
      }
   }

   return result;
}

ar_result_t apm_proxy_util_sort_graph_mgmt_sg_lists(apm_t *apm_info_ptr)
{
   ar_result_t              result               = AR_EOK;
   apm_proxysg_list_node_t *proxymgr_sg_list_ptr = NULL;
   spf_list_node_t         *proxy_mgr_list_ptr   = NULL;
   apm_sub_graph_t         *apm_sg_node_ptr;
   uint32_t                *temp_proxy_sg_list_ptr = NULL;
   uint32_t                 num_proxy_sgs_overall  = 0; // This is the count for proxy sg for all the proxy mgrs.
   spf_list_node_t         *curr_node_ptr, *next_node_ptr;
   apm_sub_graph_t         *sg_node_to_process_ptr;
   uint32_t                 num_sub_graphs;
   apm_sub_graph_state_t    sg_list_state            = APM_SG_STATE_INVALID;
   uint32_t                 num_sub_graphs_per_proxy = 0; // This is count for proxy sg per proxy mgr.

   /** Validate input params. */
   if (NULL == apm_info_ptr)
   {
      return AR_EBADPARAM;
   }

   apm_graph_info_t *graph_info_ptr   = &apm_info_ptr->graph_info;
   apm_cmd_ctrl_t   *apm_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   if (APM_SG_STATE_INVALID == (sg_list_state = apm_gm_cmd_get_default_sg_list_state(apm_cmd_ctrl_ptr->cmd_opcode)))
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_proxy_util_sort_graph_mgmt_sg_lists: Invalid sub-graph list state");

      return AR_EFAILED;
   }

   num_sub_graphs = apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_reg_sub_graphs;

   /** Allocate memory for temp subgraph lists for proxy and non-proxy subgraph lists. */
   if (NULL == (temp_proxy_sg_list_ptr =
                   (uint32_t *)posal_memory_malloc((sizeof(uint32_t) * num_sub_graphs), APM_INTERNAL_STATIC_HEAP_ID)))
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_proxy_util_sort_graph_mgmt_sg_lists: Failed to allocate scratch memory");

      return AR_ENOMEMORY;
   }

   proxy_mgr_list_ptr = graph_info_ptr->proxy_manager_list_ptr;

   while (proxy_mgr_list_ptr)
   {
      apm_proxy_manager_t *proxy_mgr_node = (apm_proxy_manager_t *)proxy_mgr_list_ptr->obj_ptr;

      num_sub_graphs_per_proxy = 0;

      /* Graph close can be done for voice sub-graphs prior to VSID configuration,
         but other graph mgmt cmds need to be skipped.
      */

      if ((APM_SUB_GRAPH_VSID_DONT_CARE == proxy_mgr_node->vcpm_properties.vsid) &&
          (APM_CMD_GRAPH_CLOSE != apm_cmd_ctrl_ptr->cmd_opcode) && (APM_CMD_CLOSE_ALL != apm_cmd_ctrl_ptr->cmd_opcode))
      {
         AR_MSG(DBG_HIGH_PRIO,
                "apm_proxy_util_sort_graph_mgmt_sg_lists: Skipping proxy mgr with default vsid 0x%08X",
                proxy_mgr_node->vcpm_properties.vsid);

         // If graph managment command is issued for the SG with invalid VSID, need to remove them from
         // regular sg list of apm current command ctrl.

         proxymgr_sg_list_ptr = proxy_mgr_node->proxy_sg_list_ptr;

         while (proxymgr_sg_list_ptr)
         {
            apm_sg_node_ptr = proxymgr_sg_list_ptr->apm_sg_node;

            /** Get the pointer to input list of sub-graphs to process */
            curr_node_ptr = apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.reg_sg_list_ptr;

            while (curr_node_ptr)
            {
               sg_node_to_process_ptr = (apm_sub_graph_t *)curr_node_ptr->obj_ptr;

               /** Set the next node pointer */
               next_node_ptr = curr_node_ptr->next_ptr;

               /** If the sub-graph ID matches with the proxy mgr manger SG
                *  ID, cached it and remove it from the regular list */
               if (sg_node_to_process_ptr->sub_graph_id == apm_sg_node_ptr->sub_graph_id)
               {
                  /** Remove this sub-graph node from the regular (non-proxy)
                   *  sub-graph list. This call also advances the list pointer.
                   *  Removal from regular sg list is required so that proxy managed sub-grpahs does not get processed
                   *  for proxy object getting skipped(invalid vsid). */

                  spf_list_find_delete_node(&apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.reg_sg_list_ptr,
                                            curr_node_ptr->obj_ptr,
                                            TRUE);

                  /** Decrement the number of regular (non-proxy) sub-graphs */
                  apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_reg_sub_graphs--;
               }

               /** Advance to next node in the list */
               curr_node_ptr = next_node_ptr;

            } /** End of inner while (input sg list) */

            proxymgr_sg_list_ptr = proxymgr_sg_list_ptr->next_ptr;

         } /** End of outer while (proxy mgr sub-graph list) */

         proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;
         continue;
      }

      apm_proxy_cmd_ctrl_t *curr_proxy_cmd_ctrl = NULL;

      /** Get/allocate a cmd contrl object from this Proxy Manager for processing current command. */
      result = apm_proxy_util_get_cmd_ctrl_obj(proxy_mgr_node, apm_cmd_ctrl_ptr, &curr_proxy_cmd_ctrl);

      if (AR_EOK != result || NULL == curr_proxy_cmd_ctrl)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_sort_graph_mgmt_sg_lists: Failed to allocate proxy_cmd_ctrl. result %d",
                result);
         return result;
      }

      curr_proxy_cmd_ctrl->cached_cfg_params.graph_mgmt_params.input_start_index = num_proxy_sgs_overall;

      proxymgr_sg_list_ptr = proxy_mgr_node->proxy_sg_list_ptr;

      while (proxymgr_sg_list_ptr)
      {
         apm_sg_node_ptr = proxymgr_sg_list_ptr->apm_sg_node;

         /** Get the pointer to input list of sub-graphs to process */
         curr_node_ptr = apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.reg_sg_list_ptr;

         while (curr_node_ptr)
         {
            sg_node_to_process_ptr = (apm_sub_graph_t *)curr_node_ptr->obj_ptr;

            /** Set the next node pointer */
            next_node_ptr = curr_node_ptr->next_ptr;

            /** If the sub-graph ID matches with the proxy mgr manger SG
             *  ID, cached it and remove it from the regular list */
            if (sg_node_to_process_ptr->sub_graph_id == apm_sg_node_ptr->sub_graph_id)
            {
               /** Add sg id to proxy cmd ctrl. */
               temp_proxy_sg_list_ptr[num_proxy_sgs_overall++] = sg_node_to_process_ptr->sub_graph_id;

               num_sub_graphs_per_proxy++;

               /** Add Proxy manger to apm cmd-Rsp ctrl pending list*/
               if (!curr_proxy_cmd_ctrl->rsp_ctrl.pending_msg_proc)
               {
                  apm_db_add_node_to_list(&apm_cmd_ctrl_ptr->rsp_ctrl.inactive_proxy_mgr_list_ptr,
                                          (void *)proxy_mgr_node,
                                          &apm_cmd_ctrl_ptr->rsp_ctrl.num_inactive_proxy_mgrs);
                  /** Set the pending flag  */
                  curr_proxy_cmd_ctrl->rsp_ctrl.pending_msg_proc = TRUE;
               }

               /** Remove this sub-graph node from the regular (non-proxy)
                *  sub-graph list. This call also advances the list pointer */
               spf_list_find_delete_node(&apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.reg_sg_list_ptr,
                                         curr_node_ptr->obj_ptr,
                                         TRUE);

               /** Decrement the number of regular (non-proxy) sub-graphs */
               apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_reg_sub_graphs--;

               apm_proxy_get_updated_sg_list_state(&sg_list_state,
                                                   sg_node_to_process_ptr->state,
                                                   apm_cmd_ctrl_ptr->cmd_opcode);
            }

            /** Advance to next node in the list */
            curr_node_ptr = next_node_ptr;

         } /** End of inner while (input sg list) */

         proxymgr_sg_list_ptr = proxymgr_sg_list_ptr->next_ptr;

      } /** End of outer while (proxy mgr sub-graph list) */

      curr_proxy_cmd_ctrl->cached_cfg_params.graph_mgmt_params.input_end_index = num_proxy_sgs_overall;

      proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;

      // As there is no proxy sg for this proxy mgr obj relase the allocated proxy mgr cmd ctrl.
      if (!num_sub_graphs_per_proxy)
      {
         AR_MSG(DBG_HIGH_PRIO,
                "apm_proxy_util_sort_graph_mgmt_sg_lists: releasing proxy mgr cmd ctrl with vsid 0x%08X as "
                "num_sub_graphs_per_proxy is %d",
                proxy_mgr_node->vcpm_properties.vsid,
                num_sub_graphs_per_proxy);

         apm_proxy_util_release_cmd_ctrl_obj(proxy_mgr_node, curr_proxy_cmd_ctrl);
      }

   } /** End of outermost while (proxy mgr list) */

   if (!num_proxy_sgs_overall && temp_proxy_sg_list_ptr)
   {
      /** There are no Proxy subgraphs. Free the temp memory created for proxy subgraph lists. */
      posal_memory_free(temp_proxy_sg_list_ptr);
      AR_MSG(DBG_MED_PRIO,
             "apm_proxy_util_sort_graph_mgmt_sg_lists: "
             "num_proxy_sgs_overall is %d",
             num_proxy_sgs_overall);
   }
   else /** Proxy sub-graphs present */
   {
      /** Cache the Proxy Subgraphs to be processed later.*/
      apm_info_ptr->curr_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_proxy_sub_graphs = num_proxy_sgs_overall;
      apm_info_ptr->curr_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.proxy_sg_id_list_ptr =
         (apm_sub_graph_id_t *)temp_proxy_sg_list_ptr;

      apm_info_ptr->curr_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.proxy_sub_graph_pending = TRUE;

      apm_info_ptr->curr_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.proxy_sg_list_state = sg_list_state;

      AR_MSG(DBG_HIGH_PRIO,
             "apm_proxy_util_sort_graph_mgmt_sg_lists: num_proxy_subgraphs: %lu, num regular subgraphs: %lu",
             num_proxy_sgs_overall,
             apm_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_reg_sub_graphs);
   }

   return result;
}

ar_result_t apm_proxy_util_validate_input_sg_list(spf_list_node_t    *proxy_manager_list_ptr,
                                                  apm_sub_graph_id_t *sg_array_ptr,
                                                  uint32_t            num_sgs)
{
   ar_result_t result      = AR_EOK;
   bool_t      match_found = FALSE;

   /** Validate input params. */
   if (NULL == proxy_manager_list_ptr || NULL == sg_array_ptr)
   {
      return AR_EBADPARAM;
   }

   for (uint32_t i = 0; i < num_sgs; i++)
   {
      match_found = FALSE;
      AR_MSG(DBG_HIGH_PRIO,
             "apm_proxy_util_validate_input_sg_list: Received Input "
             " SG ID 0x%08X, indx %d",
             sg_array_ptr[i].sub_graph_id,
             i);

      while (proxy_manager_list_ptr)
      {
         apm_proxy_manager_t *proxy_mgr = (apm_proxy_manager_t *)proxy_manager_list_ptr->obj_ptr;

         apm_proxysg_list_node_t *proxy_sg_list = proxy_mgr->proxy_sg_list_ptr;

         while (proxy_sg_list)
         {
            if (sg_array_ptr[i].sub_graph_id == proxy_sg_list->apm_sg_node->sub_graph_id)
            {
               match_found = TRUE;
               break;
            }

            proxy_sg_list = proxy_sg_list->next_ptr;
         }

         if (match_found)
         {
            break;
         }
         proxy_manager_list_ptr = proxy_manager_list_ptr->next_ptr;
      }

      if (FALSE == match_found)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_validate_input_sg_list: No Proxy entry found"
                " for SG ID 0x%08X, indx %d",
                sg_array_ptr[i].sub_graph_id,
                i);

         return AR_EBADPARAM;
      }
   }

   return result;
}

ar_result_t apm_proxy_util_send_cfg_to_proxy_mgrs(spf_handle_t     *apm_handle_ptr,
                                                  apm_cmd_ctrl_t   *apm_cmd_ctrl_ptr,
                                                  apm_graph_info_t *graph_info_ptr)
{
   ar_result_t               result              = AR_EOK;
   spf_list_node_t          *proxy_mgr_list_ptr  = NULL;
   apm_proxy_cmd_ctrl_t     *curr_proxy_cmd_ctrl = NULL;
   uint32_t                  num_ptr_objects     = 0;
   uint32_t                  msg_payload_size    = 0;
   spf_msg_t                 proxy_msg;
   uint8_t                  *set_cfg_data_start_ptr = NULL;
   apm_module_param_data_t **param_data_pptr, *param_data_ptr;
   uint32_t                  msg_opcode = 0;
   spf_msg_token_t           token;

   /** Validate input params. */
   if (NULL == apm_handle_ptr || NULL == apm_cmd_ctrl_ptr || NULL == apm_cmd_ctrl_ptr)
   {
      return AR_EBADPARAM;
   }

   switch (apm_cmd_ctrl_ptr->cmd_opcode)
   {
      // NOTE: pbm - TODO:Remove this. This should not be done here, opcodes to process should be filled before
      case APM_CMD_GRAPH_OPEN:
      {
         switch (apm_cmd_ctrl_ptr->cmd_seq.graph_open_seq.op_idx)
         {
            case APM_OPEN_CMD_OP_HDL_DB_QUERY_SEND_INFO:
            case APM_OPEN_CMD_OP_ERR_HDLR:
            {
               msg_opcode = SPF_MSG_CMD_SET_CFG;
               break;
            }
            default:
            {
               break;
            }
         }
         break;
      }
      case APM_CMD_GRAPH_CLOSE:
      {
         switch (apm_cmd_ctrl_ptr->cmd_seq.graph_mgmt_seq.op_idx)
         {
            case APM_GM_CMD_OP_DB_QUERY_SEND_INFO:
            {
               msg_opcode = SPF_MSG_CMD_SET_CFG;
               break;
            }
            default:
            {
               break;
            }
         }
         break;
      }
      case APM_CMD_SET_CFG:
      {
         msg_opcode = SPF_MSG_CMD_SET_CFG;
         break;
      }

      case APM_CMD_REGISTER_CFG:
      {
         msg_opcode = SPF_MSG_CMD_REGISTER_CFG;
         break;
      }

      case APM_CMD_DEREGISTER_CFG:
      {
         msg_opcode = SPF_MSG_CMD_DEREGISTER_CFG;
         break;
      }

      case APM_CMD_GET_CFG:
      {
         msg_opcode = SPF_MSG_CMD_GET_CFG;
         break;
      }

      default:
      {
         return AR_EOK;
      }
   }

   AR_MSG(DBG_HIGH_PRIO, "apm_proxy_util_send_cfg_to_proxy_mgrs:  Opcode 0x%08X", apm_cmd_ctrl_ptr->cmd_opcode);

   proxy_mgr_list_ptr = apm_cmd_ctrl_ptr->rsp_ctrl.active_proxy_mgr_list_ptr;

   while (proxy_mgr_list_ptr)
   {

      apm_proxy_manager_t *proxy_mgr_node = (apm_proxy_manager_t *)proxy_mgr_list_ptr->obj_ptr;
      uint32_t             arr_idx        = 0;
      result = apm_proxy_util_get_cmd_ctrl_obj(proxy_mgr_node, apm_cmd_ctrl_ptr, &curr_proxy_cmd_ctrl);

      if (AR_EOK != result || NULL == curr_proxy_cmd_ctrl)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_send_cfg_to_proxy_mgrs: Failed to allocate proxy_cmd_ctrl. result %d",
                result);
         return result;
      }

      num_ptr_objects = curr_proxy_cmd_ctrl->cached_cfg_params.proxy_cfg_params.num_mod_param_cfg;

      msg_payload_size = (sizeof(spf_msg_cmd_param_data_cfg_t) + (SIZE_OF_PTR() * num_ptr_objects));
      msg_payload_size = GET_SPF_MSG_REQ_SIZE(msg_payload_size);

      token.token_ptr = curr_proxy_cmd_ctrl;

      /** Create gk message for proxy manager */
      if (AR_EOK != (result = spf_msg_create_msg(&proxy_msg,                       /** MSG Ptr */
                                                 &msg_payload_size,                /** MSG payload size */
                                                 msg_opcode,                       /** MSG opcode */
                                                 apm_handle_ptr,                   /** APM response handle */
                                                 &token,                           /** MSG Token */
                                                 proxy_mgr_node->proxy_handle_ptr, /** Destination handle */
                                                 APM_INTERNAL_STATIC_HEAP_ID)))
      {
         AR_MSG(DBG_HIGH_PRIO,
                "FAILED to create msg payload, opcode: 0x%lx, proxy instance ID[0x%lX], result: 0x%lx",
                msg_opcode,
                proxy_mgr_node->scenario_id,
                result);

         return result;
      }

      spf_msg_header_t *msg_header_ptr = (spf_msg_header_t *)proxy_msg.payload_ptr;

      spf_msg_cmd_param_data_cfg_t *msg_set_cfg_ptr = (spf_msg_cmd_param_data_cfg_t *)&msg_header_ptr->payload_start;

      msg_set_cfg_ptr->num_param_id_cfg = curr_proxy_cmd_ctrl->cached_cfg_params.proxy_cfg_params.num_mod_param_cfg;

      /** Get the pointer to list of module list ptr */
      spf_list_node_t *curr_proxy_param_list =
         curr_proxy_cmd_ctrl->cached_cfg_params.proxy_cfg_params.param_data_list_ptr;

      /** Set the pointer to start of the array of pointers for
       *  param ID data payload */
      set_cfg_data_start_ptr = (uint8_t *)msg_set_cfg_ptr + sizeof(spf_msg_cmd_param_data_cfg_t);

      msg_set_cfg_ptr->param_data_pptr = (void **)set_cfg_data_start_ptr;

      /** Populate the msg payload. */
      while (curr_proxy_param_list)
      {
         param_data_ptr = (apm_module_param_data_t *)curr_proxy_param_list->obj_ptr;

         param_data_pptr = (apm_module_param_data_t **)msg_set_cfg_ptr->param_data_pptr;

         AR_MSG(DBG_HIGH_PRIO,
                "apm_proxy_util_send_cfg_to_proxy_mgrs: index %d, param_data_ptr 0x%lx",
                arr_idx,
                param_data_ptr);

         param_data_pptr[arr_idx++] = param_data_ptr;

         /** Iterate over the param data list */
         curr_proxy_param_list = curr_proxy_param_list->next_ptr;
      }

      if (curr_proxy_cmd_ctrl->cached_cfg_params.proxy_cfg_params.use_sys_q)
      {
         /** Push the message packet to Proxy manager's system command Q */
         if (AR_EOK != (result = spf_msg_send_sys_cmd(&proxy_msg, proxy_mgr_node->proxy_handle_ptr)))
         {
            AR_MSG(DBG_ERROR_PRIO, "Failed to send msg  to system cmd Q, result: 0x%lx", result);
            spf_msg_return_msg(&proxy_msg);
            return result;
         }
      }
      else
      {
         /** Push the message packet to Proxy manager's command Q */
         if (AR_EOK != (result = spf_msg_send_cmd(&proxy_msg, proxy_mgr_node->proxy_handle_ptr)))
         {
            AR_MSG(DBG_ERROR_PRIO, "Failed to send msg to proxy cmd Q, result: 0x%lx", result);
            spf_msg_return_msg(&proxy_msg);
            return result;
         }
      }

      AR_MSG(DBG_HIGH_PRIO,
             "apm_proxy_util_send_cfg_to_proxy_mgrs: Sent command to Proxy Manager."
             " cmd ID 0x%08X, proxy_instance  0x%08X, num_params %d",
             msg_opcode,
             proxy_mgr_node->proxy_instance_id,
             arr_idx);

      /** Update command ctronl and response control. */
      curr_proxy_cmd_ctrl->rsp_ctrl.rsp_pending       = TRUE;
      curr_proxy_cmd_ctrl->rsp_ctrl.rsp_result        = AR_EOK;
      curr_proxy_cmd_ctrl->rsp_ctrl.reuse_rsp_msg_buf = FALSE;

      /** Increment the number of commands issues */
      apm_cmd_ctrl_ptr->rsp_ctrl.num_proxy_cmd_issued++;
      apm_cmd_ctrl_ptr->rsp_ctrl.cmd_rsp_pending    = CMD_RSP_PENDING;
      apm_cmd_ctrl_ptr->rsp_ctrl.proxy_resp_pending = CMD_RSP_PENDING;

      proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;
   }

   return result;
}

ar_result_t apm_gm_cmd_handle_proxy_sub_graph_list(apm_t *apm_info_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *apm_cmd_ctrl_ptr;
   uint32_t        cmd_opcode;

   /** Get the pointer to APM command ctrl object */
   apm_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Get the current command opcode being processed */
   cmd_opcode = apm_cmd_ctrl_ptr->cmd_opcode;

   /** If there are any sub-graphs present managed via proxy
    *  manager, send the graph management command to them. Proxy
    *  manager are only expected to  provide permission for
    *  given command opcode if they are in the right state to be
    *  processed. Proxy manager returns back the response with
    *  the list of sub-graphs that can be operated upon. This
    *  sub-graph list can be empty as well. */

   /** Send command to Proxy Managers.*/
   if (AR_EOK != (result = apm_proxy_util_send_mgmt_command_to_proxy_managers(apm_info_ptr->curr_cmd_ctrl_ptr,
                                                                              &apm_info_ptr->handle)))
   {
      AR_MSG(DBG_ERROR_PRIO, "GRAPH_MGMT: Failed to send command to Proxy managers, cmd_opcode[0x%08lx]", cmd_opcode);

      return result;
   }

   /** Check if the spf_msg command response is pending. Flag
    *  error if not pending */
   if (!apm_cmd_ctrl_ptr->rsp_ctrl.cmd_rsp_pending)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "GRAPH_MGMT: Rsp pending flag is not set after sending msg to proxy mgrs, cmd_opcode[0x%08lx]",
             cmd_opcode);

      result = AR_EFAILED;
   }

   return result;
}

ar_result_t apm_proxy_graph_open_sequencer(apm_t *apm_info_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *cmd_ctrl_ptr;
   apm_op_seq_t   *curr_op_seq_ptr;

   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Get the current op seq obj pointer   */
   curr_op_seq_ptr = cmd_ctrl_ptr->cmd_seq.curr_op_seq_ptr;

   if (!cmd_ctrl_ptr->rsp_ctrl.inactive_proxy_mgr_list_ptr && !curr_op_seq_ptr->curr_cmd_op_pending)
   {
      AR_MSG(DBG_ERROR_PRIO, "apm_proxy_graph_open_sequencer(): no inactive proxies pending processing");
      return AR_EOK;
   }

   AR_MSG(DBG_MED_PRIO,
          "apm_proxy_graph_open_sequencer(): cmd_opcode: 0x%08lx curr_seq_idx: %d",
          cmd_ctrl_ptr->cmd_opcode,
          curr_op_seq_ptr->curr_seq_idx);

   apm_init_next_cmd_op_seq_idx(curr_op_seq_ptr, APM_SEQ_SET_UP_PROXY_MGR_MSG_SEQ);

   switch (curr_op_seq_ptr->curr_seq_idx)
   {
      case APM_SEQ_SET_UP_PROXY_MGR_MSG_SEQ:
      {

         /** Populate the container command sequence as per command
          *  opcode and current sub-graph state */
         if (AR_EOK != (result = apm_populate_proxy_mgr_cmd_seq(cmd_ctrl_ptr)))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "apm_proxy_graph_open_sequencer(): Failed to populate cont cmd seq,"
                   " cmd_opcode[0x%08lx]",
                   cmd_ctrl_ptr->cmd_opcode);

            return result;
         }
         // Only vcpm uses this command
         apm_move_proxy_to_active_list_by_id(apm_info_ptr, VCPM_MODULE_INSTANCE_ID);

         /** Set up next sequence */
         curr_op_seq_ptr->curr_seq_idx = APM_SEQ_SEND_OPEN_CMD_TO_PROXY_MGR;

         break;
      }
      case APM_SEQ_SEND_OPEN_CMD_TO_PROXY_MGR:
      {
         result = apm_proxy_util_send_graph_info_to_proxy_managers(cmd_ctrl_ptr, &apm_info_ptr->handle);

         /** Set up next sequence */
         curr_op_seq_ptr->curr_seq_idx = APM_SEQ_SEND_MSG_TO_PROXY_MGR_COMPLETED;

         break;
      }
      case APM_SEQ_SEND_MSG_TO_PROXY_MGR_COMPLETED:
      {
         /** Cache the op status   */
         result = curr_op_seq_ptr->status;

         apm_clear_curr_cmd_op_pending_status(curr_op_seq_ptr);

         break;
      }
      default:
      {
         break;
      }

   } /** End of switch (cmd_ctrl_ptr->cmd_seq.curr_seq_idx) */

   return result;
}

ar_result_t apm_proxy_graph_mgmt_sequencer(apm_t *apm_info_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *cmd_ctrl_ptr;
   apm_op_seq_t   *curr_op_seq_ptr;

   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Nothing to if list of proxy managed sub-graphs is empty  */
   if (!apm_info_ptr->curr_cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_proxy_sub_graphs)
   {
      return AR_EOK;
   }

   /** Get the current op sequencer pointer */
   curr_op_seq_ptr = cmd_ctrl_ptr->cmd_seq.curr_op_seq_ptr;

   apm_init_next_cmd_op_seq_idx(curr_op_seq_ptr, APM_SEQ_SET_UP_PROXY_MGR_MSG_SEQ);

   switch (curr_op_seq_ptr->curr_seq_idx)
   {
      case APM_SEQ_SET_UP_PROXY_MGR_MSG_SEQ:
      {

         /** Populate the Proxy command sequence as per command
          *  opcode and current sub-graph state. Do it only if the
          *  processing has not started already. */

         if (AR_EOK != (result = apm_populate_proxy_mgr_cmd_seq(cmd_ctrl_ptr)))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "GRAPH_MGMT: Failed to populate cont cmd seq, cmd_opcode[0x%08lx]",
                   cmd_ctrl_ptr->cmd_opcode);

            return result;
         }

         // Only vcpm uses this command
         apm_move_proxy_to_active_list_by_id(apm_info_ptr, VCPM_MODULE_INSTANCE_ID);

         /** Set next sequencer index */
         curr_op_seq_ptr->curr_seq_idx = APM_SEQ_SEEK_PROXY_MGR_PERMISSION;

         break;
      }
      case APM_SEQ_SEEK_PROXY_MGR_PERMISSION:
      {
         result = apm_gm_cmd_handle_proxy_sub_graph_list(apm_info_ptr);

         /** Set next sequencer index */
         curr_op_seq_ptr->curr_seq_idx = APM_SEQ_VALIDATE_SG_LIST;

         break;
      }
      case APM_SEQ_VALIDATE_SG_LIST:
      case APM_SEQ_SET_UP_CONT_MSG_SEQ:
      case APM_SEQ_PREPROC_GRAPH_MGMT_MSG:
      case APM_SEQ_PREPARE_FOR_NEXT_CONT_MSG:
      case APM_SEQ_SEND_MSG_TO_CONTAINERS:
      case APM_SEQ_CONT_SEND_MSG_COMPLETED:
      case APM_SEQ_REG_SG_PROC_COMPLETED:
      {
         /** If permisison granted */
         if (cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.reg_sg_list_ptr)
         {
            result = apm_cmd_graph_mgmt_cmn_sequencer(apm_info_ptr);

            if (APM_SEQ_REG_SG_PROC_COMPLETED == curr_op_seq_ptr->curr_seq_idx)
            {
               /** Set next sequencer index */
               curr_op_seq_ptr->curr_seq_idx = APM_SEQ_PROXY_MGR_SG_PROC_COMPLETED;
            }
         }
         else /** No permission, end current operation */
         {
            /** Set next sequencer index */
            curr_op_seq_ptr->curr_seq_idx = APM_SEQ_PROXY_MGR_SG_PROC_COMPLETED;
         }

         break;
      }
      case APM_SEQ_PROXY_MGR_SG_PROC_COMPLETED:
      {
         cmd_ctrl_ptr->proxy_msg_opcode.proxy_opcode_idx++;

         if (cmd_ctrl_ptr->proxy_msg_opcode.proxy_opcode_idx == cmd_ctrl_ptr->proxy_msg_opcode.num_proxy_msg_opcode)
         {
            curr_op_seq_ptr->curr_cmd_op_pending = FALSE;

            cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.proxy_sub_graph_pending = FALSE;

            /** Free up the memory allocated for the sub-graphs managed via
             *  proxy manager  */
            if (cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.proxy_sg_id_list_ptr)
            {
               /** This is a temporary cached memory created at
                *  the beginning of Graph Management command processing.
                *  Shall free it now.*/
               posal_memory_free(cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.proxy_sg_id_list_ptr);

               /** Clear the sub-graph list pointer and counter */
               cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.proxy_sg_id_list_ptr = NULL;
               cmd_ctrl_ptr->graph_mgmt_cmd_ctrl.sg_list.num_proxy_sub_graphs = 0;
            }
         }
         else
         {
            /** Set next sequencer index */
            curr_op_seq_ptr->curr_seq_idx = APM_SEQ_SEEK_PROXY_MGR_PERMISSION;
         }

         break;
      }
      default:
      {
         break;
      }

   } /** End of switch (cmd_ctrl_ptr->cmd_seq.curr_seq_idx) */

   return result;
}

ar_result_t apm_proxy_set_cfg_sequencer(apm_t *apm_info_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *cmd_ctrl_ptr;
   apm_op_seq_t   *curr_op_seq_ptr;

   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Get the current op sequencer pointer */
   curr_op_seq_ptr = cmd_ctrl_ptr->cmd_seq.curr_op_seq_ptr;

   /* We only handle this sequencer in case there are active proxies, and all active proxies are handled at once. In the
    * case of a set/get cfg coming to apm, all the proxies will be activated at parse time. In case of graph management,
    * set/get cfg proixies will be individually activated as needed when their respective cmd is generated */
   if (!(cmd_ctrl_ptr->rsp_ctrl.active_proxy_mgr_list_ptr) && !curr_op_seq_ptr->curr_cmd_op_pending)
   {
      return AR_EOK;
   }

   apm_init_next_cmd_op_seq_idx(curr_op_seq_ptr, APM_SEQ_SET_UP_PROXY_MGR_MSG_SEQ);

   switch (curr_op_seq_ptr->curr_seq_idx)
   {
      case APM_SEQ_SET_UP_PROXY_MGR_MSG_SEQ:
      {
         /** Populate the Proxy command sequence as per command
          *  opcode and current sub-graph state. Do it only if the
          *  processing has not started already. */
         if (AR_EOK != (result = apm_populate_proxy_mgr_cmd_seq(cmd_ctrl_ptr)))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "apm_proxy_set_cfg_sequencer(): Failed to populate cont cmd seq, cmd_opcode[0x%08lx]",
                   cmd_ctrl_ptr->cmd_opcode);

            return result;
         }

         /** Set next sequencer index */
         curr_op_seq_ptr->curr_seq_idx = APM_SEQ_SEND_CFG_CMD_TO_PROXY_MGR;

         break;
      }
      case APM_SEQ_SEND_CFG_CMD_TO_PROXY_MGR:
      {
         result = apm_proxy_util_send_cfg_to_proxy_mgrs(&apm_info_ptr->handle,
                                                        apm_info_ptr->curr_cmd_ctrl_ptr,
                                                        &apm_info_ptr->graph_info);

         /** If no response pending, clear the operation pending flag */
         if (!cmd_ctrl_ptr->rsp_ctrl.cmd_rsp_pending)
         {
            apm_clear_curr_cmd_op_pending_status(curr_op_seq_ptr);
         }
         else
         {
            curr_op_seq_ptr->curr_seq_idx = APM_SEQ_SEND_MSG_TO_PROXY_MGR_COMPLETED;
         }

         break;
      }
      case APM_SEQ_SEND_MSG_TO_PROXY_MGR_COMPLETED:
      {
         /** Cache the current op status   */
         result = curr_op_seq_ptr->status;

         /** Clear the command op pending status   */
         apm_clear_curr_cmd_op_pending_status(curr_op_seq_ptr);

         break;
      }
      default:
      {
         break;
      }

   } /** End of switch (cmd_ctrl_ptr->cmd_seq.curr_seq_idx) */

   return result;
}

ar_result_t apm_proxy_graph_open_cmn_sequencer(apm_t *apm_info_ptr)
{
   ar_result_t     result = AR_EOK;
   apm_cmd_ctrl_t *cmd_ctrl_ptr;
   apm_op_seq_t   *curr_op_seq_ptr;

   /** Get the current command control object pointer  */
   cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** Get the current op seq obj pointer   */
   curr_op_seq_ptr = cmd_ctrl_ptr->cmd_seq.curr_op_seq_ptr;

   /** Switch to handler function corresponding to current
    *  sequener index configured  */
   switch (curr_op_seq_ptr->op_idx)
   {
      case APM_OPEN_CMD_OP_PROXY_MGR_OPEN:
      {
         result = apm_proxy_graph_open_sequencer(apm_info_ptr);

         break;
      }
      case APM_OPEN_CMD_PROXY_MGR_PREPROCESS:
      {
         /* In cases where there is a set-cfg in band with the graph open, there will be inactive proxies pending
          * processing. Since VCPM will have been handled at this point we can safely activate ALL remaning inactive
          * proxies for processing. */
         result = apm_move_proxies_to_active_list(apm_info_ptr);
         break;
      }
      case APM_OPEN_CMD_OP_PROXY_MGR_CFG:
      {
         result = apm_proxy_set_cfg_sequencer(apm_info_ptr);

         break;
      }
      default:
      {
         break;
      }

   } /** End of switch (cmd_ctrl_ptr->cmd_seq.curr_open_cmd_op_idx) */

   return result;
}

ar_result_t apm_clear_permitted_proxy_list(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr)
{
   ar_result_t           result = AR_EOK;
   spf_list_node_t      *curr_node_ptr;
   apm_proxy_manager_t  *proxy_mgr_ptr;
   apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl_ptr;

   /** Validate input params. */
   if (NULL == apm_cmd_ctrl_ptr)
   {
      return AR_EBADPARAM;
   }

   /** Get the pointer to list of containers pending send
    *  message */
   curr_node_ptr = apm_cmd_ctrl_ptr->rsp_ctrl.permitted_proxy_mgr_list_ptr;

   /** Iterate over the container list */
   while (curr_node_ptr)
   {
      proxy_mgr_ptr = (apm_proxy_manager_t *)curr_node_ptr->obj_ptr;

      /** Get the container's command control object corresponding
       *  to current APM command control object */
      apm_proxy_util_get_allocated_cmd_ctrl_obj(proxy_mgr_ptr, apm_cmd_ctrl_ptr, &proxy_cmd_ctrl_ptr);

      if (NULL != proxy_cmd_ctrl_ptr)
      {
         if (NULL != proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr)
         {
            posal_memory_free(proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr);
            proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr = NULL;
         }
      }

      //      proxy_cmd_ctrl_ptr->rsp_ctrl.num_permitted_subgraphs = 0;

      /** Free up the list node memory and
          advance to next node in the list */
      spf_list_delete_node(&curr_node_ptr, TRUE /*pool_used */);
   }

   /** Clear the pending container list pointer */
   apm_cmd_ctrl_ptr->rsp_ctrl.permitted_proxy_mgr_list_ptr = NULL;

   /** Clear the number of pending containers */
   apm_cmd_ctrl_ptr->rsp_ctrl.num_permitted_proxy_mgrs = 0;

   return result;
}
ar_result_t apm_proxy_util_remove_closed_subgraphs_from_proxy_mgrs(apm_t *apm_info_ptr)
{
   ar_result_t           result         = AR_EOK;
   apm_proxy_manager_t  *proxy_mgr_node = NULL;
   bool_t                found_match    = FALSE;
   apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl_ptr;

   /** Validate input params. */
   if (NULL == apm_info_ptr)
   {
      return AR_EBADPARAM;
   }

   apm_cmd_ctrl_t *apm_curr_cmd_ctrl = apm_info_ptr->curr_cmd_ctrl_ptr;

   apm_graph_info_t *graph_info_ptr = &apm_info_ptr->graph_info;

   spf_list_node_t *proxy_mgr_list_ptr = apm_curr_cmd_ctrl->rsp_ctrl.permitted_proxy_mgr_list_ptr;

   while (proxy_mgr_list_ptr)
   {
      proxy_mgr_node = (apm_proxy_manager_t *)proxy_mgr_list_ptr->obj_ptr;

      apm_proxy_util_get_allocated_cmd_ctrl_obj(proxy_mgr_node, apm_curr_cmd_ctrl, &proxy_cmd_ctrl_ptr);

      if (NULL == proxy_cmd_ctrl_ptr)
      {

         /** Advance to the next Proxy Manager. */
         proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;
         continue;
      }

      apm_sub_graph_id_t *permitted_sg_array_ptr  = proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr;
      uint32_t            num_permitted_subgraphs = proxy_cmd_ctrl_ptr->rsp_ctrl.num_permitted_subgraphs;

      AR_MSG(DBG_HIGH_PRIO,
             "apm_proxy_util_remove_closed_subgraphs_from_proxy_mgrs: Proxy Manager InstanceID 0x%08X, "
             "num_permitted_subgraphs to be removed  %d ",
             proxy_mgr_node->scenario_id,
             num_permitted_subgraphs);

      apm_proxysg_list_node_t *proxy_sg_list_ptr = proxy_mgr_node->proxy_sg_list_ptr;
      apm_proxysg_list_node_t *temp_ptr;

      while (proxy_sg_list_ptr)
      {
         apm_sub_graph_t *sg_node = proxy_sg_list_ptr->apm_sg_node;
         found_match              = FALSE;

         for (uint32_t i = 0; i < num_permitted_subgraphs; i++)
         {
            if (permitted_sg_array_ptr[i].sub_graph_id == sg_node->sub_graph_id)
            {
               AR_MSG(DBG_HIGH_PRIO,
                      "apm_proxy_util_remove_closed_subgraphs_from_proxy_mgrs:"
                      " Remove SG ID 0x%08X ",
                      sg_node->sub_graph_id);

               found_match = TRUE;
               break;
            }
         }

         temp_ptr = proxy_sg_list_ptr->next_ptr;

         if (found_match)
         {
            /** Remove subgraph node from the subgraph list. */

            if (NULL != proxy_sg_list_ptr->prev_ptr)
            {
               proxy_sg_list_ptr->prev_ptr->next_ptr = proxy_sg_list_ptr->next_ptr;
            }
            else
            {
               proxy_mgr_node->proxy_sg_list_ptr = proxy_sg_list_ptr->next_ptr;
            }

            if (NULL != proxy_sg_list_ptr->next_ptr)
            {
               proxy_sg_list_ptr->next_ptr->prev_ptr = proxy_sg_list_ptr->prev_ptr;
            }

            posal_memory_free(proxy_sg_list_ptr);

            proxy_mgr_node->num_proxy_subgraphs--;
         }
         /** Move to next node.*/
         proxy_sg_list_ptr = temp_ptr;
      }

      /** Advance to the next Proxy Manager. */
      proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;

      if (NULL == proxy_mgr_node->proxy_sg_list_ptr)
      {

         if (NULL != proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr)
         {
            posal_memory_free(proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr);
         }

         /** Remove proxy Manager from pending rsp ctrl list. */
         apm_db_remove_node_from_list(&apm_curr_cmd_ctrl->rsp_ctrl.permitted_proxy_mgr_list_ptr,
                                      proxy_mgr_node,
                                      &apm_curr_cmd_ctrl->rsp_ctrl.num_permitted_proxy_mgrs);

         /** Remove proxy Manager from pending rsp ctrl list. */
         apm_db_remove_node_from_list(&apm_curr_cmd_ctrl->rsp_ctrl.active_proxy_mgr_list_ptr,
                                      proxy_mgr_node,
                                      &apm_curr_cmd_ctrl->rsp_ctrl.num_active_proxy_mgrs);

         /** Remove Proxy Manager node from apm graph info list. */
         apm_db_remove_node_from_list(&graph_info_ptr->proxy_manager_list_ptr,
                                      proxy_mgr_node,
                                      &graph_info_ptr->num_proxy_managers);

         /** Free the memory allocated to proxy manager node.*/
         posal_memory_free(proxy_mgr_node);
      }
   }

   return result;
}

// VCPM
ar_result_t apm_proxy_util_update_proxy_manager(apm_cmd_ctrl_t *apm_cmd_ctrl_ptr)
{
   ar_result_t result = AR_EOK;

   spf_list_node_t      *curr_node_ptr;
   apm_proxy_manager_t  *proxy_mgr_ptr;
   apm_proxy_cmd_ctrl_t *curr_proxy_cmd_ctrl;

   /** Validate input params. */
   if (NULL == apm_cmd_ctrl_ptr)
   {
      return AR_EBADPARAM;
   }

   /** Get the pointer to list of containers pending send
    *  message */
   curr_node_ptr = apm_cmd_ctrl_ptr->rsp_ctrl.active_proxy_mgr_list_ptr;

   /** Iterate over the container list */
   while (curr_node_ptr)
   {
      proxy_mgr_ptr = (apm_proxy_manager_t *)curr_node_ptr->obj_ptr;

      /** Get the container's command control object corresponding
       *  to current APM command control object */
      apm_proxy_util_get_allocated_cmd_ctrl_obj(proxy_mgr_ptr, apm_cmd_ctrl_ptr, &curr_proxy_cmd_ctrl);

      if (NULL != curr_proxy_cmd_ctrl)
      {
         apm_proxy_graph_open_params_t *proxy_open_params = &curr_proxy_cmd_ctrl->cached_cfg_params.graph_open_params;

         spf_list_node_t *cached_sg_list = proxy_open_params->sg_list_ptr;

         while (cached_sg_list)
         {

            apm_proxysg_list_node_t *proxy_sg_node_ptr = NULL;
            /** ALlocate memory for Proxy subgraph list node for APM graph DB */
            if (NULL ==
                (proxy_sg_node_ptr = (apm_proxysg_list_node_t *)posal_memory_malloc(sizeof(apm_proxysg_list_node_t),
                                                                                    APM_INTERNAL_STATIC_HEAP_ID)))
            {
               AR_MSG(DBG_ERROR_PRIO,
                      "apm_proxy_util_update_proxy_manager: Failed to allocat Proxy Manager node mem, SG_ID: [0x%lX]",
                      ((apm_sub_graph_t *)(cached_sg_list->obj_ptr))->sub_graph_id);

               return AR_ENOMEMORY;
            }
            /** Initiate node. */
            proxy_sg_node_ptr->apm_sg_node = (apm_sub_graph_t *)cached_sg_list->obj_ptr;

            proxy_sg_node_ptr->next_ptr = NULL;
            proxy_sg_node_ptr->prev_ptr = NULL;

            if (NULL == proxy_mgr_ptr->proxy_sg_list_ptr)
            {
               /** Add new node as first node in the list.*/
               proxy_mgr_ptr->proxy_sg_list_ptr = proxy_sg_node_ptr;
            }
            else
            {
               /** Add new node at the tail. */
               apm_proxysg_list_node_t *temp_node = proxy_mgr_ptr->proxy_sg_list_ptr;
               while (NULL != temp_node->next_ptr)
               {
                  temp_node = temp_node->next_ptr;
               }
               temp_node->next_ptr         = proxy_sg_node_ptr;
               proxy_sg_node_ptr->prev_ptr = temp_node;
            }

            AR_MSG(DBG_MED_PRIO,
                   "apm_proxy_util_update_proxy_manager: Added Scenario ID [0x%lX] , SG ID :[0x%lX] to Proxy Manager",
                   proxy_mgr_ptr->scenario_id,
                   ((apm_sub_graph_t *)(cached_sg_list->obj_ptr))->sub_graph_id);

            proxy_mgr_ptr->num_proxy_subgraphs++;

            cached_sg_list = cached_sg_list->next_ptr;
         }
      }

      curr_node_ptr = curr_node_ptr->next_ptr;
   }

   return result;
}

ar_result_t apm_proxy_util_process_pending_subgraphs(apm_t *apm_info_ptr, apm_cmd_ctrl_t *apm_curr_cmd_ctrl)
{
   ar_result_t           result         = AR_EOK;
   apm_proxy_manager_t  *proxy_mgr_node = NULL;
   apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl_ptr;
   uint16_t              num_sgs_to_process = 0;
   // uint32_t              mem_size           = 0;
   // apm_sub_graph_id_t *  temp_sg_array_ptr  = NULL;
   // uint8_t *             temp_ptr           = NULL;
   // uint32_t              temp_size          = 0;
   uint32_t            cmd_opcode;
   apm_sub_graph_id_t *sg_id_list_ptr;

   /** Validate input params. */
   if (NULL == apm_info_ptr || NULL == apm_curr_cmd_ctrl)
   {
      return AR_EBADPARAM;
   }

   /** Get the current command opcode under process */
   cmd_opcode = apm_curr_cmd_ctrl->cmd_opcode;

   spf_list_node_t *proxy_mgr_list_ptr = apm_curr_cmd_ctrl->rsp_ctrl.permitted_proxy_mgr_list_ptr;

   if (0 == apm_curr_cmd_ctrl->rsp_ctrl.num_permitted_proxy_mgrs)
   {
      /** No Proxy subgraphs to be processed. UNEXPECTED!! return from here. */
      AR_MSG(DBG_ERROR_PRIO,
             "apm_proxy_util_process_pending_subgraphs: UNEXPECTED!!!,"
             " cmd_opcode[0x%08lx]",
             apm_curr_cmd_ctrl->cmd_opcode);

      return AR_EUNEXPECTED;
   }

   AR_MSG(DBG_HIGH_PRIO,
          "apm_proxy_util_process_pending_subgraphs: num proxy mgrs %d,"
          " cmd_opcode[0x%08lx]",
          apm_curr_cmd_ctrl->rsp_ctrl.num_permitted_proxy_mgrs,
          apm_curr_cmd_ctrl->cmd_opcode);

   proxy_mgr_list_ptr = apm_curr_cmd_ctrl->rsp_ctrl.permitted_proxy_mgr_list_ptr;

   while (proxy_mgr_list_ptr)
   {
      proxy_mgr_node = (apm_proxy_manager_t *)proxy_mgr_list_ptr->obj_ptr;

      apm_proxy_util_get_allocated_cmd_ctrl_obj(proxy_mgr_node, apm_curr_cmd_ctrl, &proxy_cmd_ctrl_ptr);

      if (!proxy_cmd_ctrl_ptr)
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_proxy_util_process_pending_subgraphs:: Failed to get cmd ctrl obj, cmd_opcode[0x%lX]",
                apm_curr_cmd_ctrl->cmd_opcode);

         return AR_EFAILED;
      }

      sg_id_list_ptr = proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr;

      num_sgs_to_process = proxy_cmd_ctrl_ptr->rsp_ctrl.num_permitted_subgraphs;

      /** Validate sub-graph ID list being processed. This call
       *  populates the list of sub-graphs to be processed and also
       *  sets the default state for the overall sub-graph list */
      if (AR_EOK !=
          (result = apm_gm_cmd_validate_sg_list(apm_info_ptr, num_sgs_to_process, sg_id_list_ptr, cmd_opcode)))
      {
         AR_MSG(DBG_ERROR_PRIO, "GRAPH_MGMT: SG_ID list validation failed, cmd_opcode[0x%08lx]", cmd_opcode);

         return result;
      }

      proxy_mgr_list_ptr = proxy_mgr_list_ptr->next_ptr;

   } /** End of while (proxy_mgr_list_ptr) */

   apm_curr_cmd_ctrl->rsp_ctrl.permitted_proxy_sgs_pending = FALSE;

   return result;
}

ar_result_t apm_gm_cmd_clear_proxy_mgr_sg_info(apm_t *apm_info_ptr)
{
   apm_cmd_ctrl_t *apm_cmd_ctrl_ptr;

   apm_cmd_ctrl_ptr = apm_info_ptr->curr_cmd_ctrl_ptr;

   /** If APM_CMD_GRAPH_CLOSE, clear the proxy manager list from the APM graph info. */
   /* For SPF_MSG_CMD_SET_CFG, close all can be called */
   if ((APM_CMD_GRAPH_CLOSE == apm_cmd_ctrl_ptr->cmd_opcode) || APM_CMD_CLOSE_ALL == apm_cmd_ctrl_ptr->cmd_opcode ||
       SPF_MSG_CMD_SET_CFG == apm_cmd_ctrl_ptr->cmd_opcode || apm_is_open_cmd_err_seq(&apm_cmd_ctrl_ptr->cmd_seq))
   {
      apm_proxy_util_remove_closed_subgraphs_from_proxy_mgrs(apm_info_ptr);
   }

   /**Clear permitted Proxy manager list. */
   apm_clear_permitted_proxy_list(apm_cmd_ctrl_ptr);

   /** Done with sending all proxy commands.
       Clear the active proxy manager list, if non-empty */
   apm_clear_active_proxy_list(apm_info_ptr, apm_cmd_ctrl_ptr);

   return AR_EOK;
}

static void apm_proxy_util_clear_proxy_helper(apm_t            *apm_info_ptr,
                                              apm_cmd_ctrl_t   *proxy_list_pptr,
                                              spf_list_node_t **curr_node_pptr,
                                              uint32_t         *proxy_count_ptr)
{
   spf_list_node_t      *next_node_ptr;
   apm_proxy_manager_t  *proxy_mgr_ptr;
   apm_proxy_cmd_ctrl_t *proxy_cmd_ctrl_ptr;
   spf_list_node_t      *curr_node_ptr = *curr_node_pptr;

   /** Iterate over the proxy list */
   while (curr_node_ptr)
   {
      proxy_mgr_ptr = (apm_proxy_manager_t *)curr_node_ptr->obj_ptr;

      next_node_ptr = curr_node_ptr->next_ptr;

      /** Get the proxy's command control object corresponding
       *  to current APM command control object */
      apm_proxy_util_get_allocated_cmd_ctrl_obj(proxy_mgr_ptr, proxy_list_pptr, &proxy_cmd_ctrl_ptr);

      if (NULL != proxy_cmd_ctrl_ptr)
      {
         if (VCPM_MODULE_INSTANCE_ID == proxy_mgr_ptr->proxy_instance_id)
         {
            if (NULL != proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr)
            {
               posal_memory_free(proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr);
               proxy_cmd_ctrl_ptr->rsp_ctrl.permitted_sg_array_ptr = NULL;
            }

            /** Clear the container command control */
            memset(&proxy_cmd_ctrl_ptr->rsp_ctrl, 0, sizeof(apm_proxy_cmd_rsp_ctrl_t));

            /** Release the container command control object */
            apm_proxy_util_release_cmd_ctrl_obj(proxy_mgr_ptr, proxy_cmd_ctrl_ptr);

            /** Free up the list node memory and
             *  advance to next node in the list */
            spf_list_find_delete_node(curr_node_pptr, proxy_mgr_ptr, TRUE /*pool_used */);

            /** Decrement number of proxy manager */
            (*proxy_count_ptr)--;

            if (NULL == proxy_mgr_ptr->proxy_sg_list_ptr)
            {
               /** Remove Proxy Manager node from apm graph info list. */
               apm_db_remove_node_from_list(&apm_info_ptr->graph_info.proxy_manager_list_ptr,
                                            proxy_mgr_ptr,
                                            &apm_info_ptr->graph_info.num_proxy_managers);
               posal_memory_free(proxy_mgr_ptr);
            }

            AR_MSG(DBG_HIGH_PRIO, "Proxy removed during error handling");
         }
      }

      /** Advance to next node */
      curr_node_ptr = next_node_ptr;

   } /** End of while() */
}

ar_result_t apm_proxy_util_clear_vcpm_active_or_inactive_proxy(apm_t *apm_info_ptr, apm_cmd_ctrl_t *apm_cmd_ctrl_ptr)
{
   ar_result_t result = AR_EOK;
   /** Validate input params. */
   if (NULL == apm_cmd_ctrl_ptr)
   {
      return AR_EBADPARAM;
   }

   apm_proxy_util_clear_proxy_helper(apm_info_ptr,
                                     apm_cmd_ctrl_ptr,
                                     &apm_cmd_ctrl_ptr->rsp_ctrl.active_proxy_mgr_list_ptr,
                                     &apm_cmd_ctrl_ptr->rsp_ctrl.num_active_proxy_mgrs);
   apm_proxy_util_clear_proxy_helper(apm_info_ptr,
                                     apm_cmd_ctrl_ptr,
                                     &apm_cmd_ctrl_ptr->rsp_ctrl.inactive_proxy_mgr_list_ptr,
                                     &apm_cmd_ctrl_ptr->rsp_ctrl.num_inactive_proxy_mgrs);

   return result;
}
