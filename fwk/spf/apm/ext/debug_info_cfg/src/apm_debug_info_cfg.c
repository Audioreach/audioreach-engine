/**
 * \apm_debug_info_cfg.c
 *
 * \brief
 *     This file contains Apm Debug Info Config utilities
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/
#include "rtm_logging_api.h"
#include "apm_sub_graph_api.h"
#include "apm_debug_info_cfg.h"
#include "spf_list_utils.h"
#include "apm_internal.h"
#include "apm_cntr_debug_if.h"


static bool_t g_sub_graph_state_query_enable = 0;
//static bool_t g_port_media_format_query_enable = 0;
static uint32_t g_rtm_log_sg_state_seq_num = 0;
/**==============================================================================
   Function Declaration
==============================================================================*/

ar_result_t apm_parse_debug_info_params(apm_t *apm_info_ptr, apm_module_param_data_t *mod_data_ptr);
ar_result_t apm_sg_state_change(spf_list_node_t *processed_sg_list_ptr, uint32_t num_processed_sg);

/**==============================================================================
   Global Defines
==============================================================================*/

apm_debug_info_utils_vtable_t debug_info_cfg_util_funcs = {.apm_parse_debug_info_fptr = apm_parse_debug_info_params,
                                                           .apm_log_sg_state_change_fptr  = apm_sg_state_change };

ar_result_t apm_populate_port_mf_cntr_param_payload_size(uint32_t                    container_id,
                                                 apm_cont_set_get_cfg_hdr_t *set_get_cfg_hdr_ptr,
                                                 apm_module_param_data_t *   param_ptr)
{
   ar_result_t result = AR_EOK;

   apm_cont_aggregate_payload_cfg_t *cont_cached_cfg_ptr;
   cont_cached_cfg_ptr = (apm_cont_aggregate_payload_cfg_t *)set_get_cfg_hdr_ptr;

   apm_update_cont_param_set_cfg_msg_hdr(container_id,
                                         CNTR_PARAM_ID_DATA_PORT_MEDIA_FORMAT,
                                         sizeof(cntr_port_mf_param_data_cfg_t),
                                         param_ptr);

   /** populate the payload for container ptr */
   cntr_port_mf_param_data_cfg_t *port_mf_param_data_cfg_ptr = (cntr_port_mf_param_data_cfg_t *)(param_ptr + 1);
   port_mf_param_data_cfg_ptr->enable                        = cont_cached_cfg_ptr->cntr_payload.enable;

   return result;
}

ar_result_t apm_compute_port_mf_cntr_msg_payload_size(uint32_t *cntr_msg_payload_size_ptr)
{
   ar_result_t      result = AR_EOK;
   *cntr_msg_payload_size_ptr += (sizeof(apm_module_param_data_t) + sizeof(cntr_port_mf_param_data_cfg_t));
   return result;

}

static ar_result_t apm_util_cfg_port_media_fmt_reporting(apm_t *apm_info_ptr, apm_module_param_data_t *mod_data_ptr)
{
   ar_result_t result    = AR_EOK;
   bool_t      is_master = FALSE;

   if (apm_info_ptr->ext_utils.offload_vtbl_ptr &&
       apm_info_ptr->ext_utils.offload_vtbl_ptr->apm_offload_is_master_pid_fptr)
   {
      is_master = apm_info_ptr->ext_utils.offload_vtbl_ptr->apm_offload_is_master_pid_fptr();
   }

   if (is_master)
   {
      // Set pending message to sattelite apm, as we have recevied the set param.
      apm_info_ptr->curr_cmd_ctrl_ptr->set_cfg_cmd_ctrl.debug_info.is_sattelite_debug_info_send_pending = TRUE;
   }

   apm_param_id_port_media_fmt_report_cfg_enable_t *port_media_mft_report_enable_ptr =
      (apm_param_id_port_media_fmt_report_cfg_enable_t *)(mod_data_ptr + 1);

   // store enable pramater for sending it to satellite cntr
   apm_info_ptr->curr_cmd_ctrl_ptr->set_cfg_cmd_ctrl.debug_info.is_port_media_fmt_enable =
      port_media_mft_report_enable_ptr->is_port_media_fmt_report_cfg_enabled;

   apm_container_t *host_cont_node_ptr;
   for (uint32_t i = 0; i < APM_CONT_HASH_TBL_SIZE; i++)
   {
      spf_list_node_t *curr_ptr;
      curr_ptr = apm_info_ptr->graph_info.container_list_ptr[i];
      while (curr_ptr != NULL)
      {
         host_cont_node_ptr = (apm_container_t *)curr_ptr->obj_ptr;

         apm_cont_cmd_ctrl_t *             cont_cmd_ctrl_ptr = host_cont_node_ptr->cmd_list.cmd_ctrl_list;
         apm_cont_aggregate_payload_cfg_t *port_media_fmt_cfg_ptr;
         apm_cont_cached_cfg_t *           cont_cached_cfg_ptr;

         apm_get_cont_cmd_ctrl_obj(host_cont_node_ptr, apm_info_ptr->curr_cmd_ctrl_ptr, &cont_cmd_ctrl_ptr);

         if (NULL == (port_media_fmt_cfg_ptr = (apm_cont_aggregate_payload_cfg_t *)
                         posal_memory_malloc(sizeof(apm_cont_aggregate_payload_cfg_t), POSAL_HEAP_DEFAULT)))
         {
            AR_MSG(DBG_ERROR_PRIO,
                   "apm_aggregate_cntr_payload(): Failed to allocate memory for caching cont info, "
                   "CONT_ID[0x%lX]",
                   host_cont_node_ptr->container_id);

            return AR_ENOMEMORY;
         }

         /** Populate the allocated cached object */
         port_media_fmt_cfg_ptr->header.param_id = CNTR_PARAM_ID_DATA_PORT_MEDIA_FORMAT;
         port_media_fmt_cfg_ptr->cntr_payload.enable =
            port_media_mft_report_enable_ptr->is_port_media_fmt_report_cfg_enabled;

         cont_cached_cfg_ptr = &cont_cmd_ctrl_ptr->cached_cfg_params;
         if (AR_EOK != (result = apm_db_add_node_to_list(&cont_cached_cfg_ptr->cont_cfg_params.param_data_list_ptr,
                                                         port_media_fmt_cfg_ptr,
                                                         &cont_cached_cfg_ptr->cont_cfg_params.num_cfg_params)))
         {
            return result;
         }
         /** Add this container node to the list of container pending
             *  send message */

         if (AR_EOK != (result = apm_add_cont_to_pending_msg_send_list(apm_info_ptr->curr_cmd_ctrl_ptr,
                                                                       host_cont_node_ptr,
                                                                       cont_cmd_ctrl_ptr)))
         {
            return result;
         }
         curr_ptr = curr_ptr->next_ptr;
      }
   }

   return result;
}

static ar_result_t apm_rtm_dump_fill_sg_state_payload (uint32_t max_num_subgraphs, spf_list_node_t *sub_graph_list_node_ptr)
{
   ar_result_t                     result                     = AR_EOK;
   uint32_t size = (sizeof(apm_num_sub_graphs_state_t) + (sizeof(apm_sub_graph_id_state_t) * max_num_subgraphs));
   uint32_t max_payload_size = (sizeof(rtm_logging_param_data_t) + size + sizeof(rtm_logging_header_t));

   uint8_t *log_ptr = posal_data_log_alloc((max_payload_size), DEBUG_INFO_LOG_CODE, LOG_DATA_FMT_BITSTREAM);
   if (log_ptr != NULL)
   {
      memset((uint8_t *)log_ptr, 0, max_payload_size);

      rtm_logging_header_t *param_ptr      = (rtm_logging_header_t *)log_ptr;
      param_ptr->version                   = RTM_HEADER_VERSION_0;
      rtm_logging_param_data_t *param_data = (rtm_logging_param_data_t *)(param_ptr + 1);

      param_data->module_instance_id                    = APM_MODULE_INSTANCE_ID;
      param_data->param_id                              = APM_PARAM_ID_SG_STATE_REPORT_CFG;
      param_data->reserved                              = 0;
      param_data->param_size                            = size;
      apm_num_sub_graphs_state_t *payload_num_of_sg_ptr = (apm_num_sub_graphs_state_t *)(param_data + 1);
      payload_num_of_sg_ptr->num_of_sub_graphs                            = max_num_subgraphs;

      apm_sub_graph_id_state_t *payload_sg_id_and_state_ptr = (apm_sub_graph_id_state_t *)(payload_num_of_sg_ptr + 1);

      spf_list_node_t *sg_list_ptr = sub_graph_list_node_ptr;
      while (NULL != sg_list_ptr)
      {
         apm_sub_graph_t *sg_obj_ptr               = (apm_sub_graph_t *)sg_list_ptr->obj_ptr;
         payload_sg_id_and_state_ptr->sub_graph_id = sg_obj_ptr->sub_graph_id;
         payload_sg_id_and_state_ptr->state        = sg_obj_ptr->state;
         sg_list_ptr                               = sg_list_ptr->next_ptr;
         payload_sg_id_and_state_ptr               = payload_sg_id_and_state_ptr + 1;
      }

      posal_data_log_info_t posal_log_info;
      posal_log_info.buf_size   = max_payload_size;
      posal_log_info.data_fmt   = LOG_DATA_FMT_BITSTREAM;
      posal_log_info.log_code   = (uint32_t)DEBUG_INFO_LOG_CODE;
      posal_log_info.log_tap_id = DEBUG_INFO_TAP_ID;
      posal_log_info.session_id = 0;
         posal_log_info.seq_number_ptr = &g_rtm_log_sg_state_seq_num;
         g_rtm_log_sg_state_seq_num++;
      posal_data_log_commit(log_ptr, &posal_log_info);
   }
   return result;
}

static ar_result_t apm_util_rtm_log_sg_state(apm_t *apm_info_ptr, apm_module_param_data_t *mod_data_ptr)
{
   ar_result_t                     result                     = AR_EOK;
   apm_param_id_sub_graph_state_t *sg_state_report_enable_ptr = (apm_param_id_sub_graph_state_t *)(mod_data_ptr + 1);
   spf_list_node_t *sub_graph_list_node_ptr = (spf_list_node_t *)(apm_info_ptr->graph_info.sub_graph_list_ptr);
   uint32_t         max_num_subgraphs       = apm_info_ptr->graph_info.num_sub_graphs;

   if (g_sub_graph_state_query_enable == sg_state_report_enable_ptr->is_sg_state_report_cfg_enabled)
   {
      return result;
   }
   g_sub_graph_state_query_enable = sg_state_report_enable_ptr->is_sg_state_report_cfg_enabled;
   if (FALSE == sg_state_report_enable_ptr->is_sg_state_report_cfg_enabled)
   {
      return result;
   }
   apm_rtm_dump_fill_sg_state_payload(max_num_subgraphs, sub_graph_list_node_ptr);
   return result;
}

ar_result_t apm_parse_debug_info_params(apm_t *apm_info_ptr, apm_module_param_data_t *mod_data_ptr)
{
   ar_result_t result = AR_EOK;
   /** Validate input arguments */
   if (!apm_info_ptr || !mod_data_ptr)
   {
      AR_MSG(DBG_ERROR_PRIO,
             "APM_SET_CFG: APM info[0x%lX] or module data ptr[0x%lX] is/are NULL",
             apm_info_ptr,
             mod_data_ptr);

      return AR_EFAILED;
   }

   switch (mod_data_ptr->param_id)
   {
      case APM_PARAM_ID_PORT_MEDIA_FMT_REPORT_CFG:
      {
         result = apm_util_cfg_port_media_fmt_reporting(apm_info_ptr, mod_data_ptr);
         break;
      }
      case APM_PARAM_ID_SG_STATE_REPORT_CFG:
      {
         result = apm_util_rtm_log_sg_state(apm_info_ptr, mod_data_ptr);
         break;
      }
      default:
      {
         AR_MSG(DBG_ERROR_PRIO,
                "apm_parse_fwk_set_cfg_params :: WARNING :: Un-supported PID: 0x%lX",
                mod_data_ptr->param_id);

         result = AR_EUNSUPPORTED;
         break;
      }
   }
   return result;
}

ar_result_t apm_sg_state_change(spf_list_node_t *processed_sg_list_ptr, uint32_t num_processed_sg)
{
   ar_result_t result = AR_EOK;
   if ((TRUE == g_sub_graph_state_query_enable) && (0 != num_processed_sg))
   {
      AR_MSG(DBG_HIGH_PRIO,"num_processed_sg IS %lu",num_processed_sg);
      apm_rtm_dump_fill_sg_state_payload(num_processed_sg, processed_sg_list_ptr);
   }
   return result;
}

ar_result_t apm_debug_info_init(apm_t *apm_info_ptr)
{
   ar_result_t result = AR_EOK;

   apm_info_ptr->ext_utils.debug_info_utils_vtable_ptr = &debug_info_cfg_util_funcs;

   return result;
}
