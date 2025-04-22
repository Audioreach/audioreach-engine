/**
 * \file cu_path_delay.c
 * \brief
 *     This file contains container utility functions for path delay handling
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_i.h"
#include "apm.h"
#include "capi_intf_extn_path_delay.h"

/**
 * SPF_EVT_TO_APM_FOR_PATH_DELAY
 */
static ar_result_t cu_raise_event_to_apm_for_path_delay(cu_base_t *base_ptr,
                                                        uint32_t   src_module_instance_id,
                                                        uint32_t   src_port_id,
                                                        uint32_t   dst_module_instance_id,
                                                        uint32_t   dst_port_id)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "EVT:Raising path-delay event: PATH_DELAY: current channel mask=0x%x. src 0x%08lX, %lu, dst 0x%08lX, %lu",
          base_ptr->curr_chan_mask,
          src_module_instance_id,
          src_port_id,
          dst_module_instance_id,
          dst_port_id);

   spf_msg_t     msg;
   uint32_t      total_size = GET_SPF_INLINE_DATABUF_REQ_SIZE(sizeof(spf_evt_to_apm_for_path_delay_t));
   spf_handle_t *apm_handle = apm_get_apm_handle();

   if (AR_DID_FAIL(result = spf_msg_create_msg(&msg,
                                               &total_size,
                                               SPF_EVT_TO_APM_FOR_PATH_DELAY,
                                               NULL,
                                               NULL,
                                               apm_handle,
                                               base_ptr->heap_id)))
   {
      CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "PATH_DELAY: Fail to path delay event buffer");
      THROW(result, result);
   }

   {
      spf_msg_header_t *               header_ptr = (spf_msg_header_t *)(msg.payload_ptr);
      spf_evt_to_apm_for_path_delay_t *evt_ptr    = (spf_evt_to_apm_for_path_delay_t *)&header_ptr->payload_start;
      evt_ptr->path_def.src_module_instance_id    = src_module_instance_id;
      evt_ptr->path_def.src_port_id               = src_port_id;
      evt_ptr->path_def.dst_module_instance_id    = dst_module_instance_id;
      evt_ptr->path_def.dst_port_id               = dst_port_id;

      if (AR_DID_FAIL(result = spf_msg_send_cmd(&msg, apm_handle)))
      {
         spf_msg_return_msg(&msg);
         THROW(result, result);
      }
   }

   CATCH(result, CU_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }

   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "EVT:Raised event: PATH_DELAY: current channel mask=0x%x. result=0x%lx. src 0x%08lX, %lu, dst 0x%08lX, %lu",
          base_ptr->curr_chan_mask,
          result,
          src_module_instance_id,
          src_port_id,
          dst_module_instance_id,
          dst_port_id);

   return result;
}

ar_result_t cu_handle_event_to_dsp_service_topo_cb_for_path_delay(cu_base_t *        cu_ptr,
                                                                  gu_module_t *      module_ptr,
                                                                  capi_event_info_t *event_info_ptr)
{
   ar_result_t                       result        = AR_EOK;
   capi_buf_t *                      payload       = &event_info_ptr->payload;
   capi_event_data_to_dsp_service_t *dsp_event_ptr = (capi_event_data_to_dsp_service_t *)(payload->data_ptr);

   switch (dsp_event_ptr->param_id)
   {
      case INTF_EXTN_EVENT_ID_REQUEST_PATH_DELAY:
      {
         if (dsp_event_ptr->payload.actual_data_len < sizeof(intf_extn_event_id_request_path_delay_t))
         {
            CU_MSG(cu_ptr->gu_ptr->log_id,
                   DBG_ERROR_PRIO,
                   "Module 0x%lX: Error in callback function. The actual size %lu is less than the required size "
                   "%lu for id %lu.",
                   module_ptr->module_instance_id,
                   payload->actual_data_len,
                   sizeof(intf_extn_event_id_request_path_delay_t),
                   dsp_event_ptr->param_id);
            return AR_ENEEDMORE;
         }

         intf_extn_event_id_request_path_delay_t *data_ptr =
            (intf_extn_event_id_request_path_delay_t *)(dsp_event_ptr->payload.data_ptr);

         result = cu_raise_event_to_apm_for_path_delay(cu_ptr,
                                                       data_ptr->src_module_instance_id,
                                                       data_ptr->src_port_id,
                                                       data_ptr->dst_module_instance_id,
                                                       data_ptr->dst_port_id);
         break;
      }
      default:
      {
         return AR_EUNSUPPORTED;
      }
   }

   return result;
}

/**
 * call sequence:
 * A. one time query:      cu_path_delay_cfg -> cu_aggregate_ext_in_port_delay -> get_additional_ext_in_port_delay_cu_cb
 *                            -> gen_cntr_aggregate_ext_in_port_delay_util
 * B. Updating delay var:  cu_update_path_delay -> cu_operate_on_delay_paths -> topo.update_path_delays
 *                            -> cntr.aggregate_ext_in_port_delay -> cu_aggregate_ext_in_port_delay
 *                            -> get_additional_ext_in_port_delay_cu_cb -> gen_cntr_aggregate_ext_in_port_delay_util
 */
uint32_t cu_aggregate_ext_in_port_delay(cu_base_t *base_ptr, gu_ext_in_port_t *gu_ext_in_port_ptr)
{
   /**
    * the external input contributes to delay if there's buffering at ext-in.
    */
   cu_ext_in_port_t *ext_in_port_ptr =
      (cu_ext_in_port_t *)((uint8_t *)gu_ext_in_port_ptr + base_ptr->ext_in_port_cu_offset);

   if (!SPF_IS_PACKETIZED_OR_PCM(ext_in_port_ptr->media_fmt.data_format))
   {
      return 0;
   }

   if (base_ptr->cntr_vtbl_ptr->get_additional_ext_in_port_delay_cu_cb)
   {
      return base_ptr->cntr_vtbl_ptr->get_additional_ext_in_port_delay_cu_cb(base_ptr, gu_ext_in_port_ptr);
   }
   return 0;
}

uint32_t cu_aggregate_ext_out_port_delay(cu_base_t *base_ptr, gu_ext_out_port_t *gu_ext_out_port_ptr)
{
   uint32_t reg_bufs_contribution = 0;

   cu_ext_out_port_t *ext_out_port_ptr =
      (cu_ext_out_port_t *)((uint8_t *)gu_ext_out_port_ptr + base_ptr->ext_out_port_cu_offset);

   if (!SPF_IS_PACKETIZED_OR_PCM(ext_out_port_ptr->media_fmt.data_format))
   {
      return 0;
   }

   /**
    * Path delay will not be accurate in STOP state.
    * Because upstream RT will not be correct.
    */
   uint32_t is_upstream_rt = FALSE;
   base_ptr->topo_vtbl_ptr->get_port_property(base_ptr->topo_ptr,
                                              TOPO_DATA_OUTPUT_PORT_TYPE,
                                              PORT_PROPERTY_IS_UPSTREAM_RT,
                                              gu_ext_out_port_ptr->int_out_port_ptr,
                                              &is_upstream_rt);
   if (is_upstream_rt > 0)
   {
      reg_bufs_contribution = 0;
   }
   else
   {
      reg_bufs_contribution = ext_out_port_ptr->icb_info.icb.num_reg_bufs;
   }

   /* RT propagation does not necessarily happen through multi port modules, hence setting reg bufs contribution
    * to zero for voice call use case. */
   if (cu_has_voice_sid(base_ptr))
   {
      reg_bufs_contribution = 0;
   }

   uint32_t delay =
      (reg_bufs_contribution + ext_out_port_ptr->icb_info.icb.num_reg_prebufs) * base_ptr->cntr_frame_len.frame_len_us +
      ext_out_port_ptr->icb_info.icb.otp.frame_len_us;

   if (base_ptr->cntr_vtbl_ptr->get_additional_ext_out_port_delay_cu_cb)
   {
      delay += base_ptr->cntr_vtbl_ptr->get_additional_ext_out_port_delay_cu_cb(base_ptr, gu_ext_out_port_ptr);
   }
   return delay;
}

#define PRINT_PATH_DELAY(base_ptr, path_id, algo_delay, ext_in_delay, ext_out_delay)                                   \
   CU_MSG(base_ptr->gu_ptr->log_id,                                                                                    \
          DBG_HIGH_PRIO,                                                                                               \
          "PATH_DELAY: path-id 0x%lX, total algo delay %lu us, ext in delay %lu us, ext out delay %lu us. Total path " \
          "delay %lu us",                                                                                              \
          path_id,                                                                                                     \
          algo_delay,                                                                                                  \
          ext_in_delay,                                                                                                \
          ext_out_delay,                                                                                               \
          algo_delay + ext_out_delay + ext_in_delay)

/**
 * APM get param for CNTR_PARAM_ID_PATH_DELAY_CFG
 */
ar_result_t cu_path_delay_cfg(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING
   uint32_t                        algo_delay_us        = 0;
   uint32_t                        ext_out_buf_delay_us = 0, ext_in_delay_us = 0;
   uint32_t                        num_ext_in_port      = 0;
   uint32_t                        num_ext_out_port     = 0;
   gu_module_t *                   prev_module_ptr      = NULL;
   gu_cmn_port_t *                 prev_port_ptr        = NULL;
   cntr_param_id_path_delay_cfg_t *cmd_ptr              = NULL;
   VERIFY(result,
          (NULL != base_ptr->topo_vtbl_ptr) && (NULL != base_ptr->topo_vtbl_ptr->query_module_delay) &&
             (NULL != base_ptr->topo_vtbl_ptr->add_path_delay_info));

   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "CMD:PATH_DELAY: Executing path delay cfg set-param. current channel mask=0x%x",
          base_ptr->curr_chan_mask);

   VERIFY(result, *param_size_ptr >= sizeof(cntr_param_id_path_delay_cfg_t));
   cmd_ptr = (cntr_param_id_path_delay_cfg_t *)param_payload_ptr;

   // reserve 0 for internal use (0 == all paths)
   VERIFY(result, (NULL != cmd_ptr) && (CU_PATH_ID_ALL_PATHS != cmd_ptr->path_id));

   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_LOW_PRIO,
          "CMD:PATH_DELAY: path_id 0x%lx, one time query? %u",
          cmd_ptr->path_id,
          cmd_ptr->is_one_time_query);

   VERIFY(result,
          (NULL != cmd_ptr->path_def_ptr) && (NULL != cmd_ptr->path_def_ptr->obj_ptr) &&
             (NULL != cmd_ptr->delay_us_ptr));

   for (cntr_list_node_t *node_ptr = cmd_ptr->path_def_ptr; (NULL != node_ptr); LIST_ADVANCE(node_ptr))
   {
      gu_module_t *  module_ptr = NULL;
      gu_cmn_port_t *port_ptr   = NULL;

      cntr_graph_vertex_t *vertex_ptr = (cntr_graph_vertex_t *)node_ptr->obj_ptr;

      module_ptr = gu_find_module(base_ptr->gu_ptr, vertex_ptr->module_instance_id);
      if (NULL != module_ptr)
      {
         port_ptr = gu_find_cmn_port_by_id(module_ptr, vertex_ptr->port_id);

         if (port_ptr)
         {
            if (!cmd_ptr->is_one_time_query)
            {
               base_ptr->topo_vtbl_ptr->add_path_delay_info(base_ptr->topo_ptr,
                                                            module_ptr,
                                                            vertex_ptr->port_id,
                                                            port_ptr,
                                                            cmd_ptr->path_id);
            }

            gu_ext_in_port_t *gu_ext_in_port_ptr = gu_get_ext_in_port_from_cmn_port(port_ptr);
            if (NULL != gu_ext_in_port_ptr)
            {
               num_ext_in_port++;
               ext_in_delay_us += cu_aggregate_ext_in_port_delay(base_ptr, gu_ext_in_port_ptr);
            }

            gu_ext_out_port_t *gu_ext_out_port_ptr = NULL;
            gu_get_ext_out_port_for_last_module(base_ptr->gu_ptr, port_ptr, &gu_ext_out_port_ptr);
            if (NULL != gu_ext_out_port_ptr)
            {
               num_ext_out_port++;
               ext_out_buf_delay_us += cu_aggregate_ext_out_port_delay(base_ptr, gu_ext_out_port_ptr);
            }

            /*
             * in the vertex list, if prev and current module's are same, then prev_port_ptr is the input and
             * port_ptr is the output. Using this we can query delay of MIMO module.
             *
             * in case modules are distributed in the list, this method won't work. we need another loop.
             *
             * For now, it's assumed that APM sorts the list properly to have same module ports in subsequent array
             * indices.
             *
             * see comment in APM_PARAM_ID_PATH_DELAY: source, sink module delays are always included.
             * for other modules it's src output to dst output
             */
            uint32_t temp_delay = 0;
            if (module_ptr->flags.is_source || module_ptr->flags.is_sink ||
                (prev_module_ptr && (prev_module_ptr->module_instance_id == vertex_ptr->module_instance_id)))
            {
               base_ptr->topo_vtbl_ptr->query_module_delay(base_ptr->topo_ptr,
                                                           module_ptr,
                                                           prev_port_ptr,
                                                           port_ptr,
                                                           &temp_delay);
            }
            algo_delay_us += temp_delay;
         }
      }

      prev_module_ptr = module_ptr;
      prev_port_ptr   = port_ptr;
   }

   /**
    * Generally path from A->Z will hit container only once - so num of ext-in and num of ext-out is just one.
    * But C1 [ A->B ] -> C2 [C->D] -> C1 [ E -> F] is a graph where C1 is appearing twice. Two ext-input possible.
    */
   if ((num_ext_in_port > 1) || (num_ext_out_port > 1))
   {
      CU_MSG(base_ptr->gu_ptr->log_id,
             DBG_LOW_PRIO,
             "CMD:PATH_DELAY: Found %lu ext inputs and %lu ext outputs. Container has disjoint graphs",
             num_ext_in_port,
             num_ext_out_port);
   }

   if (!cmd_ptr->is_one_time_query)
   {
      cu_delay_info_t *delay_info_ptr = NULL;
      MALLOC_MEMSET(delay_info_ptr, cu_delay_info_t, sizeof(cu_delay_info_t), base_ptr->heap_id, result);
      TRY(result,
          spf_list_insert_tail(&base_ptr->delay_path_list_ptr, delay_info_ptr, base_ptr->heap_id, TRUE /*use pool*/));

      delay_info_ptr->path_id      = cmd_ptr->path_id;
      delay_info_ptr->delay_us_ptr = cmd_ptr->delay_us_ptr; // delay pointer from APM
   }

   // clang-format off
   CATCH(result, CU_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {

   }

   if (cmd_ptr)
   {
      if (cmd_ptr->delay_us_ptr)
      {
         *cmd_ptr->delay_us_ptr = (algo_delay_us + ext_out_buf_delay_us + ext_in_delay_us);
      }

      PRINT_PATH_DELAY(base_ptr, cmd_ptr->path_id, algo_delay_us, ext_in_delay_us, ext_out_buf_delay_us);
   }

   // clang-format on
   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "CMD:PATH_DELAY: Done Executing path delay cfg cmd, current channel mask=0x%x. result=0x%lx",
          base_ptr->curr_chan_mask,
          result);

   return result;
}

/**
 * APM SET param for CNTR_PARAM_ID_CFG_SRC_MOD_DELAY_LIST
 *
 * here source module means the source of event SPF_EVT_TO_APM_FOR_PATH_DELAY
 * not necessarily the one with zero inputs.
 */
ar_result_t cu_cfg_src_mod_delay_list(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "CMD:CFG_SRC_MD_DELAY_LIST: PATH_DELAY: Executing setting source module delay list. current channel "
          "mask=0x%x",
          base_ptr->curr_chan_mask);

   VERIFY(result, *param_size_ptr >= sizeof(cntr_param_id_cfg_src_mod_delay_list_t));

   {
      cntr_param_id_cfg_src_mod_delay_list_t *cmd_ptr = (cntr_param_id_cfg_src_mod_delay_list_t *)param_payload_ptr;

      intf_extn_path_delay_response_t payload = {
         .path_id                = cmd_ptr->path_id,
         .src_module_instance_id = cmd_ptr->src_module_instance_id,
         .src_port_id            = cmd_ptr->src_port_id,
         .dst_module_instance_id = cmd_ptr->dst_module_instance_id,
         .dst_port_id            = cmd_ptr->dst_port_id,
         .num_delay_ptrs         = cmd_ptr->num_delay_ptrs,
         .delay_us_pptr          = cmd_ptr->delay_us_pptr,
      };

      uint32_t param_size = sizeof(payload);
      uint32_t error_code = 0;
      cu_set_get_cfg_wrapper(base_ptr,
                             cmd_ptr->src_module_instance_id,
                             INTF_EXTN_PARAM_ID_RESPONSE_PATH_DELAY,
                             (int8_t *)&payload,
                             &param_size,
                             &error_code,
                             TRUE /* is_set */,
                             FALSE /* is_deregister */,
                             SPF_CFG_DATA_TYPE_DEFAULT,
                             NULL);

      if (AR_DID_FAIL(result) || (AR_DID_FAIL(error_code)))
      {
         CU_MSG(base_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "CMD:CFG_SRC_MD_DELAY_LIST: PATH_DELAY: failed set cfg to module 0x%08lX",
                cmd_ptr->src_module_instance_id);
      }
   }
   CATCH(result, CU_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }

   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "CMD:CFG_SRC_MD_DELAY_LIST: PATH_DELAY: Done Executing path delay cfg cmd, current channel mask=0x%x. "
          "result=0x%lx.",
          base_ptr->curr_chan_mask,
          result);

   return result;
}

/**
 * APM Set param for CNTR_PARAM_ID_DESTROY_SRC_MOD_DELAY_LIST
 *
 * here source module means the source of event SPF_EVT_TO_APM_FOR_PATH_DELAY
 * not necessarily the one with zero inputs.
 */
ar_result_t cu_destroy_src_mod_delay_list(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "CMD:DESTROY_SRC_MD_DELAY_LIST: PATH_DELAY: Executing setting source module delay list. current channel "
          "mask=0x%x",
          base_ptr->curr_chan_mask);

   VERIFY(result, *param_size_ptr >= sizeof(cntr_param_id_destroy_src_mod_delay_list_t));

   {
      cntr_param_id_destroy_src_mod_delay_list_t *cmd_ptr =
         (cntr_param_id_destroy_src_mod_delay_list_t *)param_payload_ptr;

      intf_extn_path_delay_destroy_t payload = {
         .path_id                = cmd_ptr->path_id,
         .src_module_instance_id = cmd_ptr->src_module_instance_id,
         .src_port_id            = cmd_ptr->src_port_id,
      };

      uint32_t param_size = sizeof(payload);
      uint32_t error_code = 0;
      cu_set_get_cfg_wrapper(base_ptr,
                             cmd_ptr->src_module_instance_id,
                             INTF_EXTN_PARAM_ID_DESTROY_PATH_DELAY,
                             (int8_t *)&payload,
                             &param_size,
                             &error_code,
                             TRUE /* is_set */,
                             FALSE /* is_deregister */,
                             SPF_CFG_DATA_TYPE_DEFAULT,
                             NULL);

      if (AR_DID_FAIL(result) || (AR_DID_FAIL(error_code)))
      {
         CU_MSG(base_ptr->gu_ptr->log_id,
                DBG_ERROR_PRIO,
                "CMD:DESTROY_SRC_MD_DELAY_LIST: PATH_DELAY: failed set cfg to module 0x%08lX",
                cmd_ptr->src_module_instance_id);
      }
   }
   CATCH(result, CU_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }

   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "CMD:DESTROY_SRC_MD_DELAY_LIST: PATH_DELAY: Done Executing path delay cfg cmd, current channel mask=0x%x. "
          "result=0x%lx.",
          base_ptr->curr_chan_mask,
          result);

   return result;
}

/**
 * path-id = 0 => all paths
 *
 * used for destroy or update
 */
ar_result_t cu_operate_on_delay_paths(cu_base_t *base_ptr, uint32_t path_id, cu_path_delay_op_t op)
{
   ar_result_t result = AR_EOK;

   // if removal, then remove topo layer specific info first
   // then remove CU specific
   if (CU_PATH_DELAY_OP_REMOVE == op)
   {
      if (base_ptr->topo_vtbl_ptr && base_ptr->topo_vtbl_ptr->remove_path_delay_info)
      {
         base_ptr->topo_vtbl_ptr->remove_path_delay_info(base_ptr->topo_ptr, path_id);
      }

      if (CU_PATH_ID_ALL_PATHS == path_id)
      {
#ifdef PATH_DELAY_DEBUGGING
         CU_MSG(base_ptr->gu_ptr->log_id, DBG_LOW_PRIO, "PATH_DELAY: Destroying all path delay info");
#endif
         spf_list_delete_list_and_free_objs(&base_ptr->delay_path_list_ptr, TRUE /* pool-used*/);
         return result;
      }
   }

   spf_list_node_t *node_ptr = base_ptr->delay_path_list_ptr;
   spf_list_node_t *next_ptr = NULL;

   while (node_ptr)
   {
      cu_delay_info_t *delay_info_ptr = (cu_delay_info_t *)node_ptr->obj_ptr;
      next_ptr                        = node_ptr->next_ptr;

      switch (op)
      {
         case CU_PATH_DELAY_OP_REMOVE:
         {
            if (path_id == delay_info_ptr->path_id)
            {
#ifdef PATH_DELAY_DEBUGGING
               CU_MSG(base_ptr->gu_ptr->log_id,
                      DBG_LOW_PRIO,
                      "PATH_DELAY: path-id 0x%lX, Destroying delay info ",
                      path_id);
#endif
               spf_list_delete_node_and_free_obj(&node_ptr, &base_ptr->delay_path_list_ptr, TRUE /* pool-used*/);
            }
            break;
         }
         case CU_PATH_DELAY_OP_UPDATE:
         {
            if ((CU_PATH_ID_ALL_PATHS == path_id) || (path_id == delay_info_ptr->path_id))
            {
               uint32_t algo_delay    = 0;
               uint32_t prev_delay    = 0;
               uint32_t new_delay     = 0;
               uint32_t ext_out_delay = 0, ext_in_delay = 0;
               if (base_ptr->topo_vtbl_ptr && base_ptr->topo_vtbl_ptr->update_path_delays)
               {
                  base_ptr->topo_vtbl_ptr->update_path_delays(base_ptr->topo_ptr,
                                                              delay_info_ptr->path_id,
                                                              &algo_delay,
                                                              &ext_in_delay,
                                                              &ext_out_delay);
               }

               PRINT_PATH_DELAY(base_ptr, delay_info_ptr->path_id, algo_delay, ext_in_delay, ext_out_delay);

               prev_delay = *(delay_info_ptr->delay_us_ptr);
               new_delay  = (algo_delay + ext_out_delay + ext_in_delay);

               // implies that the container is processing in satellite domain
               cu_raise_event_get_path_delay(base_ptr, prev_delay, new_delay, delay_info_ptr->path_id);

               *(delay_info_ptr->delay_us_ptr) = new_delay;
            }
            break;
         }
         default:
         {
            CU_MSG(base_ptr->gu_ptr->log_id, DBG_ERROR_PRIO, "Invalid path delay operation %lu", op);
         }
      }

      node_ptr = next_ptr;
   }

   return result;
}

ar_result_t cu_update_path_delay(cu_base_t *base_ptr, uint32_t path_id)
{
   return cu_operate_on_delay_paths(base_ptr, path_id, CU_PATH_DELAY_OP_UPDATE);
}
/**
 * called by APM through set param: CNTR_PARAM_ID_PATH_DESTROY
 */
ar_result_t cu_destroy_delay_path_cfg(cu_base_t *base_ptr, int8_t *param_payload_ptr, uint32_t *param_size_ptr)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "CMD:DESTROY_PATH_DELAY: Executing path delay cfg cmd. current channel mask=0x%x",
          base_ptr->curr_chan_mask);

   VERIFY(result, *param_size_ptr >= sizeof(cntr_param_id_path_destroy_t));

   {
      cntr_param_id_path_destroy_t *cmd_ptr = (cntr_param_id_path_destroy_t *)param_payload_ptr;

      result = cu_operate_on_delay_paths(base_ptr, cmd_ptr->path_id, CU_PATH_DELAY_OP_REMOVE);
   }

   CATCH(result, CU_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }

   CU_MSG(base_ptr->gu_ptr->log_id,
          DBG_HIGH_PRIO,
          "CMD:DESTROY_PATH_DELAY: Done Executing path delay cfg cmd, current channel mask=0x%x. result=0x%lx.",
          base_ptr->curr_chan_mask,
          result);

   return result;
}
