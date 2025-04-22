/**
 * \file olc_buf_util.c
 * \brief
 *     This file contains olc utility functions for managing external port buffers (input and output).
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "olc_i.h"

/* =======================================================================
Static Function Declarations.
========================================================================== */

/* =======================================================================
Static Function Definitions
========================================================================== */
/**
 * Determines the number of elements needed in the external input port data
 * queue.
 */
uint32_t olc_get_in_queue_num_elements(olc_t *me_ptr, olc_ext_in_port_t *ext_port_ptr)
{
   return OLC_MAX_INP_DATA_Q_ELEMENTS;
}

/**
 * Determines the number of elements needed in the external output port buffer queue.
 */
uint32_t olc_get_out_queue_num_elements(olc_t *me_ptr, olc_ext_out_port_t *ext_port_ptr)
{
   return OLC_MAX_OUT_BUF_Q_ELEMENTS;
}

static ar_result_t olc_deinit_ext_port_queue(olc_t *me_ptr, spf_handle_t *hdl_ptr, uint32_t bit_mask)
{
   if (hdl_ptr->q_ptr)
   {

      cu_release_bit_in_bit_mask(&me_ptr->cu, bit_mask);

      /*Deinit the queue */
      posal_queue_deinit(hdl_ptr->q_ptr);
      hdl_ptr->q_ptr = NULL;
   }

   return AR_EOK;
}

/*
 * Create an external input port's data queue.
 * --  Determines the number of elements, bit mask, and name,
 * --  and calls a common function to allocate the queue.
 * Create the Write Queue for the same external input port.
 * -- Every external input port is associated with a Write Queue.
 * --  Determines the number of elements, bit mask, and name for the write Queue
 * --  and calls a common function to allocate the queue.
 */
ar_result_t olc_create_ext_in_queue(void *base_ptr, gu_ext_in_port_t *gu_ext_port_ptr)
{
   ar_result_t        result        = AR_EOK;
   olc_t *            me_ptr        = (olc_t *)base_ptr;
   olc_ext_in_port_t *ext_port_ptr  = (olc_ext_in_port_t *)gu_ext_port_ptr;
   uint32_t           wr_client_mid = ext_port_ptr->gu.int_in_port_ptr->cmn.module_ptr->module_instance_id;

   char data_q_name[POSAL_DEFAULT_NAME_LEN]; // data queue name
   snprintf(data_q_name, POSAL_DEFAULT_NAME_LEN, "%s%s%8lX", "D", "OLC", me_ptr->cu.gu_ptr->log_id);

   uint32_t num_elements = olc_get_in_queue_num_elements(me_ptr, ext_port_ptr);

   OLC_MSG(me_ptr->topo.gu.log_id,
           DBG_HIGH_PRIO,
           "Creating external input queue, number of dataQ "
           "elements is determined to be %lu",
           num_elements);

   uint32_t bit_mask = cu_request_bit_in_bit_mask(&me_ptr->cu.available_bit_mask);
   if (0 == bit_mask)
   {
      OLC_MSG(me_ptr->cu.gu_ptr->log_id,
              DBG_ERROR_PRIO,
              "Creating external input queue "
              "Bit mask has no bits available 0x%lx",
              me_ptr->cu.available_bit_mask);
      return AR_ENORESOURCE;
   }
   else
   {
      OLC_MSG(me_ptr->topo.gu.log_id,
              DBG_HIGH_PRIO,
              "creating external input queue"
              " with bit mask input 0x%08lx ",
              bit_mask);
   }

   ext_port_ptr->cu.bit_mask = bit_mask;

   if (AR_EOK != (result = cu_init_queue(&me_ptr->cu,
                                           data_q_name,
                                           num_elements,
                                           ext_port_ptr->cu.bit_mask,
                                           olc_input_dataQ_trigger,
                                           me_ptr->cu.channel_ptr,
                                           &gu_ext_port_ptr->this_handle.q_ptr,
                                           OLC_GET_EXT_IN_Q_ADDR(gu_ext_port_ptr),
                                           me_ptr->cu.heap_id)))
   {
      OLC_MSG(me_ptr->cu.gu_ptr->log_id, DBG_ERROR_PRIO, "Failed to create external input queue ");
      return result;
   }

   if (AR_EOK != (result = sgm_create_wr_data_queue(&me_ptr->cu,
                                                    &me_ptr->spgm_info,
                                                    bit_mask,
                                                    wr_client_mid,
                                                    OLC_GET_EXT_IN_SGM_Q_ADDR(gu_ext_port_ptr),
                                                    &ext_port_ptr->wdp_ctrl_cfg_ptr)))
   {
      OLC_MSG(me_ptr->cu.gu_ptr->log_id,
              DBG_ERROR_PRIO,
              "Failed to create write queue"
              " for this external input port ");
      return result;
   }

   if (AR_EOK != (result = sdm_setup_wr_data_port(&me_ptr->spgm_info, ext_port_ptr->wdp_ctrl_cfg_ptr->sdm_port_index)))
   {
      OLC_MSG(me_ptr->cu.gu_ptr->log_id,
              DBG_ERROR_PRIO,
              "Failed to set up write input port event port index %lu",
              ext_port_ptr->wdp_ctrl_cfg_ptr->sdm_port_index);
      return result;
   }

   return result;
}

/*
 * Initialize an external input port.
 */
ar_result_t olc_init_ext_in_port(void *base_ptr, gu_ext_in_port_t *gu_ext_port_ptr)
{
   ar_result_t        result       = AR_EOK;
   olc_t *            me_ptr       = (olc_t *)base_ptr;
   olc_ext_in_port_t *ext_port_ptr = (olc_ext_in_port_t *)gu_ext_port_ptr;
   ext_port_ptr->cu.id             = cu_get_next_unique_id(&(me_ptr->cu));

   // ext_in_port_ptr->configured_media_fmt;
   ext_port_ptr->gu.this_handle.cmd_handle_ptr = &me_ptr->cu.cmd_handle;

   result = olc_create_ext_in_queue(base_ptr, gu_ext_port_ptr);

   return result;
}

/*
 * Create an external output port's data queue. Determines the number of elements,
 * bit mask, and name, and calls a common function to allocate the queue.
 */
ar_result_t olc_create_ext_out_queue(void *base_ptr, gu_ext_out_port_t *gu_ext_port_ptr)
{
   ar_result_t         result        = AR_EOK;
   olc_t *             me_ptr        = (olc_t *)base_ptr;
   olc_ext_out_port_t *ext_port_ptr  = (olc_ext_out_port_t *)gu_ext_port_ptr;
   uint32_t            rd_client_mid = ext_port_ptr->gu.int_out_port_ptr->cmn.module_ptr->module_instance_id;

   char data_q_name[POSAL_DEFAULT_NAME_LEN]; // data queue name
   snprintf(data_q_name, POSAL_DEFAULT_NAME_LEN, "%s%s%8lX", "B", "OLC", me_ptr->cu.gu_ptr->log_id);

   uint32_t num_elements = olc_get_out_queue_num_elements(me_ptr, ext_port_ptr);

   OLC_MSG(me_ptr->topo.gu.log_id,
           DBG_HIGH_PRIO,
           "Creating external output queue, "
           "number of bufQ elements is determined to be %lu",
           num_elements);

   uint32_t bit_mask = cu_request_bit_in_bit_mask(&me_ptr->cu.available_bit_mask);
   if (0 == bit_mask)
   {
      OLC_MSG(me_ptr->cu.gu_ptr->log_id,
              DBG_ERROR_PRIO,
              "Creating external output queue,"
              "bit mask has no bits available 0x%lx",
              me_ptr->cu.available_bit_mask);
      return result;
   }
   else
   {
      OLC_MSG(me_ptr->topo.gu.log_id,
              DBG_HIGH_PRIO,
              "Creating external output queue,"
              "bit mask output 0x%08lx ",
              bit_mask);
   }

   ext_port_ptr->cu.bit_mask = bit_mask;

   if (AR_EOK != (result = cu_init_queue(&me_ptr->cu,
                                           data_q_name,
                                           num_elements,
                                           ext_port_ptr->cu.bit_mask,
                                           olc_output_bufQ_trigger,
                                           me_ptr->cu.channel_ptr,
                                           &gu_ext_port_ptr->this_handle.q_ptr,
                                           OLC_GET_EXT_CTRL_PORT_Q_ADDR(gu_ext_port_ptr),
                                           me_ptr->cu.heap_id)))
   {
      OLC_MSG(me_ptr->cu.gu_ptr->log_id, DBG_ERROR_PRIO, "Failed to init external output queue ");
      return result;
   }

   if (AR_EOK != (result = sgm_create_rd_data_queue(&me_ptr->cu,
                                                    &me_ptr->spgm_info,
                                                    bit_mask,
                                                    rd_client_mid,
                                                    OLC_GET_EXT_OUT_SGM_Q_ADDR(gu_ext_port_ptr),
                                                    &ext_port_ptr->rdp_ctrl_cfg_ptr)))
   {
      OLC_MSG(me_ptr->cu.gu_ptr->log_id,
              DBG_ERROR_PRIO,
              "Failed to create read queue"
              " for this external output port ");
      return result;
   }

   if (AR_EOK != (result = sdm_setup_rd_data_port(&me_ptr->spgm_info, ext_port_ptr->rdp_ctrl_cfg_ptr->sdm_port_index)))
   {
      OLC_MSG(me_ptr->cu.gu_ptr->log_id,
              DBG_ERROR_PRIO,
              "Failed to set up read output port"
              " event port index %lu",
              ext_port_ptr->rdp_ctrl_cfg_ptr->sdm_port_index);

      return result;
   }

   return result;
}

/*
 * Initialize an external output port.
 */
ar_result_t olc_init_ext_out_port(void *base_ptr, gu_ext_out_port_t *gu_ext_port_ptr)
{
   ar_result_t         result       = AR_EOK;
   olc_t *             me_ptr       = (olc_t *)base_ptr;
   olc_ext_out_port_t *ext_port_ptr = (olc_ext_out_port_t *)gu_ext_port_ptr;

   // Handle to the container command queue.
   ext_port_ptr->gu.this_handle.cmd_handle_ptr = &me_ptr->cu.cmd_handle;

   result = olc_create_ext_out_queue(base_ptr, gu_ext_port_ptr);

   return result;
}

/*
 * Deinitialize an external input port.
 */
ar_result_t olc_deinit_ext_in_port(void *base_ptr, gu_ext_in_port_t *gu_ext_port_ptr)
{
   ar_result_t        result          = AR_EOK;
   olc_t *            me_ptr          = (olc_t *)base_ptr;
   olc_ext_in_port_t *ext_in_port_ptr = (olc_ext_in_port_t *)gu_ext_port_ptr;

   if (SPF_RAW_COMPRESSED == ext_in_port_ptr->cu.media_fmt.data_format)
   {
      tu_capi_destroy_raw_compr_med_fmt(&ext_in_port_ptr->cu.media_fmt.raw);
   }

   if (ext_in_port_ptr->gu.this_handle.q_ptr)
   {
      olc_flush_input_data_queue(me_ptr, ext_in_port_ptr, FALSE /* keep data msg */, FALSE, FALSE);
   }

   if (ext_in_port_ptr->wdp_ctrl_cfg_ptr)
   {
      result |= sgm_destroy_wr_data_port(&me_ptr->spgm_info, ext_in_port_ptr->wdp_ctrl_cfg_ptr->sdm_port_index);
   }
   result |= olc_deinit_ext_port_queue(me_ptr, &ext_in_port_ptr->gu.this_handle, ext_in_port_ptr->cu.bit_mask);

   // Destroy the internal port.
   result |= gen_topo_destroy_input_port(&me_ptr->topo, (gen_topo_input_port_t *)ext_in_port_ptr->gu.int_in_port_ptr);
   return result;
}

/**
 * destroys (ext_port_ptr->cu.num_buf_allocated - num_bufs_to_keep) num of buffers,
 * where num_bufs_to_keep can be different from num_buf_allocated
 */
void olc_destroy_ext_buffers(olc_t *me_ptr, olc_ext_out_port_t *ext_port_ptr, uint32_t num_bufs_to_keep)
{
   uint32_t num_bufs_to_destroy = 0;
   if (num_bufs_to_keep < ext_port_ptr->cu.num_buf_allocated)
   {
      num_bufs_to_destroy = (ext_port_ptr->cu.num_buf_allocated - num_bufs_to_keep);
   }

   olc_flush_cnt_output_data_queue(me_ptr, ext_port_ptr);

   if (NULL == ext_port_ptr->gu.this_handle.q_ptr)
   {
      return;
   }

   // if we had allocated buffers, need to destroy now
   if (num_bufs_to_destroy)
   {
      spf_svc_free_buffers_in_buf_queue_nonblocking(ext_port_ptr->gu.this_handle.q_ptr,
                                                    &ext_port_ptr->cu.num_buf_allocated);
   }

   uint32_t num_bufs_destroyed = 0;
   uint32_t num_bufs_kept      = ext_port_ptr->cu.num_buf_allocated;
   if (num_bufs_to_destroy > num_bufs_kept)
   {
      num_bufs_destroyed = num_bufs_to_destroy - num_bufs_kept;
   }

   OLC_MSG(me_ptr->topo.gu.log_id,
           DBG_LOW_PRIO,
           " Destroyed %lu external buffers. num_bufs_kept %lu",
           num_bufs_destroyed,
           num_bufs_kept);

   ext_port_ptr->buf.data_ptr        = NULL;
   ext_port_ptr->buf.actual_data_len = 0;
   ext_port_ptr->buf.max_data_len    = 0;
}

/*
 * De-init an external output port.
 */
ar_result_t olc_deinit_ext_out_port(void *base_ptr, gu_ext_out_port_t *gu_ext_port_ptr)
{
   ar_result_t         result           = AR_EOK;
   olc_t *             me_ptr           = (olc_t *)base_ptr;
   olc_ext_out_port_t *ext_out_port_ptr = (olc_ext_out_port_t *)gu_ext_port_ptr;

   if (ext_out_port_ptr->cu.num_buf_allocated)
   {
      olc_flush_output_data_queue(me_ptr, ext_out_port_ptr, FALSE); // not a client cmd.
      // Destroy all the allocated buffers.
      while (0 < ext_out_port_ptr->cu.num_buf_allocated)
      {
         (void)posal_channel_wait(posal_queue_get_channel(ext_out_port_ptr->gu.this_handle.q_ptr),
                                  posal_queue_get_channel_bit(ext_out_port_ptr->gu.this_handle.q_ptr));

         olc_destroy_ext_buffers(me_ptr, ext_out_port_ptr, 0);
      }
   }
   else
   {
      olc_destroy_ext_buffers(me_ptr, ext_out_port_ptr, 0);
   }

   if (ext_out_port_ptr->rdp_ctrl_cfg_ptr)
   {
      result |= sgm_destroy_rd_data_port(&me_ptr->spgm_info, ext_out_port_ptr->rdp_ctrl_cfg_ptr->sdm_port_index);
   }
   result |= olc_deinit_ext_port_queue(me_ptr, &ext_out_port_ptr->gu.this_handle, ext_out_port_ptr->cu.bit_mask);

   // Destroy the internal port
   result |=
      gen_topo_destroy_output_port(&me_ptr->topo, (gen_topo_output_port_t *)ext_out_port_ptr->gu.int_out_port_ptr);

   if (SPF_RAW_COMPRESSED == ext_out_port_ptr->cu.media_fmt.data_format)
   {
      tu_capi_destroy_raw_compr_med_fmt(&ext_out_port_ptr->cu.media_fmt.raw);
   }

   return result;
}

ar_result_t olc_ext_out_port_apply_pending_media_fmt(olc_t *             me_ptr,
                                                     olc_ext_out_port_t *ext_out_port_ptr,
                                                     bool_t              is_data_path)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t    result           = AR_EOK;
   uint32_t       media_fmt_opcode = SPF_MSG_DATA_MEDIA_FORMAT;
   posal_queue_t *q_ptr            = NULL;

   q_ptr = is_data_path ? ext_out_port_ptr->gu.downstream_handle.spf_handle_ptr->q_ptr
                        : ext_out_port_ptr->gu.downstream_handle.spf_handle_ptr->cmd_handle_ptr->cmd_q_ptr;

   media_fmt_opcode = is_data_path ? SPF_MSG_DATA_MEDIA_FORMAT : SPF_MSG_CMD_MEDIA_FORMAT;

   if ((ext_out_port_ptr->gu.downstream_handle.spf_handle_ptr) &&
       (ext_out_port_ptr->cu.media_fmt.data_format != SPF_UNKNOWN_DATA_FORMAT))
   {
      spf_msg_t media_fmt_msg = { 0 };

      TRY(result,
          cu_create_media_fmt_msg_for_downstream(&me_ptr->cu, &ext_out_port_ptr->gu, &media_fmt_msg, media_fmt_opcode));

      TRY(result, cu_send_media_fmt_update_to_downstream(&me_ptr->cu, &ext_out_port_ptr->gu, &media_fmt_msg, q_ptr));

      ext_out_port_ptr->out_media_fmt_changed = FALSE;
   }

   CATCH(result, OLC_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   return result;
}

/**
 * Currently only used during handle_prepare.
 */
ar_result_t olc_ext_out_port_apply_pending_media_fmt_cmd_path(void *base_ptr, gu_ext_out_port_t *ext_out_port_ptr)
{
   bool_t is_data_path = FALSE;
   return olc_ext_out_port_apply_pending_media_fmt((olc_t *)base_ptr,
                                                   (olc_ext_out_port_t *)ext_out_port_ptr,
                                                   is_data_path);
}


// Flushes the container output queue only
ar_result_t olc_flush_cnt_output_data_queue(olc_t *me_ptr, olc_ext_out_port_t *ext_out_port_ptr)
{
   ar_result_t result = AR_EOK;

   // Nothing to do if the external port is NULL
   if (!ext_out_port_ptr)
   {
      return AR_EOK;
   }

   if (ext_out_port_ptr->cu.out_bufmgr_node.buf_ptr)
   {
      olc_return_back_out_buf(me_ptr, ext_out_port_ptr);
   }

   ext_out_port_ptr->cu.out_bufmgr_node.buf_ptr = NULL;
   ext_out_port_ptr->buf.data_ptr               = NULL;
   ext_out_port_ptr->buf.actual_data_len        = 0;

   return result;
}

// Flushes the container output queue and the Satellite data path read queue
ar_result_t olc_flush_output_data_queue(olc_t *me_ptr, olc_ext_out_port_t *ext_out_port_ptr, bool_t is_flush)
{
   ar_result_t result = AR_EOK;

   // Nothing to do if the external port is NULL
   if (!ext_out_port_ptr)
   {
      return AR_EOK;
   }

   result = olc_flush_cnt_output_data_queue(me_ptr, ext_out_port_ptr);
   if (ext_out_port_ptr->rdp_ctrl_cfg_ptr)
   {
      result = sgm_flush_read_data_port(&me_ptr->spgm_info,
                                        ext_out_port_ptr->rdp_ctrl_cfg_ptr->sdm_port_index,
                                        is_flush,
                                        FALSE);
   }

   return result;
}

ar_result_t olc_post_operate_flush_on_subgraph(olc_t *me_ptr, gu_sg_t *gu_sg_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t         result           = AR_EOK;
   gen_topo_sg_t *     sg_ptr           = (gen_topo_sg_t *)gu_sg_ptr;
   olc_ext_out_port_t *ext_out_port_ptr = NULL;
   olc_ext_in_port_t * ext_in_port_ptr  = NULL;

   for (gu_module_list_t *module_list_ptr = sg_ptr->gu.module_list_ptr; (NULL != module_list_ptr);
        LIST_ADVANCE(module_list_ptr))
   {
      gen_topo_module_t *module_ptr = (gen_topo_module_t *)module_list_ptr->module_ptr;

      for (gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr; (NULL != out_port_list_ptr);
           LIST_ADVANCE(out_port_list_ptr))
      {
         gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
         if (out_port_ptr->gu.ext_out_port_ptr)
         {
            ext_out_port_ptr = (olc_ext_out_port_t *)out_port_ptr->gu.ext_out_port_ptr;
            if (ext_out_port_ptr->rdp_ctrl_cfg_ptr)
            {
               TRY(result,
                   sgm_flush_read_data_port(&me_ptr->spgm_info,
                                            ext_out_port_ptr->rdp_ctrl_cfg_ptr->sdm_port_index,
                                            TRUE,
                                            TRUE));
            }
         }
      }

      for (gu_input_port_list_t *in_port_list_ptr = module_ptr->gu.input_port_list_ptr; (NULL != in_port_list_ptr);
           LIST_ADVANCE(in_port_list_ptr))
      {
         gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;
         if (in_port_ptr->gu.ext_in_port_ptr)
         {
            ext_in_port_ptr = (olc_ext_in_port_t *)in_port_ptr->gu.ext_in_port_ptr;
            if (ext_in_port_ptr->wdp_ctrl_cfg_ptr)
            {
               TRY(result,
                   sgm_flush_write_data_port(&me_ptr->spgm_info,
                                             ext_in_port_ptr->wdp_ctrl_cfg_ptr->sdm_port_index,
                                             TRUE,
                                             TRUE));
            }
         }
      }
   }

   CATCH(result, OLC_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   return result;
}

ar_result_t olc_post_operate_flush(cu_base_t *base_ptr, spf_msg_t *cmd_msg)
{
   ar_result_t result = AR_EOK;
   INIT_EXCEPTION_HANDLING

   spf_cntr_sub_graph_list_t *sg_list_ptr;
   spf_msg_cmd_graph_mgmt_t * cmd_gmgmt_ptr;
   olc_t *                    me_ptr = (olc_t *)base_ptr;
   uint32_t                   log_id = me_ptr->topo.gu.log_id;

   spf_msg_header_t *header_ptr = (spf_msg_header_t *)cmd_msg->payload_ptr;

   VERIFY(result, header_ptr->payload_size >= sizeof(spf_msg_cmd_graph_mgmt_t));

   cmd_gmgmt_ptr = (spf_msg_cmd_graph_mgmt_t *)&header_ptr->payload_start;
   sg_list_ptr   = &cmd_gmgmt_ptr->sg_id_list;

   OLC_MSG(log_id,
           DBG_LOW_PRIO,
           "olc_handle_flush. num_ip_port_handle %lu, num_op_port_handle %lu,  "
           "num_sub_graph %lu",
           cmd_gmgmt_ptr->cntr_port_hdl_list.num_ip_port_handle,
           cmd_gmgmt_ptr->cntr_port_hdl_list.num_op_port_handle,
           sg_list_ptr->num_sub_graph);

   for (uint32_t i = 0; i < sg_list_ptr->num_sub_graph; i++)
   {
      uint32_t *sg_id_base_ptr = sg_list_ptr->sg_id_list_ptr;
      uint32_t  sg_id          = *(sg_id_base_ptr + i);
      gu_sg_t * sg_ptr         = gu_find_subgraph(me_ptr->cu.gu_ptr, sg_id);
      if (sg_ptr)
      {
         olc_post_operate_flush_on_subgraph(me_ptr, sg_ptr);
      }
   }

   for (uint32_t i = 0; i < cmd_gmgmt_ptr->cntr_port_hdl_list.num_ip_port_handle; i++)
   {
      gu_ext_in_port_t *ext_in_port_ptr =
         (gu_ext_in_port_t *)cmd_gmgmt_ptr->cntr_port_hdl_list.ip_port_handle_list_pptr[i];

      olc_ext_in_port_t *olc_ext_in_port_ptr = (olc_ext_in_port_t *)ext_in_port_ptr;
      if (olc_ext_in_port_ptr->wdp_ctrl_cfg_ptr)
      {
         TRY(result,
             sgm_flush_write_data_port(&me_ptr->spgm_info,
                                       olc_ext_in_port_ptr->wdp_ctrl_cfg_ptr->sdm_port_index,
                                       TRUE,
                                       TRUE));
      }
   }

   for (uint32_t i = 0; i < cmd_gmgmt_ptr->cntr_port_hdl_list.num_op_port_handle; i++)
   {
      gu_ext_out_port_t *ext_out_port_ptr =
         (gu_ext_out_port_t *)cmd_gmgmt_ptr->cntr_port_hdl_list.op_port_handle_list_pptr[i];

      olc_ext_out_port_t *olc_ext_out_port_ptr = (olc_ext_out_port_t *)ext_out_port_ptr;
      if (olc_ext_out_port_ptr->rdp_ctrl_cfg_ptr)
      {
         TRY(result,
             sgm_flush_read_data_port(&me_ptr->spgm_info,
                                      olc_ext_out_port_ptr->rdp_ctrl_cfg_ptr->sdm_port_index,
                                      TRUE,
                                      TRUE));
      }
   }

   CATCH(result, CU_MSG_PREFIX, log_id)
   {
   }

   return result;
}
