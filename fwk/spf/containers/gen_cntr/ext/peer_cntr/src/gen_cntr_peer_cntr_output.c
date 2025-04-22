/**
 * \file gen_cntr_peer_cntr_output.c
 * \brief
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"
#include "apm.h"

ar_result_t gen_cntr_create_send_media_fmt_to_peer_cntr(gen_cntr_t *me_ptr, gen_cntr_ext_out_port_t *ext_out_port_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;
   if (ext_out_port_ptr->gu.downstream_handle.spf_handle_ptr)
   {
      gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)(ext_out_port_ptr->gu.int_out_port_ptr);
      topo_port_state_t       self_sg_state =
         topo_sg_state_to_port_state(gen_topo_get_sg_state(out_port_ptr->gu.cmn.module_ptr->sg_ptr));
      topo_port_state_t conn_port_state       = ext_out_port_ptr->cu.connected_port_state;
      topo_port_state_t downgraded_port_state = out_port_ptr->common.state;

      // There is no point is sending media format if self and downstream is not in started/prepared state
      if ((TOPO_PORT_STATE_STARTED == self_sg_state || TOPO_PORT_STATE_PREPARED == self_sg_state) &&
          (TOPO_PORT_STATE_STARTED == conn_port_state || TOPO_PORT_STATE_PREPARED == conn_port_state))
      {
         if (TOPO_PORT_STATE_STARTED != downgraded_port_state)
         {
            spf_msg_t media_fmt_msg = { 0 };

            //This will take care of sending upstream frame len if it is known by this time
            TRY(result,
                cu_create_media_fmt_msg_for_downstream(&me_ptr->cu,
                                                       &ext_out_port_ptr->gu,
                                                       &media_fmt_msg,
                                                       SPF_MSG_CMD_MEDIA_FORMAT));

            TRY(result,
                cu_send_media_fmt_update_to_downstream(&me_ptr->cu,
                                                       &ext_out_port_ptr->gu,
                                                       &media_fmt_msg,
                                                       ext_out_port_ptr->gu.downstream_handle.spf_handle_ptr
                                                          ->cmd_handle_ptr->cmd_q_ptr));

            ext_out_port_ptr->flags.out_media_fmt_changed = FALSE;
            ext_out_port_ptr->cu.flags.upstream_frame_len_changed = FALSE;
         }
      }
   }
   CATCH(result, GEN_CNTR_MSG_PREFIX, me_ptr->topo.gu.log_id)
   {
   }

   return result;
}

/**
 * Currently only used for handle prepare. gen_cntr_ext_out_port_apply_pending_media_fmt
 */
ar_result_t gen_cntr_ext_out_port_apply_pending_media_fmt(void *ctx_ptr, gu_ext_out_port_t *ext_out_port_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t              result                    = AR_EOK;
   gen_cntr_t              *me_ptr                    = (gen_cntr_t *)ctx_ptr;
   cu_base_t               *base_ptr                  = (cu_base_t *)ctx_ptr;
   gen_cntr_ext_out_port_t *gen_cntr_ext_out_port_ptr = (gen_cntr_ext_out_port_t *)ext_out_port_ptr;
   gen_topo_output_port_t  *out_port_ptr              = (gen_topo_output_port_t *)ext_out_port_ptr->int_out_port_ptr;
   gen_topo_module_t       *module_ptr                = (gen_topo_module_t *)out_port_ptr->gu.cmn.module_ptr;

   // As an optimization, skip applying pending media format on stopped/suspended ports
   // This is necessary because, as part of applying ext media fmt, media fmt event is cleared from internal port.
   // In case if the port is stopped, mf event is cleared from port but container threshold change event is not set
   // during mf propagation i.e prop_media_fmt() because it checks if module is started/prepared. And later when the SG
   // is started there is threshold event is not set, since output mf is already cleared and container may never
   // propagate threshold leading to zero threshold
   if (gen_topo_is_module_sg_stopped_or_suspended(module_ptr))
   {
#ifdef VERBOSE_DEBUGGING
      AR_MSG(DBG_LOW_PRIO,
             "Module 0x%lX output 0x%lx: Module is stopped skip moving media format from internal input to external "
             "output port",
             out_port_ptr->gu.cmn.module_ptr->module_instance_id,
             out_port_ptr->gu.cmn.id);
#endif
      return AR_EOK;
   }

   // Copy media format from internal to external port. Note that we should
   // not have any pending data in the output port when we receive prepare, so
   // we don't have to worry about sending data or flushing stale data.
   gen_cntr_get_output_port_media_format_from_topo(gen_cntr_ext_out_port_ptr,
                                                   TRUE /* update_med_fmt_to_unchanged*/ );

   //recreate output buffer if media format is changed. This is needed here as this function
   //may get called after already allocating buffers with previously set media format.
   //For compressed out, prebuffers shouldn't get created.
   if (gen_cntr_ext_out_port_ptr->flags.out_media_fmt_changed)
   {
      gen_cntr_ext_out_port_recreate_bufs((void *)&me_ptr->cu, ext_out_port_ptr);
   }

   if ((gen_cntr_ext_out_port_ptr->vtbl_ptr->prop_media_fmt) &&
       (gen_cntr_ext_out_port_ptr->cu.media_fmt.data_format != SPF_UNKNOWN_DATA_FORMAT))
   {
      TRY(result, gen_cntr_ext_out_port_ptr->vtbl_ptr->prop_media_fmt(me_ptr, gen_cntr_ext_out_port_ptr));
   }

   CATCH(result, GEN_CNTR_MSG_PREFIX, base_ptr->gu_ptr->log_id)
   {
   }

   return AR_EOK;
}

ar_result_t gen_cntr_flush_output_data_queue_peer_cntr(gen_cntr_t *             me_ptr,
                                                       gen_cntr_ext_out_port_t *ext_out_port_ptr,
                                                       bool_t                   is_client_cmd)
{
   if (ext_out_port_ptr->cu.out_bufmgr_node.buf_ptr)
   {
      gen_cntr_return_back_out_buf(me_ptr, ext_out_port_ptr);
   }

   // Reset all the topo output port buffer pointers
   gen_topo_output_port_t *topo_out_port_ptr = (gen_topo_output_port_t *)ext_out_port_ptr->gu.int_out_port_ptr;

   // ext out port might have attached to the previous module's int port due to
   // elementary module at the ext out. When ext out gets connected if any topo buffer
   // was assigned to the host module's int port it must be returned here.
   if (topo_out_port_ptr->common.bufs_ptr)
   {
      topo_out_port_ptr->common.flags.force_return_buf      = TRUE;
      topo_out_port_ptr->common.bufs_ptr[0].actual_data_len = 0;
      gen_topo_check_return_one_buf_mgr_buf(&me_ptr->topo,
                                            &topo_out_port_ptr->common,
                                            topo_out_port_ptr->gu.cmn.module_ptr->module_instance_id,
                                            topo_out_port_ptr->gu.cmn.id);

      for (uint32_t b = 0; b < topo_out_port_ptr->common.sdata.bufs_num; b++)
      {
         topo_out_port_ptr->common.bufs_ptr[b].data_ptr     = NULL;
         topo_out_port_ptr->common.bufs_ptr[b].max_data_len = 0;
      }
   }

   ext_out_port_ptr->cu.out_bufmgr_node.buf_ptr = NULL;

   return AR_EOK;
}

/* If the topo output port has a pending mf event, mf will be copied to ext port and out_media_fmt_changed flag is set.
   The flag out_media_fmt_changed must be cleared once the mf is sent to downstream container. */
ar_result_t gen_cntr_get_output_port_media_format_from_topo(gen_cntr_ext_out_port_t *            ext_out_port_ptr,
                                                            bool_t                               update_to_unchanged)
{
   topo_media_fmt_t *media_fmt_ptr           = &ext_out_port_ptr->cu.media_fmt;
   gen_topo_output_port_t *topo_out_port_ptr = (gen_topo_output_port_t *)ext_out_port_ptr->gu.int_out_port_ptr;

   topo_media_fmt_t *module_out_media_fmt_ptr = topo_out_port_ptr->common.media_fmt_ptr;

   if (topo_out_port_ptr->common.flags.media_fmt_event)
   {
      tu_copy_media_fmt(media_fmt_ptr, module_out_media_fmt_ptr);
      if (SPF_IS_PACKETIZED_OR_PCM(module_out_media_fmt_ptr->data_format))
      {
         // even if last module outputs deint-unpacked, container sends out deint-packed
         media_fmt_ptr->pcm.interleaving = TU_IS_ANY_DEINTERLEAVED_UNPACKED(module_out_media_fmt_ptr->pcm.interleaving)
                                              ? TOPO_DEINTERLEAVED_PACKED
                                              : module_out_media_fmt_ptr->pcm.interleaving;
      }

      if (update_to_unchanged)
      {
         topo_out_port_ptr->common.flags.media_fmt_event = FALSE;
      }

      // copy the flag, it can be used to avoid steady state checks in the external output context when packing
      // data to send to downstream
      ext_out_port_ptr->cu.flags.is_pcm_unpacked = topo_out_port_ptr->common.flags.is_pcm_unpacked;

      ext_out_port_ptr->flags.out_media_fmt_changed = TRUE;

#ifdef VERBOSE_DEBUGGING
      AR_MSG(DBG_MED_PRIO,
             "Moved mf to external output of module 0x%x, port 0x%x, flags: media fmt %d frame len %d "
             "df %d interleaving %d",
             ext_out_port_ptr->gu.int_out_port_ptr->cmn.module_ptr->module_instance_id,
             ext_out_port_ptr->gu.int_out_port_ptr->cmn.id,
             ext_out_port_ptr->flags.out_media_fmt_changed,
             ext_out_port_ptr->cu.flags.upstream_frame_len_changed,
             module_out_media_fmt_ptr->data_format,
             media_fmt_ptr->pcm.interleaving);
#endif
   }
   return AR_EOK;
}

static ar_result_t gen_cntr_update_ext_out_v2_bufs_info(gen_cntr_ext_out_port_t *ext_out_port_ptr, POSAL_HEAP_ID heap_id, uint32_t bufs_num_v2)
{
   ar_result_t result = AR_EOK;

   ext_out_port_ptr->bufs_num = bufs_num_v2;

   if (ext_out_port_ptr->bufs_ptr)
   {
      posal_memory_free(ext_out_port_ptr->bufs_ptr);
   }

   gen_cntr_buf_t *bufs_ptr =
       (gen_cntr_buf_t *)posal_memory_malloc(ext_out_port_ptr->bufs_num * sizeof(gen_cntr_buf_t), heap_id);

   if (!bufs_ptr)
   {
      return AR_ENOMEMORY;
   }

   memset(bufs_ptr, 0, ext_out_port_ptr->bufs_num * sizeof(gen_cntr_buf_t));

   ext_out_port_ptr->bufs_ptr = bufs_ptr;
   return result;
}

ar_result_t gen_cntr_recreate_out_buf_peer_cntr(gen_cntr_t *             me_ptr,
                                                gen_cntr_ext_out_port_t *ext_out_port_ptr,
                                                uint32_t                 req_out_buf_size,
                                                uint32_t                 num_data_msg,
                                                uint32_t                 num_bufs_per_data_msg)
{
   ar_result_t result = AR_EOK;

   if ((req_out_buf_size != ext_out_port_ptr->cu.buf_max_size) ||
       (num_data_msg != ext_out_port_ptr->cu.num_buf_allocated) || (num_bufs_per_data_msg != ext_out_port_ptr->bufs_num))
   {
      /** if buffer was already popped and is returned now, then we need to pop it back after
       * recreate. */
      bool_t buf_was_present = ext_out_port_ptr->cu.out_bufmgr_node.buf_ptr != NULL;

      uint32_t num_bufs_to_keep = ((req_out_buf_size != ext_out_port_ptr->cu.buf_max_size) || (ext_out_port_ptr->bufs_num != num_bufs_per_data_msg)) ? 0 : num_data_msg;

      gen_cntr_destroy_ext_buffers(me_ptr, ext_out_port_ptr, num_bufs_to_keep);

      ext_out_port_ptr->cu.buf_max_size = req_out_buf_size;

      if (ext_out_port_ptr->bufs_num != num_bufs_per_data_msg)
      {
         result |= gen_cntr_update_ext_out_v2_bufs_info(ext_out_port_ptr, me_ptr->cu.heap_id, num_bufs_per_data_msg);
      }

      result |= gen_cntr_create_ext_out_bufs(me_ptr, ext_out_port_ptr, num_data_msg);

      if (buf_was_present && (ext_out_port_ptr->cu.out_bufmgr_node.buf_ptr == NULL))
      {
         gen_cntr_process_pop_out_buf(me_ptr, ext_out_port_ptr);

         if (ext_out_port_ptr->cu.out_bufmgr_node.buf_ptr)
         {
            gen_cntr_init_after_popping_peer_cntr_out_buf(me_ptr, ext_out_port_ptr);
         }
      }
   }
   return result;
}
