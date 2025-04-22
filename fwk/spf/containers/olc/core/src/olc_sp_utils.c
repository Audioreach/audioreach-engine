/**
 * \file olc_sp_util.c
 * \brief
 *     This file contains olc utility functions for managing state propagation.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "olc_i.h"

ar_result_t olc_handle_peer_port_property_from_satellite_upstream(cu_base_t *                        base_ptr,
                                                                  uint32_t                           channel_bit_index,
                                                                  spf_msg_peer_port_property_info_t *property_ptr)
{
   ar_result_t               result                = AR_EOK;
   olc_t *                   me_ptr                = (olc_t *)base_ptr;
   olc_ext_out_port_t *      ext_out_port_ptr      = NULL;
   topo_port_property_type_t prop_type             = property_ptr->property_type;
   void *                    payload_ptr           = &property_ptr->property_value;
   bool_t                    continue_propagation  = FALSE;
   bool_t                    need_to_update_states = TRUE;

   ext_out_port_ptr = (olc_ext_out_port_t *)cu_get_ext_out_port_for_bit_index(&me_ptr->cu, channel_bit_index);

   if (NULL == ext_out_port_ptr)
   {
      return AR_EUNEXPECTED;
   }

   if (PORT_PROPERTY_IS_UPSTREAM_RT == prop_type)
   {
      need_to_update_states = (property_ptr->property_value != ext_out_port_ptr->cu.icb_info.flags.is_real_time);
   }

   gen_topo_set_get_propagated_property_on_the_output_port(&me_ptr->topo,
                                                           (gen_topo_output_port_t *)
                                                              ext_out_port_ptr->gu.int_out_port_ptr,
                                                           prop_type,
                                                           payload_ptr,
                                                           &continue_propagation);

   olc_set_propagated_prop_on_ext_output(&me_ptr->topo, &ext_out_port_ptr->gu, prop_type, payload_ptr);
   cu_offload_process_peer_port_property_propagation(base_ptr, need_to_update_states);

   return result;
}

ar_result_t olc_handle_peer_port_property_from_satellite_downstream(cu_base_t *base_ptr,
                                                                    uint32_t   channel_bit_index,
                                                                    spf_msg_peer_port_property_info_t *property_ptr)
{
   ar_result_t               result                = AR_EOK;
   olc_t *                   me_ptr                = (olc_t *)base_ptr;
   olc_ext_in_port_t *       ext_in_port_ptr       = NULL;
   topo_port_property_type_t prop_type             = property_ptr->property_type;
   void *                    payload_ptr           = &property_ptr->property_value;
   bool_t                    continue_propagation  = FALSE;
   bool_t                    need_to_update_states = TRUE;

   ext_in_port_ptr = (olc_ext_in_port_t *)cu_get_ext_in_port_for_bit_index(&me_ptr->cu, channel_bit_index);

   if (NULL == ext_in_port_ptr)
   {
      return AR_EUNEXPECTED;
   }

   if (PORT_PROPERTY_IS_DOWNSTREAM_RT == prop_type)
   {
      need_to_update_states = (property_ptr->property_value != ext_in_port_ptr->cu.icb_info.flags.is_real_time);
   }

   gen_topo_set_get_propagated_property_on_the_input_port(&me_ptr->topo,
                                                          (gen_topo_input_port_t *)ext_in_port_ptr->gu.int_in_port_ptr,
                                                          prop_type,
                                                          payload_ptr,
                                                          &continue_propagation);

   olc_set_propagated_prop_on_ext_input(&me_ptr->topo, &ext_in_port_ptr->gu, prop_type, payload_ptr);
   cu_offload_process_peer_port_property_propagation(base_ptr, need_to_update_states);

   if ((PORT_PROPERTY_TOPO_STATE == prop_type) && (TOPO_PORT_STATE_STOPPED == property_ptr->property_value))
   {
      spdm_process_upstream_stopped(&me_ptr->spgm_info, ext_in_port_ptr->wdp_ctrl_cfg_ptr->sdm_port_index);
   }

   return result;
}

ar_result_t olc_reset_downstream_and_send_internal_eos(cu_base_t *base_ptr, uint32_t channel_bit_index)
{
   ar_result_t         result           = AR_EOK;
   olc_t *             me_ptr           = (olc_t *)base_ptr;
   olc_ext_out_port_t *ext_out_port_ptr = NULL;
   topo_sg_state_t     sg_state         = 0;

   ext_out_port_ptr = (olc_ext_out_port_t *)cu_get_ext_out_port_for_bit_index(&me_ptr->cu, channel_bit_index);

   if (NULL == ext_out_port_ptr)
   {
      return AR_EUNEXPECTED;
   }

   /** need to send EOS only for ext port which is connected and which is not in stop or invalid states.
     *  Because if last module is stopped, neighbouring container also knows about it through APM.*/
   if (ext_out_port_ptr && ext_out_port_ptr->gu.downstream_handle.spf_handle_ptr &&
       (TOPO_SG_STATE_STOPPED != sg_state) && (TOPO_SG_STATE_INVALID != sg_state))
   {
      // send EOS to external clients if ext port is in different SG
      uint32_t                  INPUT_PORT_ID_NONE = 0; // NULL input port -> don't care input port id.
      module_cmn_md_eos_flags_t eos_md_flag        = { .word = 0 };
      eos_md_flag.is_flushing_eos                  = TRUE;
      eos_md_flag.is_internal_eos                  = TRUE;

      result = gen_topo_create_eos_for_cntr(&me_ptr->topo,
                                            NULL, /* input_port_ptr*/
                                            INPUT_PORT_ID_NONE,
                                            me_ptr->cu.heap_id,
                                            &ext_out_port_ptr->md_list_ptr,
                                            NULL, /* md_flag_ptr */
                                            NULL, /*tracking_payload_ptr*/
                                            &eos_md_flag, /* eos_payload_flags */
                                            ext_out_port_ptr->buf.actual_data_len,
                                            &ext_out_port_ptr->cu.media_fmt);

      OLC_MSG(me_ptr->topo.gu.log_id,
              DBG_LOW_PRIO,
              "MD_DBG: Created EOS for ext out port (0x%0lX, 0x%lx) with result 0x%lx",
              ext_out_port_ptr->gu.int_out_port_ptr->cmn.module_ptr->module_instance_id,
              ext_out_port_ptr->gu.int_out_port_ptr->cmn.id,
              result);

      // if there's a buffer already popped use it
      {
         // unlike regular data path messages this message will use buf mgr buffer not bufQ buf
         if (AR_DID_FAIL(olc_create_send_eos_md(me_ptr, ext_out_port_ptr)))
         {
            gen_topo_destroy_all_metadata(me_ptr->topo.gu.log_id,
                                          (void *)ext_out_port_ptr->gu.int_out_port_ptr->cmn.module_ptr,
                                          &ext_out_port_ptr->md_list_ptr,
                                          TRUE /*is_dropped*/);
         }
         olc_ext_out_port_basic_reset(me_ptr, ext_out_port_ptr);
      }
   }

   return result;
}
