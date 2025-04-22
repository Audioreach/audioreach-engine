/**
 * \file gen_cntr_cmn_sh_mem_ep.c
 * \brief
 *     This file contains functions for read shared memory end point
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "gen_cntr_i.h"
#include "apm.h"
#include "media_fmt_extn_api.h"

ar_result_t gen_cntr_shmem_cmn_validate_peer_client_property_configuration(uint32_t num_properties, sh_mem_peer_client_property_payload_t *port_property_payload_ptr)
{
   ar_result_t result = AR_EOK;

   for (uint32_t i = 0; i < num_properties; i++)
   {
      switch (port_property_payload_ptr[i].property_type)
      {
         case PROPERTY_TYPE_IS_UPSTREAM_REALTIME:
         case PROPERTY_TYPE_IS_DOWNSTREAM_REALTIME:
         {
            if (!((PROPERTY_VALUE_IS_FTRT == port_property_payload_ptr[i].property_value) ||
                  (PROPERTY_VALUE_IS_RT == port_property_payload_ptr[i].property_value)))
            {
               result = AR_EBADPARAM;
            }
            break;
         }
         case PROPERTY_TYPE_IS_INVALID:
         {
            break;
         }
         default:
         {
            result = AR_EBADPARAM;
         }
      }
   }

   return result;
}

ar_result_t gen_cntr_shmem_cmn_process_and_apply_peer_client_property_configuration(cu_base_t *   base_ptr,
                                                                                    spf_handle_t *dst_handle_ptr,
                                                                                    int8_t *      payload_ptr,
                                                                                    uint32_t      param_size)
{
   ar_result_t result                    = AR_EOK;
   bool_t      need_to_update_states     = FALSE;
   uint32_t    get_need_to_update_states = FALSE;
   uint32_t    req_psize                 = 0;
   gen_cntr_t *me_ptr                    = (gen_cntr_t *)base_ptr;

   param_id_sh_mem_peer_client_property_config_t *peer_property_payload_ptr =
      (param_id_sh_mem_peer_client_property_config_t *)payload_ptr;

   peer_property_payload_ptr = (param_id_sh_mem_peer_client_property_config_t *)payload_ptr;
   sh_mem_peer_client_property_payload_t *cur_ptr =
      (sh_mem_peer_client_property_payload_t *)(peer_property_payload_ptr + 1);

   req_psize = sizeof(param_id_sh_mem_peer_client_property_config_t);
   req_psize += peer_property_payload_ptr->num_properties * sizeof(sh_mem_peer_client_property_payload_t);
   if (param_size < req_psize)
   {
      GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                   DBG_HIGH_PRIO,
                   "processing peer port property configuration, "
                   "required payload size %lu is less than actual payload size %lu",
                   req_psize,
                   param_size);
      return result;
   }

   result = gen_cntr_shmem_cmn_validate_peer_client_property_configuration(peer_property_payload_ptr->num_properties,
                                                                           cur_ptr);
   if (result)
   {
      GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                   DBG_HIGH_PRIO,
                   "shmem_peer_client_property_configuration, ebadparam failed to apply the config");
      return result;
   }

   for (uint32_t i = 0; i < peer_property_payload_ptr->num_properties; i++)
   {
      result |= cu_process_peer_port_property(base_ptr,
                                              dst_handle_ptr,
                                              cur_ptr[i].property_type,
                                              cur_ptr[i].property_value,
                                              &get_need_to_update_states);
      if (get_need_to_update_states)
      {
         need_to_update_states = TRUE;
      }
   }

   // after RT/FTRT re-assign the states because states are not propagated for FTRT
   if (need_to_update_states)
   {
      result |= cu_update_all_sg_port_states(base_ptr, FALSE);

      // state prop and RT/FTRT is complete now. inform upstream or downstream such that propagation across container
      // happens
      result |= cu_inform_downstream_about_upstream_property(base_ptr);
      result |= cu_inform_upstream_about_downstream_property(base_ptr);
   }

   return result;
}
