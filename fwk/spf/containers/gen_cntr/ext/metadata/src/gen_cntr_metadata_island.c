/**
 * \file gen_cntr_metadata.c
 * \brief
 *     This file contains functions that handle metadata.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"
#include "gen_cntr_utils.h"

/**
 * We don't need to access ext_inp_ref because all we do is checking if EOS entered the container
 * through ext in port. If so, we simply decrement a global ref counter and remove votes.
 */
ar_result_t gen_cntr_clear_eos(gen_topo_t *         topo_ptr,
                               void *               ext_inp_ref,
                               uint32_t             ext_inp_id,
                               module_cmn_md_eos_t *eos_metadata_ptr)
{
   gen_cntr_t *          me_ptr       = (gen_cntr_t *)GET_BASE_PTR(gen_cntr_t, topo, topo_ptr);
   gen_topo_eos_cargo_t *cntr_ref_ptr = (gen_topo_eos_cargo_t *)eos_metadata_ptr->cntr_ref_ptr;

   if (cntr_ref_ptr)
   {
      /**
       * did_eos_come_from_ext_in is set only when flushing EOS comes in on the ext in port of container.
       * - If flushing EOS gets converted to non-flushing within the container, then clear_eos is
       *   still called and below code is executed.
       * - If non-flushing EOS enters the container, no container ref (cargo) is created.
       */
      if (!cntr_ref_ptr->did_eos_come_from_ext_in)
      {
         return AR_EOK;
      }

      // Important note: by this time ext-in might be destroyed. so don't access it.
      GEN_CNTR_MSG_ISLAND(topo_ptr->gu.log_id,
                          DBG_HIGH_PRIO,
                          "MD_DBG: EoS 0x%p being cleared for ext input port ",
                          eos_metadata_ptr);

      // in intra-container SG case or source module case, this may not be coming from ext-in.
      if (me_ptr->total_flush_eos_stuck > 0)
      {
         me_ptr->total_flush_eos_stuck--;

         GEN_CNTR_MSG_ISLAND(topo_ptr->gu.log_id,
                             DBG_HIGH_PRIO,
                             "MD_DBG: Flushing EOS left %lu ",
                             me_ptr->total_flush_eos_stuck);

         /* When EOS leaves the container, if all inputs become data-flow-state=at_gap, then
          * we can release votes (for FTRT input and output only).
          * If any flush_eos_cnt exists, then it means that there's an EOS stuck in the middle & even though ext-in is
          * at gap, we cannot release votes . Corner cases:
          *  - same input receiving multiple EOSes
          *  - multiple inputs receiving different EOSes
          *  - flushing EOS becoming non-flushing in the middle.
          *  - EOS followed by data and again EOS before first one goes out.
          *  - some modules are not in data-flow-gap, while ext input is: this is not possible unless there are source
          *     modules.
          *  - ext in getting destroyed before EOS comes out.
          *  */

         if (0 == me_ptr->total_flush_eos_stuck)
         {
            GEN_CNTR_MSG_ISLAND(topo_ptr->gu.log_id,
                                DBG_HIGH_PRIO,
                                "MD_DBG: As no more flushing EOSes are left, trying to remove vote from power manager");

            if (FALSE == me_ptr->topo.flags.defer_voting_on_dfs_change)
            {
               CU_SET_ONE_FWK_EVENT_FLAG(&me_ptr->cu, dfs_change);
            }

            GEN_CNTR_MSG_ISLAND(me_ptr->topo.gu.log_id,
                                DBG_HIGH_PRIO,
                                "gen_cntr_clear_eos defer_voting_on_dfs_change:%lu",
                                me_ptr->topo.flags.defer_voting_on_dfs_change);
         }
         else
         {
            GEN_CNTR_MSG_ISLAND(topo_ptr->gu.log_id,
                                DBG_HIGH_PRIO,
                                "MD_DBG: total_flush_eos_left %lu, hence not trying to remove votes yet",
                                me_ptr->total_flush_eos_stuck);
         }
      }
   }
   else
   {
      // if container reference is not present, eos could have been created an internal module (Ex: DTMF gen)
      // in that case dfs change needs to be marked to free the votes

      GEN_CNTR_MSG_ISLAND(topo_ptr->gu.log_id,
                          DBG_HIGH_PRIO,
                          "MD_DBG: Clearing EOS without container reference, is_flushing %u",
                          eos_metadata_ptr->flags.is_flushing_eos);

      if (eos_metadata_ptr->flags.is_flushing_eos)
      {
         if (FALSE == me_ptr->topo.flags.defer_voting_on_dfs_change)
         {
            CU_SET_ONE_FWK_EVENT_FLAG(&me_ptr->cu, dfs_change);
         }

         GEN_CNTR_MSG_ISLAND(me_ptr->topo.gu.log_id,
                             DBG_HIGH_PRIO,
                             "gen_cntr_clear_eos internal EOS defer_voting_on_dfs_change:%lu",
                             me_ptr->topo.flags.defer_voting_on_dfs_change);
      }
   }

   return AR_EOK;
}

ar_result_t gen_cntr_process_eos_md_from_peer_cntr_util_(gen_cntr_t             *me_ptr,
                                                         gen_cntr_ext_in_port_t *ext_in_port_ptr,
                                                         module_cmn_md_list_t  **md_list_head_pptr,
                                                         module_cmn_md_list_t   *eos_md_list_node_ptr)
{
   ar_result_t           result      = AR_EOK;
   bool_t                is_flushing = FALSE;
   module_cmn_md_list_t *node_ptr    = eos_md_list_node_ptr;

   module_cmn_md_t *md_ptr = node_ptr->obj_ptr;

   if (MODULE_CMN_MD_ID_EOS == md_ptr->metadata_id)
   {
      module_cmn_md_eos_t *eos_metadata_ptr = 0;
      uint32_t             is_out_band      = md_ptr->metadata_flag.is_out_of_band;
      if (is_out_band)
      {
         eos_metadata_ptr = (module_cmn_md_eos_t *)md_ptr->metadata_ptr;
      }
      else
      {
         eos_metadata_ptr = (module_cmn_md_eos_t *)&(md_ptr->metadata_buf);
      }
      is_flushing = eos_metadata_ptr->flags.is_flushing_eos;

      GEN_CNTR_MSG_ISLAND(me_ptr->topo.gu.log_id,
                   DBG_MED_PRIO,
                   "MD_DBG: gen_cntr received EOS cmd from Peer container at Module, port id (0x%lX, 0x%lx). "
                   "is_flushing "
                   "%u, node_ptr 0x%p, offset %lu",
                   ext_in_port_ptr->gu.int_in_port_ptr->cmn.module_ptr->module_instance_id,
                   ext_in_port_ptr->gu.int_in_port_ptr->cmn.id,
                   is_flushing,
                   node_ptr,
                   md_ptr->offset);

      // we don't need to make EOS as non-flushing EOE STM containers because, after data-flow-gap, we no longer
      // underrun.

      gen_topo_eos_cargo_t *cargo_ptr = NULL;

      if (is_flushing)
      {
         gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)ext_in_port_ptr->gu.int_in_port_ptr;
         result                             = gen_topo_create_eos_cntr_ref(&me_ptr->topo,
                                               me_ptr->cu.heap_id,
                                               in_port_ptr,
                                               ext_in_port_ptr->cu.id,
                                               &cargo_ptr);
         if (AR_DID_FAIL(result))
         {
            gen_topo_capi_metadata_destroy((void *)ext_in_port_ptr->gu.int_in_port_ptr->cmn.module_ptr,
                                           node_ptr,
                                           TRUE /*is_dropped*/,
                                           md_list_head_pptr,
										   0,
										   FALSE);
         }
         else
         {
            // do not change the offset as upstream sends with correct offsets

            me_ptr->total_flush_eos_stuck++;

            eos_metadata_ptr->cntr_ref_ptr = cargo_ptr;
            // We cannot set data flow state we move EOS out of this port. Reason: data and EOS can exist together in
            // the last buf. set port state only after all above are successful. this is call for peer container for
            // which MD is
            // already allocated by upstream
            // gen_topo_handle_data_flow_end(&me_ptr->topo, &in_port_ptr->common, &in_port_ptr->gu.cmn);
         }
      }

      // even though EoS is also input_discontinuity, it's handled separately
      // process any partially processed data
   }

   return result;
}
