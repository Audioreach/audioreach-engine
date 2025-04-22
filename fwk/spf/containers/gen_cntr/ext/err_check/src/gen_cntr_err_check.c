/**
 * \file gen_cntr_err_check.c
 * \brief
 *     This file contains functions that do optional error checking.
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"
#include "gen_cntr_utils.h"
#include "apm.h"

static bool_t gen_cntr_is_ep_threshold_an_int_factor(uint64_t ep_threshold_us, uint64_t mod_thresh_us)
{
    //check to avoid division by zero error
    if(0 == mod_thresh_us)
    {
        return FALSE;
    }
   uint64_t q = ep_threshold_us / mod_thresh_us;
   if ((q * mod_thresh_us) == ep_threshold_us)
   {
      return TRUE;
   }
   return FALSE;
}

/** in GEN_CNTR, it's ok to have multiple thresholds. but EP threshold should be int multiple of other modules thresh.
 * Otherwise, EP may underrun. Example: 1 ms HW EP and 10 ms module is not ok. 10 ms EP but 1 ms module is ok.
 *
 * */

ar_result_t gen_cntr_check_for_multiple_thresh_modules(gen_cntr_t *me_ptr)
{
   ar_result_t        result             = AR_EOK;
   uint64_t           ep_thresh_us       = 0;
   gen_topo_module_t *ep_ptr             = NULL;
   uint32_t           ep_port_id         = 0; // port from which overall thresh is obtained
   bool_t             is_media_fmt_valid = FALSE;

   ep_ptr = gen_cntr_get_stm_module(me_ptr);
   if (NULL == ep_ptr)
   {
      // maybe EP module is not created yet.
      return result;
   }

   if (ep_ptr->gu.output_port_list_ptr)
   {
      gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)ep_ptr->gu.output_port_list_ptr->op_port_ptr;
      is_media_fmt_valid                   = out_port_ptr ? out_port_ptr->common.flags.is_mf_valid : FALSE;
      if (is_media_fmt_valid && (out_port_ptr->common.flags.port_has_threshold) &&
          SPF_IS_PACKETIZED_OR_PCM(out_port_ptr->common.media_fmt_ptr->data_format))
      {
         ep_thresh_us = topo_bytes_to_us(gen_topo_get_curr_port_threshold(&out_port_ptr->common),
                                         out_port_ptr->common.media_fmt_ptr,
                                         NULL);
      }
   }
   else if (ep_ptr->gu.input_port_list_ptr)
   {
      gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)ep_ptr->gu.input_port_list_ptr->ip_port_ptr;
      is_media_fmt_valid                 = in_port_ptr ? in_port_ptr->common.flags.is_mf_valid : FALSE;
      if (is_media_fmt_valid && (in_port_ptr->common.flags.port_has_threshold) &&
          SPF_IS_PACKETIZED_OR_PCM(in_port_ptr->common.media_fmt_ptr->data_format))
      {
         ep_thresh_us = topo_bytes_to_us(gen_topo_get_curr_port_threshold(&in_port_ptr->common),
                                         in_port_ptr->common.media_fmt_ptr,
                                         NULL);
      }
   }

   // if media fmt is not yet valid, check for thresh is made once again when med fmt becomes valid. skip now.
   if (!is_media_fmt_valid)
   {
      return AR_EOK;
   }

   if (0 == ep_thresh_us)
   {
      // threshold of EP module must be nonzero, but until media fmt flows
      GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                   DBG_ERROR_PRIO,
                   "EP module 0x%lX has zero threshold",
                   ep_ptr->gu.module_instance_id);
      return AR_EFAILED;
   }

   for (gu_sg_list_t *sg_list_ptr = me_ptr->topo.gu.sg_list_ptr; (NULL != sg_list_ptr); LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         gen_topo_module_t *module_ptr = (gen_topo_module_t *)module_list_ptr->module_ptr;

         // Module has thresh if any port has threshold.
         for (gu_output_port_list_t *out_port_list_ptr = module_ptr->gu.output_port_list_ptr;
              (NULL != out_port_list_ptr);
              LIST_ADVANCE(out_port_list_ptr))
         {
            gen_topo_output_port_t *out_port_ptr = (gen_topo_output_port_t *)out_port_list_ptr->op_port_ptr;
            if (out_port_ptr->common.flags.port_has_threshold &&
                SPF_IS_PCM_DATA_FORMAT(out_port_ptr->common.media_fmt_ptr->data_format))
            {
               uint64_t module_thresh_val = topo_bytes_to_us(gen_topo_get_curr_port_threshold(&out_port_ptr->common),
                                                             out_port_ptr->common.media_fmt_ptr,
                                                             NULL);
               if (!gen_cntr_is_ep_threshold_an_int_factor(ep_thresh_us, module_thresh_val))
               {
                  GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                               DBG_ERROR_PRIO,
                               "Modules with non-multiple thresholds found. Overall (%lu, 0x%lX, 0x%lx); This (%lu, "
                               "0x%lX, 0x%lx). This will not work!!",
                               (uint32_t)ep_thresh_us,
                               ep_ptr->gu.module_instance_id,
                               ep_port_id,
                               (uint32_t)module_thresh_val,
                               module_ptr->gu.module_instance_id,
                               out_port_ptr->gu.cmn.id);
                  result = AR_EFAILED;
               }
               else if(ep_thresh_us != module_thresh_val)
               {
                  /**
                   *  If a module threshold doesn't match STM module, then this container
                   *  cannot use Pure signal triggered topo
                   */
                  me_ptr->topo.flags.cannot_be_pure_signal_triggered = TRUE;
                  GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                               DBG_HIGH_PRIO,
                               "Warning! Modules with multiple thresholds found. Overall (%lu, 0x%lX, 0x%lx); This (%lu, "
                               "0x%lX, 0x%lx). This cntr cannot use Pure signal trigger Topo!!",
                               (uint32_t)ep_thresh_us,
                               ep_ptr->gu.module_instance_id,
                               ep_port_id,
                               (uint32_t)module_thresh_val,
                               module_ptr->gu.module_instance_id,
                               out_port_ptr->gu.cmn.id);
               }
            }
         }

         for (gu_input_port_list_t *in_port_list_ptr = module_ptr->gu.input_port_list_ptr; (NULL != in_port_list_ptr);
              LIST_ADVANCE(in_port_list_ptr))
         {
            gen_topo_input_port_t *in_port_ptr = (gen_topo_input_port_t *)in_port_list_ptr->ip_port_ptr;
            if (in_port_ptr->common.flags.port_has_threshold &&
                SPF_IS_PCM_DATA_FORMAT(in_port_ptr->common.media_fmt_ptr->data_format))
            {
               uint64_t module_thresh_val = topo_bytes_to_us(gen_topo_get_curr_port_threshold(&in_port_ptr->common),
                                                             in_port_ptr->common.media_fmt_ptr,
                                                             NULL);
               if (!gen_cntr_is_ep_threshold_an_int_factor(ep_thresh_us, module_thresh_val))
               {
                  GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                               DBG_ERROR_PRIO,
                               "Modules with non-multiple thresholds found. Overall (%lu, 0x%lX, 0x%lx); This (%lu, "
                               "0x%lX, 0x%lx). This will not work!!",
                               (uint32_t)ep_thresh_us,
                               ep_ptr->gu.module_instance_id,
                               ep_port_id,
                               (uint32_t)module_thresh_val,
                               module_ptr->gu.module_instance_id,
                               in_port_ptr->gu.cmn.id);
                  result = AR_EFAILED;
               }
               else if(ep_thresh_us != module_thresh_val)
               {
                  /**
                   *  If a module threshold doesn't match STM module, then this container
                   *  cannot use Pure signal triggered topo
                   */
                  me_ptr->topo.flags.cannot_be_pure_signal_triggered = TRUE;

                  GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                               DBG_HIGH_PRIO,
                               "Warning! Modules with multiple thresholds found. Overall (%lu, 0x%lX, 0x%lx); This (%lu, "
                               "0x%lX, 0x%lx). This cntr cannot use Pure signal trigger Topo!!",
                               (uint32_t)ep_thresh_us,
                               ep_ptr->gu.module_instance_id,
                               ep_port_id,
                               (uint32_t)module_thresh_val,
                               module_ptr->gu.module_instance_id,
                               in_port_ptr->gu.cmn.id);
               }
            }
         }
      }
   }

   return result;
}
