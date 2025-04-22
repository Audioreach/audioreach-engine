/**
 * \file gen_topo_module_bypass.c
 * \brief
 *     This file contains utility for handling disabled modules
 *
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"

#define BYPASS_BW (100 * 1024)

static bool_t gen_topo_is_module_bypassable(gen_topo_module_t *module_ptr)
{
   /**
    * checking max port because, if we check num_ports, then while disabled, new ports may be created.
    * bypass not possible to achieve for MIMO.
    *
    * bypass possible for SISO only.
    *
    */
   if ((module_ptr->gu.max_input_ports != 1) || (module_ptr->gu.max_output_ports != 1))
   {
      return FALSE;
   }

   // if module can act as pseudo source and pseudo sink then it is not true SISO module and can not be bypassed.
   if ((0 == module_ptr->gu.min_input_ports) || 0 == (module_ptr->gu.min_output_ports))
   {
      return FALSE;
   }

   // Signal triggered module cannot be bypassed.
   if (module_ptr->flags.need_stm_extn)
   {
      return FALSE;
   }

   /**
    * Placeholder enc/dec may be disabled until they are set with a real-module-id
    * module_type of placeholder is changed once real-module-id is set
    */
   /**
    * Only generic, PP and framework modules may be disabled (except placeholder enc/dec)
    * enc, dec, pack, depack, detector, generator, end points cannot be bypassed.
    */
   if (!((AMDB_MODULE_TYPE_GENERIC == module_ptr->gu.module_type) ||
         (AMDB_MODULE_TYPE_FRAMEWORK == module_ptr->gu.module_type) ||
         (AMDB_MODULE_TYPE_PP == module_ptr->gu.module_type)))
   {
      return FALSE;
   }

   /**
    * Trigger policy modules cannot be bypassed because evaluating trigger policy
    * is not possible when module is disabled. Switching b/w default trigger policy and
    * module trigger policy needs additional mem alloc.
    */
   if (gen_topo_is_module_data_trigger_policy_active(module_ptr))
   {
      return FALSE;
   }

   /**
    * A threshold module can be bypassed.
    * Threshold of bypassed module is used for allocation of buffers.
    */

   return TRUE;
}

static ar_result_t gen_topo_switch_bypass_module(gen_topo_t        *topo_ptr,
                                                 gen_topo_module_t *module_ptr,
                                                 bool_t             bypass_enable)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

   gen_topo_capi_event_flag_t capi_event_flags = { .word = 0 };

   VERIFY(result, topo_ptr->gen_topo_vtable_ptr->get_out_port_data_len);

   if ((1 == module_ptr->gu.num_input_ports) && (1 == module_ptr->gu.num_output_ports))
   {
      gen_topo_output_port_t *output_port_ptr =
         (gen_topo_output_port_t *)module_ptr->gu.output_port_list_ptr->op_port_ptr;
      gen_topo_input_port_t *input_port_ptr = (gen_topo_input_port_t *)module_ptr->gu.input_port_list_ptr->ip_port_ptr;

      topo_media_fmt_t *src_media_fmt_ptr = NULL;

      if (bypass_enable)
      {
         // back-up output media fmt
         tu_set_media_fmt_from_port(&topo_ptr->mf_utils,
                                    &module_ptr->bypass_ptr->media_fmt_ptr,
                                    output_port_ptr->common.media_fmt_ptr);

         // use input media fmt as output
         src_media_fmt_ptr = input_port_ptr->common.media_fmt_ptr;
      }
      else
      {
         // else restore output media fmt
         src_media_fmt_ptr = module_ptr->bypass_ptr->media_fmt_ptr;
      }

      // Create memory only if media format is different between this module's input and output
      if (topo_is_valid_media_fmt(src_media_fmt_ptr) &&
          tu_has_media_format_changed(src_media_fmt_ptr, output_port_ptr->common.media_fmt_ptr))
      {
         bool_t IS_MAX_FALSE = FALSE;
         if ((output_port_ptr->common.bufs_ptr) &&
             (0 != topo_ptr->gen_topo_vtable_ptr->get_out_port_data_len(topo_ptr, output_port_ptr, IS_MAX_FALSE)))
         {
            TOPO_MSG(topo_ptr->gu.log_id,
                     DBG_HIGH_PRIO,
                     "Warning: Module 0x%lX has data at the output while being bypass disabled/enabled (%u)",
                     module_ptr->gu.module_instance_id,
                     bypass_enable);
            // TODO: in this case, we need to propagate media fmt after this data is sent down.
         }

         tu_set_media_fmt_from_port(&topo_ptr->mf_utils, &output_port_ptr->common.media_fmt_ptr, src_media_fmt_ptr);
         output_port_ptr->common.flags.media_fmt_event = TRUE;
         capi_event_flags.media_fmt_event              = TRUE;
         output_port_ptr->common.flags.is_mf_valid     = TRUE;

         /** Reset pcm unpacked mask*/
         gen_topo_reset_pcm_unpacked_mask(&output_port_ptr->common);

         gen_topo_set_med_fmt_to_attached_module(topo_ptr, output_port_ptr);

         TOPO_MSG(topo_ptr->gu.log_id,
                  DBG_LOW_PRIO,
                  "Module 0x%lX has different media format between input or bypass and output while bypass "
                  "enable/disable (%u)",
                  module_ptr->gu.module_instance_id,
                  bypass_enable);
      }

      bool_t   kpps_changed = FALSE, bw_changed = FALSE, hw_acc_proc_delay_changed = FALSE;
      uint32_t bypass_kpps = 0;
      // KPPS/BW events are handled by container after looking into gen_topo_capi_event_flag_t::process_state flag
      if (bypass_enable)
      {
         module_ptr->bypass_ptr->kpps              = module_ptr->kpps;
         module_ptr->bypass_ptr->algo_delay        = module_ptr->algo_delay;
         module_ptr->bypass_ptr->code_bw           = module_ptr->code_bw;
         module_ptr->bypass_ptr->data_bw           = module_ptr->data_bw;
         module_ptr->bypass_ptr->hw_acc_proc_delay = module_ptr->hw_acc_proc_delay;

#if 0  //not required to vote memcpy overhead as module is made dynamic inplace.
         /*In 1 sec we transfer SR*bytes*ch bytes using memcpy. memcpy usually copies 8-bytes at a time,
         where it does 1 8-byte read and 1 8-byte write = 2 instructions for every 8-byte copy
         Therefore pps = 2*SR*bytes*ch/8 and kpps = pps/1000.*/
         bypass_kpps = topo_get_memscpy_kpps(module_ptr->bypass_ptr->media_fmt_ptr->pcm.bits_per_sample,
                                             module_ptr->bypass_ptr->media_fmt_ptr->pcm.num_channels,
                                             module_ptr->bypass_ptr->media_fmt_ptr->pcm.sample_rate);
#endif

         kpps_changed                  = (module_ptr->kpps != bypass_kpps);
         bw_changed                    = ((module_ptr->data_bw != BYPASS_BW) || (module_ptr->code_bw != 0));
         hw_acc_proc_delay_changed     = (module_ptr->hw_acc_proc_delay != 0);
         module_ptr->kpps              = bypass_kpps;
         module_ptr->algo_delay        = 0;
         module_ptr->code_bw           = 0;
         module_ptr->data_bw           = BYPASS_BW;
         module_ptr->hw_acc_proc_delay = 0;

         // Initialize bypassed thresholds to 0. If a module raises a threshold, this will become set at that time,
         // indicating threshold was raised while module was bypassed.
         module_ptr->bypass_ptr->in_thresh_bytes_all_ch  = 0;
         module_ptr->bypass_ptr->out_thresh_bytes_all_ch = 0;

         // Note on Metadata propagation: we set algo delay to zero above. Internal MD offsets which are based on
         // earlier algo delay must be set to zero.
         // to ensure EOS goes out, need to make pending zeros also zero. We are not moving internal MD to out port
         // because
         // that's taken care already in metadata propagation func. This avoids code duplication.
         module_cmn_md_list_t *node_ptr = module_ptr->int_md_list_ptr;
         while (node_ptr)
         {
            node_ptr->obj_ptr->offset = 0;
            node_ptr                  = node_ptr->next_ptr;
         }
         module_ptr->pending_zeros_at_eos = 0;
      }
      else
      {
         // bypass_disable case (module is getting enabled)
         kpps_changed              = (module_ptr->kpps != module_ptr->bypass_ptr->kpps);
         bw_changed                = ((module_ptr->data_bw != module_ptr->bypass_ptr->code_bw) ||
                       (module_ptr->code_bw != module_ptr->bypass_ptr->data_bw));
         hw_acc_proc_delay_changed = (module_ptr->hw_acc_proc_delay != module_ptr->bypass_ptr->hw_acc_proc_delay);

         module_ptr->kpps              = module_ptr->bypass_ptr->kpps;
         module_ptr->algo_delay        = module_ptr->bypass_ptr->algo_delay;
         module_ptr->code_bw           = module_ptr->bypass_ptr->code_bw;
         module_ptr->data_bw           = module_ptr->bypass_ptr->data_bw;
         module_ptr->hw_acc_proc_delay = module_ptr->bypass_ptr->hw_acc_proc_delay;

         // If thresholds are nonzero, set on ports and set event flag.
         if (0 != module_ptr->bypass_ptr->in_thresh_bytes_all_ch)
         {
            // Ignore redundant threshold events.
            if (input_port_ptr->common.threshold_raised != module_ptr->bypass_ptr->in_thresh_bytes_all_ch)
            {
               input_port_ptr->common.threshold_raised = module_ptr->bypass_ptr->in_thresh_bytes_all_ch;

               // Reset threshold info
               input_port_ptr->common.flags.port_has_threshold =
                  (input_port_ptr->common.threshold_raised) ? TRUE : FALSE;
               input_port_ptr->common.port_event_new_threshold = input_port_ptr->common.threshold_raised;

               capi_event_flags.port_thresh = TRUE;
            }
         }

         if (0 != module_ptr->bypass_ptr->out_thresh_bytes_all_ch)
         {
            // Ignore redundant threshold events.
            if (output_port_ptr->common.threshold_raised != module_ptr->bypass_ptr->out_thresh_bytes_all_ch)
            {
               output_port_ptr->common.threshold_raised = module_ptr->bypass_ptr->out_thresh_bytes_all_ch;

               // Reset threshold info
               output_port_ptr->common.flags.port_has_threshold =
                  (output_port_ptr->common.threshold_raised) ? TRUE : FALSE;
               output_port_ptr->common.port_event_new_threshold = output_port_ptr->common.threshold_raised;

               capi_event_flags.port_thresh = TRUE;
            }
         }
      }

      capi_event_flags.kpps                    = kpps_changed;
      capi_event_flags.bw                      = bw_changed;
      capi_event_flags.hw_acc_proc_delay_event = hw_acc_proc_delay_changed;
   }

   // buffers must be reassigned since a module changes its from bypass <-> to non-bypass
   capi_event_flags.dynamic_inplace_change = TRUE;

   GEN_TOPO_SET_CAPI_EVENT_FLAGS(topo_ptr, capi_event_flags);

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->gu.log_id)
   {
   }

   return result;
}

ar_result_t gen_topo_check_create_bypass_module(gen_topo_t *topo_ptr, gen_topo_module_t *module_ptr)
{
   INIT_EXCEPTION_HANDLING
   ar_result_t result = AR_EOK;

   if (module_ptr->bypass_ptr)
   {
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_LOW_PRIO,
               "Module 0x%lX (0x%lX) is already in bypass state",
               module_ptr->gu.module_instance_id,
               module_ptr->gu.module_id);
      return result;
   }

   if (!gen_topo_is_module_bypassable(module_ptr))
   {
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_LOW_PRIO,
               "Module 0x%lX (0x%lX) is not bypassable",
               module_ptr->gu.module_instance_id,
               module_ptr->gu.module_id);
      return AR_EFAILED;
   }

   TOPO_MSG(topo_ptr->gu.log_id,
            DBG_LOW_PRIO,
            SPF_LOG_PREFIX "Module 0x%lX (0x%lX) is begin bypass",
            module_ptr->gu.module_instance_id,
            module_ptr->gu.module_id);

   MALLOC_MEMSET(module_ptr->bypass_ptr,
                 gen_topo_module_bypass_t,
                 sizeof(gen_topo_module_bypass_t),
                 topo_ptr->heap_id,
                 result);

   // set default media format.
   tu_set_media_fmt(&topo_ptr->mf_utils, &module_ptr->bypass_ptr->media_fmt_ptr, NULL, topo_ptr->heap_id);

   if (AR_DID_FAIL(result = gen_topo_switch_bypass_module(topo_ptr, module_ptr, TRUE /*bypass_enable*/)))
   {
      TOPO_MSG(topo_ptr->gu.log_id,
               DBG_ERROR_PRIO,
               "Module 0x%lX (0x%lX) failed to switch bypass",
               module_ptr->gu.module_instance_id,
               module_ptr->gu.module_id);
      MFREE_NULLIFY(module_ptr->bypass_ptr);
   }

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->gu.log_id)
   {
   }

   return result;
}

ar_result_t gen_topo_check_destroy_bypass_module(gen_topo_t        *topo_ptr,
                                                 gen_topo_module_t *module_ptr,
                                                 bool_t             is_module_destroying)
{
   INIT_EXCEPTION_HANDLING
   SPF_MANAGE_CRITICAL_SECTION
   ar_result_t result = AR_EOK;

   if (!module_ptr->bypass_ptr)
   {
      return result;
   }

   // if module itself is destroying as part of close then no point in enabling the bypass module.
   if (!is_module_destroying)
   {
      TRY(result, gen_topo_switch_bypass_module(topo_ptr, module_ptr, FALSE /*bypass_enable*/));
   }

   SPF_CRITICAL_SECTION_START(&topo_ptr->gu);

   //release the ref to the media format block
   tu_release_media_fmt(&topo_ptr->mf_utils, &module_ptr->bypass_ptr->media_fmt_ptr);

   SPF_CRITICAL_SECTION_END(&topo_ptr->gu);

   MFREE_NULLIFY(module_ptr->bypass_ptr);

   TOPO_MSG(topo_ptr->gu.log_id,
            DBG_LOW_PRIO,
            SPF_LOG_PREFIX "Module 0x%lX (0x%lX) end bypass",
            module_ptr->gu.module_instance_id,
            module_ptr->gu.module_id);

   CATCH(result, TOPO_MSG_PREFIX, topo_ptr->gu.log_id)
   {
   }

   return result;
}
