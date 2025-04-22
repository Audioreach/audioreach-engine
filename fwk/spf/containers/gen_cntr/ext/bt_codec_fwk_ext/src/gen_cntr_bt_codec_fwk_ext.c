/**
 * \file gen_cntr_bt_codec_fwk_ext.c
 * \brief
 *     This file contains functions that do optional error checking.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr_i.h"
#include "gen_cntr_utils.h"
#include "apm.h"

ar_result_t gen_cntr_handle_bt_codec_ext_event(gen_topo_module_t *module_ptr, capi_event_info_t *event_info_ptr)
{
   gen_cntr_t *                      me_ptr        = (gen_cntr_t *)GET_BASE_PTR(gen_cntr_t, topo, module_ptr->topo_ptr);
   ar_result_t                       result        = AR_EOK;
   capi_buf_t *                      payload       = &event_info_ptr->payload;
   capi_event_data_to_dsp_service_t *dsp_event_ptr = (capi_event_data_to_dsp_service_t *)(payload->data_ptr);

   switch (dsp_event_ptr->param_id)
   {
      case CAPI_BT_CODEC_EXTN_EVENT_ID_DISABLE_PREBUFFER:
      {
         if (dsp_event_ptr->payload.actual_data_len < sizeof(capi_bt_codec_extn_event_disable_prebuffer_t))
         {
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_ERROR_PRIO,
                         "Module 0x%lX: Error in callback function. The actual size %lu is less than the required size "
                         "%lu for id %lu.",
                         module_ptr->gu.module_instance_id,
                         payload->actual_data_len,
                         sizeof(capi_bt_codec_extn_event_disable_prebuffer_t),
                         dsp_event_ptr->param_id);
            return AR_ENEEDMORE;
         }

         capi_bt_codec_extn_event_disable_prebuffer_t *disable_prebuffer_ptr =
            (capi_bt_codec_extn_event_disable_prebuffer_t *)dsp_event_ptr->payload.data_ptr;

         for (gu_ext_out_port_list_t *ext_out_port_list_ptr = me_ptr->topo.gu.ext_out_port_list_ptr;
              (NULL != ext_out_port_list_ptr);
              LIST_ADVANCE(ext_out_port_list_ptr))
         {
            gen_cntr_ext_out_port_t *ext_out_port_ptr =
               (gen_cntr_ext_out_port_t *)ext_out_port_list_ptr->ext_out_port_ptr;
            ext_out_port_ptr->cu.icb_info.disable_one_time_pre_buf =
               (disable_prebuffer_ptr->disable_prebuffering > 0) ? TRUE : FALSE;

            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_HIGH_PRIO,
                         "Module 0x%lX: bt_codec_extn: Pre bufffering disabled flag is set to %X",
                         module_ptr->gu.module_instance_id,
                         ext_out_port_ptr->cu.icb_info.disable_one_time_pre_buf);
         }
         break;
      }
      case CAPI_BT_CODEC_EXTN_EVENT_ID_KPPS_SCALE_FACTOR:
      {
         if (dsp_event_ptr->payload.actual_data_len < sizeof(capi_bt_codec_etxn_event_kpps_scale_factor_t))
         {
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_ERROR_PRIO,
                         "Module 0x%lX: Error in callback function. The actual size %lu is less than the required size "
                         "%lu for "
                         "id %lu.",
                         module_ptr->gu.module_instance_id,
                         payload->actual_data_len,
                         sizeof(capi_bt_codec_etxn_event_kpps_scale_factor_t),
                         dsp_event_ptr->param_id);
            return AR_ENEEDMORE;
         }

         /*
          * Multiply the scale factor to current kpps value and vote with new kpps.
          * This event is raised when the the module needs to process data faster to catch up
          * the real time
          *
          * Get the scale factor from the event payload and calculate new
          * kpps as follow,
          *      new _kpps  = old_kpps * scale_factor
          *
          */
         capi_bt_codec_etxn_event_kpps_scale_factor_t *event_ptr =
            (capi_bt_codec_etxn_event_kpps_scale_factor_t *)dsp_event_ptr->payload.data_ptr;

         // 1 in q4 format is 0x10
         if (event_ptr->scale_factor < 0x10)
         {
            GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                         DBG_ERROR_PRIO,
                         "Module 0x%lX: : bt_codec_extn: Received invalid kpps scale factor lesser than 1 = %lu ",
                         module_ptr->gu.module_instance_id,
                         event_ptr->scale_factor);
            break;
         }

         /*Store the modules KPPS scale factor, floor clock is calculated based on modules KPPS and scale factor*/
         module_ptr->kpps_scale_factor_q4 = event_ptr->scale_factor;

         /* Set this flag to vote */
         GEN_TOPO_SET_ONE_CAPI_EVENT_FLAG(module_ptr->topo_ptr, kpps);

         GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_HIGH_PRIO,
                      "Module 0x%lX: bt_codec_extn: Received KPPS scale factor = %lu ",
                      module_ptr->gu.module_instance_id,
                      event_ptr->scale_factor);
         break;
      }
      default:
      {
         GEN_CNTR_MSG(me_ptr->topo.gu.log_id,
                      DBG_HIGH_PRIO,
                      "Module 0x%lX: unknown event 0x%lx",
                      module_ptr->gu.module_instance_id,
                      dsp_event_ptr->param_id);
         return AR_EFAILED;
      }
   }

   return result;
}
