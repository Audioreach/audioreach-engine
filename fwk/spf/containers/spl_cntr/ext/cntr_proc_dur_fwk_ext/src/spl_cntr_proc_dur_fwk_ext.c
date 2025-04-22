/**
 * \file spl_cntr_proc_dur_fwk_ext.c
 *
 * \brief
 *     Implementation of container proc_dur fwk extn in spl container
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear */

/* =======================================================================
Includes
========================================================================== */
#include "spl_cntr_i.h"

/* =======================================================================
Function definitions
========================================================================== */
ar_result_t spl_cntr_fwk_extn_set_cntr_proc_duration_per_module(spl_cntr_t *       me_ptr,
                                                                spl_topo_module_t *module_ptr,
                                                                uint32_t           cont_proc_dur_us)
{
   // set param payload
   struct
   {
      apm_module_param_data_t                     module_data;
      fwk_extn_param_id_container_proc_duration_t payload;
   } set_param_payload;

   set_param_payload.payload.proc_duration_us = cont_proc_dur_us;

   set_param_payload.module_data.module_instance_id = module_ptr->t_base.gu.module_instance_id;
   set_param_payload.module_data.param_id           = FWK_EXTN_PARAM_ID_CONTAINER_PROC_DURATION;
   set_param_payload.module_data.param_size =
      (sizeof(fwk_extn_param_id_container_proc_duration_t) + sizeof(apm_module_param_data_t));
   set_param_payload.module_data.error_code = 0;

   spl_topo_set_param(&me_ptr->topo, &set_param_payload.module_data);

   return AR_EOK;
}

/* Iterates over all modules in the sg list and tries to set proc delay if it supports this extension*/
ar_result_t spl_cntr_fwk_extn_set_cntr_proc_duration(spl_cntr_t *me_ptr, uint32_t cont_proc_dur_us)
{
   for (gu_sg_list_t *sg_list_ptr = me_ptr->topo.t_base.gu.sg_list_ptr; (NULL != sg_list_ptr);
        LIST_ADVANCE(sg_list_ptr))
   {
      for (gu_module_list_t *module_list_ptr = sg_list_ptr->sg_ptr->module_list_ptr; (NULL != module_list_ptr);
           LIST_ADVANCE(module_list_ptr))
      {
         spl_topo_module_t *module_ptr = (spl_topo_module_t *)module_list_ptr->module_ptr;

         // if proc delay fwk extension is not supported return
         if (FALSE == module_ptr->t_base.flags.need_proc_dur_extn)
         {
            continue;
         }

         spl_cntr_fwk_extn_set_cntr_proc_duration_per_module(me_ptr, module_ptr, cont_proc_dur_us);
      }
   }
   return AR_EOK;
}
