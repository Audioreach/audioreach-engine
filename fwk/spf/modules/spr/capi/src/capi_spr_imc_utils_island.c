/*==========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/

/**
 *   \file capi_spr_imc_utils.c
 *   \brief
 *        This file contains CAPI V2 IMC utils implementation of SPR module
 */

#include "capi_spr_i.h"

/*==============================================================================
   Local Defines
==============================================================================*/

/*==============================================================================
   Local Function forward declaration
==============================================================================*/
spr_ctrl_port_t *spr_get_ctrl_port_instance(capi_spr_t *me_ptr, uint32_t ctrl_port_id)
{
   spr_ctrl_port_t *ret_inst_ptr = NULL;

   spr_ctrl_port_list_t *list_ptr = me_ptr->ctrl_port_list_ptr;

   while (list_ptr)
   {
      spr_ctrl_port_t *node_ptr = list_ptr->port_info_ptr;

      if (node_ptr->port_id == ctrl_port_id)
      {
         ret_inst_ptr = node_ptr;
         break;
      }

      list_ptr = list_ptr->next_ptr;
   }

   return ret_inst_ptr;
}

void capi_spr_imcl_get_drift(capi_spr_t *me_ptr, spr_ctrl_port_t *port_info_ptr)
{
   imcl_tdi_hdl_t *timer_drift_hdl_ptr = port_info_ptr->timer_drift_info_hdl_ptr;

   if ((NULL != timer_drift_hdl_ptr) && (NULL != timer_drift_hdl_ptr->get_drift_fn_ptr))
   {
#ifdef DEBUG_SPR_MODULE
      SPR_MSG_ISLAND(me_ptr->miid, DBG_HIGH_PRIO, "reading from handle %lx", timer_drift_hdl_ptr);
#endif
      timer_drift_hdl_ptr->get_drift_fn_ptr(timer_drift_hdl_ptr, &port_info_ptr->acc_drift);
   }
}
