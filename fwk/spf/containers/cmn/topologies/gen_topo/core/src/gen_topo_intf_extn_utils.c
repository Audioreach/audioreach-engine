/**
 * \file gen_topo_intf_extn_utils.c
 * \brief
 *     This file contains intf extn utils.
 *  
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_topo.h"
#include "gen_topo_capi.h"

/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/

/* -----------------------------------------------------------------------
 ** Constant / Define Declarations
 ** ----------------------------------------------------------------------- */

/* -----------------------------------------------------------------------
 ** Function prototypes
 ** ----------------------------------------------------------------------- */

ar_result_t gen_topo_intf_extn_handle_at_init(gen_topo_t *topo_ptr, gen_topo_module_t *module_ptr)
{
   ar_result_t result = AR_EOK;
   if (TRUE == module_ptr->flags.supports_metadata)
   {
      intf_extn_param_id_metadata_handler_t handler;
      gen_topo_populate_metadata_extn_vtable(module_ptr, &handler);
      result |= gen_topo_capi_set_param(topo_ptr->gu.log_id,
                                        module_ptr->capi_ptr,
                                        INTF_EXTN_PARAM_ID_METADATA_HANDLER,
                                        (int8_t *)&handler,
                                        sizeof(handler));
   }

   // Perform data ports open operation on modules, if it supports the extension.
   if (TRUE == module_ptr->flags.supports_data_port_ops)
   {
      result |= gen_topo_intf_extn_data_ports_hdl_at_init(topo_ptr, module_ptr);
   }

   return result;
}
