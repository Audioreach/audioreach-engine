/**
 *   \file wr_sh_mem_ep_ext_api.h
 *   \brief
 *        This file contains Shared mem module API extensions for
 *           Write shared memory Endpoint module
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef WR_SH_MEM_EP_API_EXT_H_
#define WR_SH_MEM_EP_API_EXT_H_

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "ar_defs.h"
#include "sh_mem_ep_metadata_api.h"
#include "ar_guids.h"
#include "wr_sh_mem_ep_api.h"

/**
  @h2xml_title1          {Write Shared Memory End Point Module API Extension}
  @h2xml_title_agile_rev {Write Shared Memory End Point Module API Extension}
  @h2xml_title_date      {July 18, 2019}
*/

/**
   @h2xmlx_xmlNumberFormat {int}
*/

/*==============================================================================
   Param ID
==============================================================================*/
#define CLIENT_ID_HLOS 0
/* definition for HLOS as client*/

#define CLIENT_ID_OLC 1
/* definition for OLC as client*/

/**
   Configuration of the Write Shared Memory Module.
*/
#define PARAM_ID_WR_EP_CLIENT_CFG 0x080010C0
/*==============================================================================
   Param structure definitions
==============================================================================*/

/** @h2xmlp_parameter   {"PARAM_ID_WR_EP_CLIENT_CFG", PARAM_ID_WR_EP_CLIENT_CFG}
    @h2xmlp_description {Parameter for setting the client configuration to the read shared memory end point module.\n }
    @h2xmlp_toolPolicy  {Calibration} */
#include "spf_begin_pack.h"
struct param_id_wr_ep_client_cfg_t
{
   uint32_t client_id;
   /**< @h2xmle_description {Specifies the information about the client
                       0 indicates external client (HLOS)
                       1 indicates internal client (OLC) }
        @h2xmle_default     {0}
        @h2xmle_rangelist   { 0 - HLOS,
                              1 - OLC }
        @h2xmle_policy      {advanced}
   */
   uint32_t gpr_port_id;
   /**< @h2xmle_description {Specifies the ID used for registration with GPR by the specified Client
                             For CLIENT_ID_OLC this would be the container ID of OLC}
        @h2xmle_default     {0 - indicates, value is not set by the client}
        @h2xmle_rangelist   {0 - 0x7FFFFFFF}
        @h2xmle_policy      {advanced}
   */
}
#include "spf_end_pack.h"
;

/* Type definition for the above structure. */
typedef struct param_id_wr_ep_client_cfg_t param_id_wr_ep_client_cfg_t;

#endif // WR_SH_MEM_EP_API_EXT_H_
