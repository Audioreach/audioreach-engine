/**
 * \file wr_sh_mem_client_api.h
 * \brief 
 *  	 This file contains Shared mem Client module APIs
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef WR_SH_MEM_CLEINT_API_H_
#define WR_SH_MEM_CLEINT_API_H_

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "ar_defs.h"
#include "apm_graph_properties.h"
/** 
  @h2xml_title1          {Write Shared Memory Client Module API}
  @h2xml_title_agile_rev {Write Shared Memory Client Module API}
  @h2xml_title_date      {May 23, 2019}
*/

/** @ingroup ar_spf_mod_ws_mem_client_macros
    Output port ID of the Write Shared Memory Client module */
#define PORT_ID_WR_SHARED_MEM_CLIENT_OUTPUT               0x1

/** @ingroup ar_spf_mod_ws_mem_client_macros
    Input port ID of the Write Shared Memory Client module. */
#define PORT_ID_WR_SHARED_MEM_CLIENT_INPUT                0x2

/** @ingroup ar_spf_mod_ws_mem_client_macros
   ID of the Write Shared Memory Client module.

   This module has one static input port with ID 2 and output port with ID 1.

   This module is supported only in Offload container. The modules functions as the
   Client to the  MODULE_ID_WR_SHARED_MEM_EP module. The module send the data from the
   Master process domain to the Write End-point module in the satellite SPF.

    @subhead4{Supported input media format ID}
    - Any
 */
#define MODULE_ID_WR_SHARED_MEM_CLIENT                    0x0700105C
/**
    @h2xmlm_module         {"MODULE_ID_WR_SHARED_MEM_CLIENT", MODULE_ID_WR_SHARED_MEM_CLIENT}
    @h2xmlm_displayName    {"Write Shared Memory Client"}
    @h2xmlm_modSearchKeys  {software}
    @h2xmlm_description    {
                            This module is used to send data from spf in Master process domain to
                            the spf in satellite process domain using the GPR packet exchange mechanism.
                            This module has one static input port with ID 0 and output port with ID 1
                            }
    @h2xmlm_offloadInsert        { WR_CLIENT }
    @h2xmlm_dataInputPorts       { IN = PORT_ID_WR_SHARED_MEM_CLIENT_INPUT}
    @h2xmlm_dataMaxInputPorts    { 1 }
    @h2xmlm_dataOutputPorts      { OUT = PORT_ID_WR_SHARED_MEM_CLIENT_OUTPUT}
    @h2xmlm_dataMaxOutputPorts   { 1 }
    @h2xmlm_supportedContTypes  { APM_CONTAINER_TYPE_OLC }
    @h2xmlm_isOffloadable        {false}
    @h2xmlm_stackSize            { 4096 }
    @{                     <-- Start of the Module -->
    @}                     <-- End of the Module -->
*/

#endif // WR_SH_MEM_CLIENT_API_H_
