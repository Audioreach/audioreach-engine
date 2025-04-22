/**
 * \file rd_sh_mem_client_api.h
 * \brief 
 *  	 This file contains Shared mem Client module APIs
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef RD_SH_MEM_CLIENT_API_H_
#define RD_SH_MEM_CLIENT_API_H_

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "ar_defs.h"
#include "apm_graph_properties.h"

/** 
    @h2xml_title1          {APIs of Read Shared Memory Client Module}
    @h2xml_title_agile_rev {APIs of Read Shared Memory Client Module}
    @h2xml_title_date      {May 23, 2019}
 */

/**
   @h2xmlx_xmlNumberFormat {int}
*/


/** @ingroup ar_spf_mod_rs_mem_client_macros
 *  Input port ID of the Read Shared Memory Client. */
#define PORT_ID_RD_SHARED_MEM_CLIENT_INPUT                            0x2

/** @ingroup ar_spf_mod_rs_mem_client_macros
    Output port ID of the Read Shared Memory Client. */
#define PORT_ID_RD_SHARED_MEM_CLIENT_OUTPUT                            0x1

/** @ingroup ar_spf_mod_rs_mem_client_macros
    ID of the Read Shared Memory Client module.

    This module has only one static input port with ID 2 and one output port with ID 1.

    @subhead4{Supported input media format ID}
    - Any
 */
#define MODULE_ID_RD_SHARED_MEM_CLIENT                                0x0700105D
/**
    @h2xmlm_module         {"MODULE_ID_RD_SHARED_MEM_CLIENT", MODULE_ID_RD_SHARED_MEM_CLIENT}
    @h2xmlm_displayName    {"Read Shared Memory Client"}
    @h2xmlm_modSearchKeys  {software}
    @h2xmlm_description    {
                            This module is used by the SPF in master process domain to read data from spf
                            in Satellite process domain through packet exchange mechanism.
                            This module has only one static input port with ID 2 and one output port with ID 1.
                            }
    @h2xmlm_offloadInsert       { RD_CLIENT }
    @h2xmlm_dataInputPorts      { IN = PORT_ID_RD_SHARED_MEM_CLIENT_INPUT}
    @h2xmlm_dataMaxInputPorts   {1}
    @h2xmlm_dataOutputPorts     { OUT = PORT_ID_RD_SHARED_MEM_CLIENT_OUTPUT}
    @h2xmlm_dataMaxOutputPorts  {1}
    @h2xmlm_supportedContTypes { APM_CONTAINER_TYPE_OLC }
    @h2xmlm_isOffloadable       {false}
    @h2xmlm_stackSize           { 1024 }
    @{                     <-- Start of the Module -->
    @}                     <-- End of the Module -->
*/


#endif // RD_SH_MEM_CLIENT_API_H_
