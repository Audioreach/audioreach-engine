#ifndef _TRM_API_H_
#define _TRM_API_H_
/**
 * \file trm_api.h
 * \brief 
 *  	 API file for Timed Renderer Module
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_graph_properties.h"

/*==============================================================================
   Constants
==============================================================================*/

/* Input port ID of TRM */
#define TRM_DATA_INPUT_PORT   0x2

/* Output port ID of TRM */
#define TRM_DATA_OUTPUT_PORT  0x1

/* Max number of input ports of TRM */
#define TRM_DATA_MAX_INPUT_PORTS 0x1

/* Max number of output ports of TRM */
#define TRM_DATA_MAX_OUTPUT_PORTS 0x1

/* Stack size of TRM */
#define TRM_STACK_SIZE 1024


/*==============================================================================
   Param ID
==============================================================================*/

/*==============================================================================
   Module
==============================================================================*/

/** @h2xml_title1           {TRM API}
    @h2xml_title_agile_rev  {TRM API}
    @h2xml_title_date       {July 7, 2019} */

/**
 * Module ID for Timed Renderer Module
 */
#define MODULE_ID_TRM								0x07001087

/**
    @h2xmlm_module              {"Timed Renderer", MODULE_ID_TRM}
    @h2xmlm_displayName         {"Timed Renderer"}
    @h2xmlm_modSearchKeys		{Voice}
    @h2xmlm_description         {
                                 This module holds or releases buffers, based on the time
                                 to render. This module's main purpose is for time stamp
                                 based voice packet rendering on downlink path\n
                                }
    @h2xmlm_dataMaxInputPorts   {TRM_DATA_MAX_INPUT_PORTS}
    @h2xmlm_dataMaxOutputPorts  {TRM_DATA_MAX_OUTPUT_PORTS}
    @h2xmlm_dataInputPorts      {IN=TRM_DATA_INPUT_PORT}
    @h2xmlm_dataOutputPorts     {OUT=TRM_DATA_OUTPUT_PORT}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable       {false}
    @h2xmlm_stackSize           {TRM_STACK_SIZE}
    @h2xmlm_toolPolicy          {Calibration}
    @{                          <-- Start of the Module -->
    @}                          <-- End of the Module -->
*/

#endif //_TRM_API_H_
