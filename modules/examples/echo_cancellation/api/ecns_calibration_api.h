#ifndef __ECNS_API_H__
#define __ECNS_API_H__

/**
 * \file ecns_calibration_api.h
 *
 * \brief
 *
 *     Example Echo Cancellation
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "module_cmn_api.h"
/**
  @h2xml_title1          {Module example ecns API}
  @h2xml_title_agile_rev {Module example ecns API}
  @h2xml_title_date      {March 6, 2019}
*/

/*==============================================================================
   Constants
==============================================================================*/

/* ECNS supports two inputs Primary input and Reference input ports.*/
#define ECNS_MAX_INPUT_PORTS                   0x2

/* This example module has only once output port.

   Can be exteneded to multiple ports as well. It may require some changes in
   data port operations, metadata handling and process.
*/
#define ECNS_MAX_OUTPUT_PORTS                  0x1

/* Stack size requirement is a static module property. It is the upper bound for peak estimated stack usage. */
#define ECNS_STACK_SIZE_REQUIREMENT          4096

/*==============================================================================
   Static Port Ids -
   Input port Ids must be always even and output ports Ids must be odd.

   Framework assigns a dynamic port index for each port ID and informs module
   through data port operation OPEN.

   After OPEN operation, framework uses port index for further port sepecific
   communication with the module. Module must use port id to port index mapping to
   get the port context.
==============================================================================*/
/* Primary input port static ID. */
#define ECNS_PRIMARY_INPUT_STATIC_PORT_ID 		0x2

/* Reference input static port ID */
#define ECNS_REFERENCE_INPUT_STATIC_PORT_ID     0x4

/* Primary output port static ID. */
#define ECNS_PRIMARY_OUTPUT_STATIC_PORT_ID		0x1

/*==============================================================================
   Param ID
==============================================================================*/

/* This is an optional example parameter defined to demonstrate how to expose
   calibration API.

   Each param ID and module definition must follow the H2XML annotations for
   defining the payload structures and module properties.

   Refer H2XML documentation for further information.
   */
#define PARAM_ID_ECNS_DUMMY_CFG 								0x080011E0

/*==============================================================================
   Param structure defintions
==============================================================================*/
/** @h2xmlp_parameter    {"PARAM_ID_ECNS_DUMMY_CFG", PARAM_ID_ECNS_DUMMY_CFG}
    @h2xmlp_description  { Structure for the optional param. }
    @h2xmlp_toolPolicy   {Calibration, RTC } */

#include "spf_begin_pack.h"
struct param_id_ecns_dummy_cfg_t
{
   int32_t feature_mask;
   /**< @h2xmle_description   {features bitmask for ECNS algorithm}
        @h2xmle_range         {0x80000000..0x7FFFFFFF}
        @h2xmle_policy        {Basic}

        @h2xmle_bitfield		{0x00000001}
        @h2xmle_bitName			{"Echo cancellation"}
        @h2xmle_description	{Enable/disable echo cancellation}
        @h2xmle_rangeList     {"Disable"=0; "Enable"=1}
        @h2xmle_default       {0}
        @h2xmle_bitfieldEnd

        @h2xmle_bitfield		{0x00000002}
        @h2xmle_bitName			{"Noise supression"}
        @h2xmle_description   {Enable/disable noise supression}
        @h2xmle_rangeList     {"Disable"=0; "Enable"=1}
        @h2xmle_default       {0}
        @h2xmle_bitfieldEnd
*/
}
#include "spf_end_pack.h"
;
/* Structure type def for above payload. */
typedef struct param_id_ecns_dummy_cfg_t param_id_ecns_dummy_cfg_t;


/*------------------------------------------------------------------------------
   Module ID
------------------------------------------------------------------------------*/

/* ECNS module ID */
#define MODULE_ID_ECNS										 0x0700109A

/**
    @h2xmlm_module      {"MODULE_ID_ECNS", MODULE_ID_ECNS}
    @h2xmlm_displayName {"Example - ECNS"}
    @h2xmlm_description { ECNS example module performs echo cancellation and noise suppression.
                         This module supports the following parameter IDs, \n
                        -  PARAM_ID_ECNS_DUMMY_CFG
                        -  PARAM_ID_MODULE_ENABLE \n
                         \n}

    @h2xmlm_dataMaxInputPorts    {ECNS_MAX_INPUT_PORTS}
    @h2xmlm_dataInputPorts       {MIC_INPUT = ECNS_PRIMARY_INPUT_STATIC_PORT_ID;
                                  REF_INPUT = ECNS_REFERENCE_INPUT_STATIC_PORT_ID}
    @h2xmlm_dataMaxOutputPorts   {ECNS_MAX_OUTPUT_PORTS}
    @h2xmlm_dataOutputPorts      {ECNS_OUTPUT = ECNS_PRIMARY_OUTPUT_STATIC_PORT_ID}
    @h2xmlm_supportedContTypes  { APM_CONTAINER_TYPE_SC}
    @h2xmlm_stackSize            { ECNS_STACK_SIZE_REQUIREMENT }

    @h2xmlm_ToolPolicy           {Calibration}
    @{                   <-- Start of the Module -->

    @h2xml_Select               { param_id_module_enable_t }
    @h2xmlp_description         { This is a common param, shared by other modules.
                                  It is used to enable/disable the module runtime.
                                  This is an optional param, if the module doesn't
                                  need this param it can be removed. }
    @h2xmlm_InsertParameter
    @h2xmlp_toolPolicy          { Calibration, RTC }
    @h2xml_Select               {param_id_module_enable_t::enable}
    @h2xmle_default             {1}

    @h2xml_Select               {param_id_ecns_dummy_cfg_t}
    @h2xmlm_InsertParameter

   @}                   <-- End of the Module -->*/
#endif // #ifndef __ECNS_API_H__
