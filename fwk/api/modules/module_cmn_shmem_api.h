#ifndef MODULE_CMN_SH_MEM_API_H_
#define MODULE_CMN_SH_MEM_API_H_
/**
 * \file module_cmn_sh_mem_api.h
 * \brief
 *    This file contains common api's for the Read, Write Shared mem module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*------------------------------------------------------------------------
  Include files
  -----------------------------------------------------------------------*/
#include "ar_defs.h"
#include "common_enc_dec_api.h"

/*# @h2xml_title1          {Common APIs of Read/Write Shared Memory Endpoint Module}
    @h2xml_title_agile_rev {Common APIs of Read/Write Shared Memory Endpoint Module}
    @h2xml_title_date      {June 6, 2023} */

/*# @h2xmlx_xmlNumberFormat {int} */

/** Configuration Value to configure the client state property type as INVALID*/
#define PROPERTY_TYPE_IS_INVALID       0

/** Configuration Value to configure the client state property type as Upstream Realtime*/
#define PROPERTY_TYPE_IS_UPSTREAM_REALTIME   1

/** Configuration Value to configure the client state property type as downstream Realtime*/
#define PROPERTY_TYPE_IS_DOWNSTREAM_REALTIME   2

/** Defines the client state configuration value as faster than real time (FTRT)
 *  This configuration value is only applicable for following property type
 *  PROPERTY_TYPE_IS_UPSTREAM_REALTIME
 *  PROPERTY_TYPE_IS_DOWNSTREAM_REALTIME */
#define PROPERTY_VALUE_IS_FTRT   0

/** Defines the client state configuration Value as Real time (RT)
 *  This configuration value is only applicable for following property type
 *  PROPERTY_TYPE_IS_UPSTREAM_REALTIME
 *  PROPERTY_TYPE_IS_DOWNSTREAM_REALTIME */
#define PROPERTY_VALUE_IS_RT   1


/**
   Payload definition of the shared memory client property configuration
*/
/** @h2xmlp_subStruct */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct sh_mem_peer_client_property_payload_t
{
   uint32_t property_type;
   /**< @h2xmle_description {specifies the property type}
        @h2xmle_default     {PROPERTY_TYPE_IS_INVALID}
        @h2xmle_rangeList   {"PROPERTY_TYPE_IS_INVALID" = PROPERTY_TYPE_IS_INVALID,
                             "PROPERTY_TYPE_IS_UPSTREAM_REALTIME" = PROPERTY_TYPE_IS_UPSTREAM_REALTIME,
                             "PROPERTY_TYPE_IS_DOWNSTREAM_REALTIME" = PROPERTY_TYPE_IS_DOWNSTREAM_REALTIME}
        @h2xmle_policy      {advanced} */

   uint32_t property_value;
   /**< @h2xmle_description {specifies the property value. The Property values are dependent on property type.
                             The following details should help to configure appropriately
                             Property type                                       Support Values
                             PROPERTY_TYPE_IS_UPSTREAM_REALTIME                   IS_FTRT, IS_RT
                             PROPERTY_TYPE_IS_DOWNSTREAM_REALTIME                 IS_FTRT, IS_RT
                             }
        @h2xmle_default     {PROPERTY_VALUE_IS_FTRT}
        @h2xmle_rangeList   {"IS_FTRT" = PROPERTY_VALUE_IS_FTRT,
                             "IS_RT" = PROPERTY_VALUE_IS_RT}
        @h2xmle_policy      {advanced} */

}
#include "spf_begin_pragma.h"
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct sh_mem_peer_client_property_payload_t sh_mem_peer_client_property_payload_t;

/**
   Configuration parameter ID for setting the up/downstream (basically client) state configuration information
   to the WR/RD_EP module in the SPF graph.
*/
#define PARAM_ID_SH_MEM_PEER_CLIENT_PROPERTY_CONFIG 0x08001A6F
/*==============================================================================
   Param structure definitions
==============================================================================*/

/** @h2xmlp_parameter   {"PARAM_ID_SH_MEM_PEER_CLIENT_PROPERTY_CONFIG", PARAM_ID_SH_MEM_PEER_CLIENT_PROPERTY_CONFIG}
    @h2xmlp_description {Parameter for setting the up/downstream (basically client) state configuration information
                        to the WR/RD_EP module in the SPF graph.\n
                        This would help the SPF Clients to set the relevant attributes to the SPF SH_MEM modules
                        and adjust the data processing characteristics in the SPF. \n
                        For example, if the Client for RD_SH_MEM_EP module operates in real-time,
                        configuring this payload with appropriate attributes would help to increase
                        the processing priority of this use case. \n
                        }
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_sh_mem_peer_client_property_config_t
{
   uint32_t num_properties;
   /**< @h2xmle_description {determines the number of client state properties
                             following this payload.}
        @h2xmle_range       {0..0x7FFFFFFF}
        @h2xmle_default     {0}
        @h2xmle_policy      {advanced} */

#if defined (__H2XML__)
   sh_mem_peer_client_property_payload_t client_property_payload[0];
   /**< @h2xmle_description {Specifies the client state property payload }
        @h2xmle_policy      {advanced}
   */
#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
/* Type definition for the above structure. */
typedef struct param_id_sh_mem_peer_client_property_config_t param_id_sh_mem_peer_client_property_config_t;


#endif // MODULE_CMN_SH_MEM_API_H_
