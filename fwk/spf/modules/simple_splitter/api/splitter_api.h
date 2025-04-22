#ifndef SPLITTER_API_H
#define SPLITTER_API_H
/**
 * \file splitter_api.h
 * \brief
 *    This file contains the Module Id, Param IDs and configuration
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "module_cmn_api.h"
#include "metadata_api.h"

/*# @h2xml_title1          {Simple Splitter API}
    @h2xml_title_agile_rev {Simple Splitter API}
    @h2xml_title_date      {July 9, 2018} */

/*==============================================================================
   Constants
==============================================================================*/

/** @ingroup ar_spf_mod_splitter_mod
    Stack size of the Simple Splitter module. */
#define CAPI_SPLITTER_STACK_SIZE 512

/** @ingroup ar_spf_mod_splitter_mod
    Maximum number of input ports for the Simple Splitter module. @newpage */
#define SPLITTER_MAX_INPUT_PORTS    1


/** @ingroup ar_spf_mod_splitter_mod
    Identifier of the Simple Splitter module, which is used to split the data
    on the input port to all the output ports.

    @subhead4{Supported input media format ID}
     - Data format       : #DATA_FORMAT_FIXED_POINT @lstsp1
     - fmt_id            : Don't care @lstsp1
     - Sample rates      : Any (>0) @lstsp1
     - Number of channels: 1..128 (for certain products this module supports only 32 channels) @lstsp1
     - Channel type      : 1..128 @lstsp1
     - Bits per sample   : 16, 24, 32 @lstsp1
     - Q format          : 15, 27, 31 @lstsp1
     - Interleaving      : Any @lstsp1
     - Signed/unsigned   : Any
 */
#define MODULE_ID_SPLITTER 0x07001011

/*# @h2xmlm_module             {"MODULE_ID_SPLITTER", MODULE_ID_SPLITTER}
    @h2xmlm_displayName        {"Splitter"}
    @h2xmlm_modSearchKeys	   {Audio, Voice}
    @h2xmlm_description        {ID for the Simple Splitter module, which
                                splits the streams on the input to all the
                                output ports. For more details, see AudioReach
                                Signal Processing Framework (SPF) API Reference.}
    @h2xmlm_dataMaxInputPorts  {SPLITTER_MAX_INPUT_PORTS}
    @h2xmlm_dataMaxOutputPorts {INFINITE}
    @h2xmlm_dataInputPorts     {IN=2}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_SC, APM_CONTAINER_TYPE_GC,APM_CONTAINER_TYPE_PTC}
    @h2xmlm_isOffloadable      {true}
    @h2xmlm_stackSize          {CAPI_SPLITTER_STACK_SIZE}
    @h2xmlm_ToolPolicy         {Calibration}

    @{                      <-- Start of the Module --> */


/*# @h2xmlp_subStruct */

/** @ingroup ar_spf_mod_splitter_mod
    Sub-structure for the per_port_md_cfg_t parameter.
*/
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct per_port_md_cfg_t
{
   uint32_t port_id;
   /**< Unique identifier for the output port on which metadata propagation
        is to be configured. */

   /*#< @h2xmle_description {ID for the output port on which metadata
                             propagation is to be configured.} */

   uint32_t num_white_listed_md;
   /**< Number of metadata IDs to be propagated on the Splitter's output that
        maps to port_id. */

   /*#< @h2xmle_description {Number of metadata IDs to be propagated on the
                             Splitter's output that maps to port_id.} */

#if defined(__H2XML__)
   uint32_t md_whitelist[0];
   /**< Array of metadata IDs that correspond to the metadata to be propagated.
        The array size is of num_white_listed_md.

        By default, the Splitter module does not propagate any metadata. */

   /*#< @h2xmle_description       {Array of metadata IDs that correspond to the
                                   metadata to be propagated. By default, the
                                   Splitter module does not propagate any
                                   metadata.}
        @h2xmle_variableArraySize {num_white_listed_md} */
#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct  per_port_md_cfg_t per_port_md_cfg_t;

/** @ingroup ar_spf_mod_splitter_mod
    Identifier for the parameter that configures splitter's metadata propagation.

    - If this param is not issued to the module, the default behavior
      is to propagate the metadata on the output ports.
    - If this param is issued to the module, only the whitelisted
      metadata is propagated, all others are blocked.

    Example usage: this param can be used to control whether EOS metadata has to be propagated
    to EC reference path. If this param is not issued, then EOS is propagated. If this param is issued
    with zero metadata, then EOS is not propagated. If this param is issued with EOS, then EOS is propagated.

    @msgpayload
    param_id_splitter_metadata_prop_cfg_t @newpage
 */
#define PARAM_ID_SPLITTER_METADATA_PROP_CFG 0x0800103C

/*# @h2xmlp_parameter   {"PARAM_ID_SPLITTER_METADATA_PROP_CFG",
                          PARAM_ID_SPLITTER_METADATA_PROP_CFG}
    @h2xmlp_description {ID for the parameter that configures splitter's metadata
                         propagation.
                         If this param is
                         not issued to the module, the default behavior is to
                         propagate all the metadata to the output ports. If
                         this param is issued to the module, only the
                         whitelisted metadata is propagated. }
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_splitter_mod
    Payload of the #PARAM_ID_SPLITTER_METADATA_PROP_CFG parameter.
 */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_splitter_metadata_prop_cfg_t
{
   uint32_t num_ports;
   /**< Specifies the number of ports for which the metadata propagation is
        configured.

        @values 0 through 32767 (Default = 0) */

   /*#< @h2xmle_description {Specifies the number of ports for which the
                             metadata propagation is configured.}
        @h2xmle_default     {0}
        @h2xmle_range       {0..32767} */

#if defined(__H2XML__)
   per_port_md_cfg_t port_md_prop_cfg_arr[0];
   /**< Variable length array of port IDS on which EoS metadata propagation
        will be disabled. The size of the array depends on num_ports. */

   /*#< @h2xmle_description       {Variable length array of port IDS on which
                                   metadata propagation will be disabled.
                                   The size of the array depends on num_ports.}
        @h2xmle_variableArraySize {num_ports} */

#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_splitter_metadata_prop_cfg_t param_id_splitter_metadata_prop_cfg_t;

/*# @h2xmlp_subStruct */

/** @ingroup ar_spf_mod_splitter_mod
    Sub-structure for the per_port_ts_cfg_t parameter.
*/

/* Output port configuration to propagation timestamp from the input */
#define SPLITTER_OUT_PORT_DEFAULT_TS_PROPAGATION 0

/* Output port configuration to invalidate timestamp propagation */
#define SPLITTER_OUT_PORT_INVALID_TS_PROPAGATION 1

/* Output port configuration to propagate the Signal Triggered Module's timestamp.
 * This is useful in sink-signal-triggered container to propagate timestamp (in reference path) at which data is actually rendered to hw-sink. */
#define SPLITTER_OUT_PORT_STM_TS_PROPAGATION 2

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct per_port_ts_cfg_t
{
   uint32_t port_id;
   /*#< @h2xmle_description {ID for the output port on which timestamp propagation is to be configured.} */

   uint32_t ts_configuration;
   /*#< @h2xmle_description {configures timestamp propagation}
        @h2xmle_default     {#SPLITTER_OUT_PORT_DEFAULT_TS_PROPAGATION}
        @h2xmle_rangeList   { "Propagate TS from input" = SPLITTER_OUT_PORT_DEFAULT_TS_PROPAGATION;
                              "Invalidate timestamp" = SPLITTER_OUT_PORT_INVALID_TS_PROPAGATION;
                              "Propagate Signal-Triggered-Module's TS" = SPLITTER_OUT_PORT_STM_TS_PROPAGATION}*/
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct  per_port_ts_cfg_t per_port_ts_cfg_t;

/** @ingroup ar_spf_mod_splitter_mod
    Identifier for the parameter that configures splitter's timestamp propagation.

    - If this param is not issued to the module, the default behavior
      is to propagate the timestamp on the output ports from the input port.
    - If this param is issued to the module then timestamp is propagated
      based on the configuration.

    Configuration to send the “Signal Triggered module’s timestamp” should be used in the Splitter 
	which is connected to Signal-Triggered-End-Point module in upstream [Splitter -> STM].
	This configuration should not be used if Splitter is in downstream of Signal-Triggered-End-Point
	module [STM -> Splitter] because in this case input buffer timestamp is same as
	Signal-Triggered Module’s Timestamp. This configuration is also ignored and default behavior 
	to propagate the timestamp from the input port is retained if the container does not provide
	the Signal-Triggered module’s information to the Splitter, This can happen in non-signal 
	triggered container or during transitions in a signal-triggered container.

	Example usage: This parameter is introduced to provide correct timestamp (Signal
	Triggered module’s timestamp) in the reference path from the Rx-EPC so that Mic data and 
	reference data can be synchronized for better echo cancellation.

    @msgpayload
    param_id_splitter_metadata_prop_cfg_t @newpage
 */
#define PARAM_ID_SPLITTER_TIMESTAMP_PROP_CFG 0x080014FC

/*# @h2xmlp_parameter   {"PARAM_ID_SPLITTER_TIMESTAMP_PROP_CFG",
                          PARAM_ID_SPLITTER_TIMESTAMP_PROP_CFG}
    @h2xmlp_description {ID for the parameter that configures splitter's timestamp
                         propagation.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_splitter_mod
    Payload of the #PARAM_ID_SPLITTER_TIMESTAMP_PROP_CFG parameter.
 */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_splitter_timestamp_prop_cfg_t
{
   uint32_t num_ports;
   /*#< @h2xmle_description {Specifies the number of ports for which the
                             timestamp propagation is configured.}
        @h2xmle_default     {0}
        @h2xmle_range       {0..32767} */

#if defined(__H2XML__)
   per_port_ts_cfg_t port_ts_prop_cfg_arr[0];
    /*#< @h2xmle_description       {Variable length array of num_ports on which
                                   timestamp propagation will be configured.
                                   The size of the array depends on num_ports.}
        @h2xmle_variableArraySize {num_ports} */

#endif
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_splitter_timestamp_prop_cfg_t param_id_splitter_timestamp_prop_cfg_t;

/*# @}                      <-- End of the Module -->*/


#endif /* SPLITTER_API_H */
