#ifndef DATA_MARKER_API_H
#define DATA_MARKER_API_H

/**
 * \file data_marker_api.h
 *  
 * \brief data_marker_api.h: This file contains the Module Id, Param IDs and configuration structures exposed by the
 *  DATA_MARKER Module.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "module_cmn_api.h"
#include "metadata_api.h"

/**
     @h2xml_title1           {Data Marker API}
     @h2xml_title_agile_rev  {Data Marker API}
     @h2xml_title_date       {June 18, 2019}
  */

/*==============================================================================
   Constants
==============================================================================*/

/** @ingroup ar_spf_mod_data_marker_macros
    Stack size supported by the Data Marker module. */
#define CAPI_DATA_MARKER_STACK_SIZE 2048

/** @ingroup ar_spf_mod_data_marker_macros
    Maximum number of ports supported by the Data Marker module. */
#define CAPI_DATA_MARKER_MAX_PORTS 1

/** @ingroup ar_spf_mod_data_marker_macros
    ID of the DATA MARKER module. This module is used to insert metadata
    at one point and intercept at another point in a graph.

    @subhead4{Supported input media format ID}
    - Data Format          : FIXED_POINT @lstsp1
    - fmt_id               : Don't care @lstsp1
    - Sample Rates         : Any (>0) @lstsp1
    - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels) @lstsp1
    - Channel type         : 1 to 128 @lstsp1
    - Bits per sample      : 16, 24, 32 @lstsp1
    - Q format             : 15, 27, 31 @lstsp1
    - Interleaving         : Any @lstsp1
    - Signed/unsigned      : Any
*/
#define MODULE_ID_DATA_MARKER 0x0700106A
/**
    @h2xmlm_module       {"MODULE_ID_DATA_MARKER",
                          MODULE_ID_DATA_MARKER}
    @h2xmlm_displayName  {"DATA MARKER"}
    @h2xmlm_modSearchKeys{Debug}
    @h2xmlm_description  {ID of the DATA MARKER module. This module is used to insert metadata
    at one point and intercept at another point in a graph. \n

   * Supported Input Media Format:      \n
   *  - Data Format          : FIXED_POINT \n
   *  - fmt_id               : Don't care \n
   *  - Sample Rates         : Any (>0) \n
   *  - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels)\n
   *  - Channel type         : 1 to 128 \n
   *  - Bits per sample      : 16, 24, 32 \n
   *  - Q format             : 15, 27, 31 \n
   *  - Interleaving         : Any \n
   *  - Signed/unsigned      : Any  \n
   *}
    @h2xmlm_dataMaxInputPorts    {CAPI_DATA_MARKER_MAX_PORTS}
    @h2xmlm_dataMaxOutputPorts   {CAPI_DATA_MARKER_MAX_PORTS}
    @h2xmlm_dataInputPorts       {IN=2}
    @h2xmlm_dataOutputPorts      {OUT=1}
    @h2xmlm_supportedContTypes   {APM_CONTAINER_TYPE_GC, APM_CONTAINER_TYPE_SC}
    @h2xmlm_stackSize            {CAPI_DATA_MARKER_STACK_SIZE}
    @h2xmlm_ToolPolicy           {Calibration}
    @{                   <-- Start of the Module -->
*/

/** @ingroup ar_spf_mod_data_marker_macros
    ID of the MD Insert parameter, used by the Data Marker module. */
#define PARAM_ID_DATA_MARKER_INSERT_MD 0x08001155
/** @h2xmlp_parameter   {"PARAM_ID_DATA_MARKER_INSERT_MD", PARAM_ID_DATA_MARKER_INSERT_MD}
    @h2xmlp_description {Structure for PARAM_ID_DATA_MARKER_INSERT_MD parameter.
                         This tells the module to create a metadatum of said ID and insert to the propagated list.
                         It also provides a token to uniquely identify the inserted MD.}
   @h2xmlp_toolPolicy              {Calibration}                         */
#include "spf_begin_pack.h"

/** @ingroup ar_spf_mod_data_marker_macros
    Payload of the PARAM_ID_DATA_MARKER_INSERT_MD parameter. */
struct param_id_data_marker_insert_md_t
{
   uint32_t metadata_id;
   /**< Specifies the metadata ID to create and propagate. */
   /**< @h2xmle_description   {Specifies the metadata_id to create and propagate.}
        @h2xmle_default       {0x0A001035}
        @h2xmle_range         {0..0xFFFFFFFF}*/

   uint32_t token;
   /**< Token to uniquely track the inserted metadata. */
   /**< @h2xmle_description  {Token to uniquely track the inserted md}
        @h2xmle_default      {0}
        @h2xmle_range        {0..0xFFFFFFFF}*/

   uint32_t frame_duration_ms;
   /**< Specifies the frame duration in ms with which the metadata is to be inserted.
        If set to 0 (default), it will be inserted only once. */
   /**< @h2xmle_description   {Specifies the frame duration in ms with which the metadata is to be inserted.
      						   If set to 0(default) it will be inserted only once}
        @h2xmle_default       {0x0}
        @h2xmle_range         {0..0xFFFFFFFF}*/
}
#include "spf_end_pack.h"
;
/* Structure for PARAM_ID_DATA_MARKER_INSERT_MD parameter. */
typedef struct param_id_data_marker_insert_md_t param_id_data_marker_insert_md_t;

/** @ingroup ar_spf_mod_data_marker_macros
    Unique Parameter ID. */
#define EVENT_ID_DELAY_MARKER_INFO 0x08001156

/*==============================================================================
   Type definitions
==============================================================================*/

/** @ingroup ar_spf_mod_data_marker_macros
    Event raised by the module to convey the path delay. */

/** @h2xmlp_parameter   {"EVENT_ID_DELAY_MARKER_INFO",
                          EVENT_ID_DELAY_MARKER_INFO}
    @h2xmlp_description { Event raised by the module to convey the path delay.}
    @h2xmlp_toolPolicy  { Calibration}*/

#include "spf_begin_pack.h"
struct event_id_delay_marker_info_t
{
   uint32_t delay;
   /**< Delay as seen by the module that raises the event (us). */
   /**< @h2xmle_description   {Delay as seen by the module that raises the event (us)}
         @h2xmle_default      {0}
         @h2xmle_range        {0..0xFFFFFFFF}*/

   uint32_t token;
   /**< Token provided by the Delay marker insertion module. */
   /**< @h2xmle_description   {Token provided by the Delay marker insertion module}
         @h2xmle_default      {0}
         @h2xmle_range        {0..0xFFFFFFFF}*/

   uint32_t frame_counter;
   /**< Specifies the frame counter at which the metadata was inserted.
        If value is 0 (default), metadata was inserted only once. */
   /**< @h2xmle_description   {Specifies the frame counter at which the metadata was inserted.
                               If value is 0(default) md was inserted only once}
        @h2xmle_default       {0x0}
        @h2xmle_range         {0..0xFFFFFFFF}*/
}
#include "spf_end_pack.h"
;

/* Structure for EVENT_ID_DELAY_MARKER_INFO parameter. */
typedef struct event_id_delay_marker_info_t event_id_delay_marker_info_t;

/** @}                   <-- End of the Module -->*/

#endif /* DATA_MARKER_API_H */
