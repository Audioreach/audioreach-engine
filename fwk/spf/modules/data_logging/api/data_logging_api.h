/**
 * \file data_logging_api.h
 * \brief
 *  	 This file contains Public APIs for Data logging Module
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _DATA_LOGGING_API_H_
#define _DATA_LOGGING_API_H_

 /*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "module_cmn_api.h"
#include "apm_container_api.h"

/** @ingroup ar_spf_mod_data_log_macros
    Maximum inport ports of the Data Logging module. */
#define DATA_LOGGING_MAX_INPUT_PORTS                  0x1

/** @ingroup ar_spf_mod_data_log_macros
    Maximum output ports of the Data Logging module. */
#define DATA_LOGGING_MAX_OUTPUT_PORTS                 0x1

/** @ingroup ar_spf_mod_data_log_macros
    Required stack size for the Data Logging module. */
#define DATA_LOGGING_STACK_SIZE_REQUIREMENT           2048

/**
   @h2xmlx_xmlNumberFormat {int}
*/

/*==============================================================================
   Param ID
==============================================================================*/

/** @ingroup ar_spf_mod_data_log_macros
    ID of the parameter used to set configuration for Data logging module. */
#define PARAM_ID_DATA_LOGGING_CONFIG             0x08001031

/*==============================================================================
   Param structure defintions
==============================================================================*/

/** @h2xmlp_parameter   {"PARAM_ID_DATA_LOGGING_CONFIG", PARAM_ID_DATA_LOGGING_CONFIG}
   @h2xmlp_description  {Configures the data logging module.\n}
   @h2xmlp_toolPolicy   {Calibration; RTC} */

#include "spf_begin_pack.h"

/** @ingroup ar_spf_mod_data_log_macros
    Configures the data logging module. */
struct data_logging_config_t
{
   uint32_t log_code;
   /**< Logging code for this module instance.

        @valuesbul
		- 0 -- Disabled (Default)
   		- 0x152E
		- 0x152F
		- 0x1531
		- 0x1534
		- 0x1532
		- 0x1530
		- 0x1533
        - 0x1535
        - 0x1536
		- 0x1586
        - 0x19AF
	    - 0x19B0
        - 0x19B1
        - 0x158A
		- 0x158B
		*/
   /**< @h2xmle_description {logging code}
        @h2xmle_default     {0}
         @h2xmle_rangeList    {"Default"=0;
                              "0x152E"=0x152E;
							  "0x152F"=0x152F;
							  "0x1531"=0x1531;
							  "0x1534"=0x1534;
							  "0x1532"=0x1532;
							  "0x1530"=0x1530;
							  "0x1533"=0x1533;
							  "0x1535"=0x1535;
							  "0x1536"=0x1536;
							  "0x1586"=0x1586;
							  "0x19AF"=0x19AF;
							  "0x19B0"=0x19B0;
							  "0x19B1"=0x19B1;
							  "0x158A"=0x158A;
							  "0x158B"=0x158B
							  }
        @h2xmle_policy      {Basic} */

   uint32_t log_tap_point_id;
   /**< Logging tap point of this module instance

        @values
        0 to 0x10FC5 */

   /**< @h2xmle_description {logging tap point}
        @h2xmle_default     {0}
        @h2xmle_range       { 0..69573}
        @h2xmle_policy      {Basic} */

   uint32_t mode;
   /**< Mode to indicate whether to log immediately (1) or wait until
    *   log buffer is completely filled (0)

        @values
        0, 1 */

   /**< @h2xmle_description {Buffering mode}
        @h2xmle_default     {0}
        @h2xmle_rangeList   { "Buffered"=0,
                              "Immediate"=1}
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;
/* Structure type def for above payload. */
typedef struct data_logging_config_t data_logging_config_t;

/** @ingroup ar_spf_mod_data_log_macros
    ID of the parameter used to configure Island logging for Data logging module. */
#define PARAM_ID_DATA_LOGGING_ISLAND_CFG         0x08001313

/** @h2xmlp_parameter   {"PARAM_ID_DATA_LOGGING_ISLAND_CFG", PARAM_ID_DATA_LOGGING_ISLAND_CFG}
   @h2xmlp_description  {Force logging for a logging module in an Island Container. Enabling this causes island exit\n}
   @h2xmlp_toolPolicy   {Calibration} */

#include "spf_begin_pack.h"

/** @ingroup ar_spf_mod_data_log_macros
    Force logging for a logging module in an Island Container. Enabling this causes island exit. */
struct data_logging_island_t
{
   uint32_t forced_logging;
   /**< Specifies whether the data logging module needs to log in island by exiting island. */

   /**< @h2xmle_description  {Specifies whether the data logging module needs to log in island by exiting island.}
        @h2xmle_rangeList    {"disallowed"=0;
                              "allowed"=1}
        @h2xmle_default      {0}
        @h2xmle_range        { 0..1}
        @h2xmle_policy       {Basic} */
}
#include "spf_end_pack.h"
;
/* Structure type def for above payload. */
typedef struct data_logging_island_t data_logging_island_t;


/** @ingroup ar_spf_mod_data_log_macros
    ID of the parameter used to configure selective channel data logging*/
#define PARAM_ID_DATA_LOGGING_SELECT_CHANNELS         0x080014E8

/** @h2xmlp_parameter   {"PARAM_ID_DATA_LOGGING_SELECT_CHANNELS", PARAM_ID_DATA_LOGGING_SELECT_CHANNELS}
    @h2xmlp_description  {Select channels to log.\n}
    @h2xmlp_toolPolicy   {NO_SUPPORT} */

/** @ingroup ar_spf_mod_data_log_macros
    Select the channel index mask for the Data Logging module. */
#define DATA_LOGGING_SELECT_CHANNEL_INDEX_MASK 0x0

/** @ingroup ar_spf_mod_data_log_macros
    Select channel type mask for the Data Logging module. */
#define DATA_LOGGING_SELECT_CHANNEL_TYPE_MASK 0x1

#include "spf_begin_pack.h"

/** @ingroup ar_spf_mod_data_log_macros
    Select channels to log. */

struct data_logging_select_channels_t
{
   uint32_t mode;
   /**< Mode to decide if selective channel logging is based on channel-index or channel-type. */

   /**< @h2xmle_description  {mode to decide if selective channel logging is based on channel-index or channel-type.}
        @h2xmle_default      {#DATA_LOGGING_SELECT_CHANNEL_INDEX_MASK}
        @h2xmle_rangeList    {"channel type mask" = #DATA_LOGGING_SELECT_CHANNEL_TYPE_MASK,
			      "channel index mask" = #DATA_LOGGING_SELECT_CHANNEL_INDEX_MASK}
        @h2xmle_policy       {Basic} */

   uint32_t channel_index_mask;
   /**< Bit mask to select the channels to log based on channel index.
        This is valid only if mode is selected as channel-index-mask. */

   /**< @h2xmle_description  {Bit mask to select the channels to log based on channel index.
                              This is valid only if mode is selected as channel-index-mask.}
        @h2xmle_default      {0xFFFFFFFF}
        @h2xmle_range        {0x0..0xFFFFFFFF}
        @h2xmle_policy       {Basic}

        @h2xmle_bitfield			{0x00000001}
  			@h2xmle_bitName			{CHANNEL_1}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000002}
  			@h2xmle_bitName			{CHANNEL_2}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000004}
  			@h2xmle_bitName			{CHANNEL_3}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000008}
  			@h2xmle_bitName			{CHANNEL_4}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000010}
  			@h2xmle_bitName			{CHANNEL_5}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000020}
  			@h2xmle_bitName			{CHANNEL_6}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000040}
  			@h2xmle_bitName			{CHANNEL_7}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000080}
  			@h2xmle_bitName			{CHANNEL_8}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000100}
  			@h2xmle_bitName			{CHANNEL_9}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000200}
  			@h2xmle_bitName			{CHANNEL_10}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000400}
  			@h2xmle_bitName			{CHANNEL_11}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000800}
  			@h2xmle_bitName			{CHANNEL_12}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00001000}
  			@h2xmle_bitName			{CHANNEL_13}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00002000}
  			@h2xmle_bitName			{CHANNEL_14}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00004000}
  			@h2xmle_bitName			{CHANNEL_15}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00008000}
  			@h2xmle_bitName			{CHANNEL_16}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00010000}
  			@h2xmle_bitName			{CHANNEL_17}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00020000}
  			@h2xmle_bitName			{CHANNEL_18}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00040000}
  			@h2xmle_bitName			{CHANNEL_19}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

         @h2xmle_bitfield			{0x00080000}
  			@h2xmle_bitName			{CHANNEL_20}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00100000}
  			@h2xmle_bitName			{CHANNEL_21}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00200000}
  			@h2xmle_bitName			{CHANNEL_22}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00400000}
  			@h2xmle_bitName			{CHANNEL_23}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00800000}
  			@h2xmle_bitName			{CHANNEL_24}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x01000000}
  			@h2xmle_bitName			{CHANNEL_25}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x02000000}
  			@h2xmle_bitName			{CHANNEL_26}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x04000000}
  			@h2xmle_bitName			{CHANNEL_27}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x08000000}
  			@h2xmle_bitName			{CHANNEL_28}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x10000000}
  			@h2xmle_bitName			{CHANNEL_29}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x20000000}
  			@h2xmle_bitName			{CHANNEL_30}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x40000000}
  			@h2xmle_bitName			{CHANNEL_31}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x80000000}
  			@h2xmle_bitName			{CHANNEL_32}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        */

   uint32_t channel_type_mask_lsw;
   /**< Bit mask to select the channels to log based on channel type.
        This is valid only if mode is selected as channel-type-mask. */

   /**< @h2xmle_description  {Bit mask to select the channels to log based on channel type.
			                  This is valid only if mode is selected as channel-type-mask.}
        @h2xmle_default      {0xFFFFFFFE}
        @h2xmle_range        {0x0..0xFFFFFFFE}
        @h2xmle_policy       {Basic}

        @h2xmle_bitfield			{0x00000001}
  			@h2xmle_bitName			{Bit_1}
  			@h2xmle_default                 {0x0}
  			@h2xmle_visibility		{hide}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000002}
  			@h2xmle_bitName			{PCM_CHANNEL_L}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000004}
  			@h2xmle_bitName			{PCM_CHANNEL_R}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000008}
  			@h2xmle_bitName			{PCM_CHANNEL_C}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000010}
  			@h2xmle_bitName			{PCM_CHANNEL_LS}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000020}
  			@h2xmle_bitName			{PCM_CHANNEL_RS}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000040}
  			@h2xmle_bitName			{PCM_CHANNEL_LFE}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000080}
  			@h2xmle_bitName			{PCM_CHANNEL_CS_PCM_CHANNEL_CB}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000100}
  			@h2xmle_bitName			{PCM_CHANNEL_LB}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000200}
  			@h2xmle_bitName			{PCM_CHANNEL_RB}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000400}
  			@h2xmle_bitName			{PCM_CHANNEL_TS}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000800}
  			@h2xmle_bitName			{PCM_CHANNEL_CVH_PCM_CHANNEL_TFC}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00001000}
  			@h2xmle_bitName			{PCM_CHANNEL_MS}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00002000}
  			@h2xmle_bitName			{PCM_CHANNEL_FLC}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00004000}
  			@h2xmle_bitName			{PCM_CHANNEL_FRC}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00008000}
  			@h2xmle_bitName			{PCM_CHANNEL_RLC}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00010000}
  			@h2xmle_bitName			{PCM_CHANNEL_RRC}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00020000}
  			@h2xmle_bitName			{PCM_CHANNEL_LFE2}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00040000}
  			@h2xmle_bitName			{PCM_CHANNEL_SL}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

         @h2xmle_bitfield			{0x00080000}
  			@h2xmle_bitName			{PCM_CHANNEL_SR}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00100000}
  			@h2xmle_bitName			{PCM_CHANNEL_TFL_PCM_CHANNEL_LVH}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00200000}
  			@h2xmle_bitName			{PCM_CHANNEL_TFR_PCM_CHANNEL_RVH}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00400000}
  			@h2xmle_bitName			{PCM_CHANNEL_TC}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00800000}
  			@h2xmle_bitName			{PCM_CHANNEL_TBL}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x01000000}
  			@h2xmle_bitName			{PCM_CHANNEL_TBR}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x02000000}
  			@h2xmle_bitName			{PCM_CHANNEL_TSL}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x04000000}
  			@h2xmle_bitName			{PCM_CHANNEL_TSR}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x08000000}
  			@h2xmle_bitName			{PCM_CHANNEL_TBC}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x10000000}
  			@h2xmle_bitName			{PCM_CHANNEL_BFC}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x20000000}
  			@h2xmle_bitName			{PCM_CHANNEL_BFL}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x40000000}
  			@h2xmle_bitName			{PCM_CHANNEL_BFR}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x80000000}
  			@h2xmle_bitName			{PCM_CHANNEL_LW}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd
        */

   uint32_t channel_type_mask_msw;
   /**< Bit mask to select the channels to log based on channel type.
        This is valid only if mode is selected as channel-type-mask. */

   /**< @h2xmle_description  {Bit mask to select the channels to log based on channel type.
			                  This is valid only if mode is selected as channel-type-mask.}
        @h2xmle_default      {0xFFFF0007}
        @h2xmle_range        {0x0..0xFFFFFFFF}
        @h2xmle_policy       {Basic}
        @h2xmle_bitfield			{0x00000001}
  			@h2xmle_bitName			{PCM_CHANNEL_RW}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00000002}
  			@h2xmle_bitName			{PCM_CHANNEL_LSD}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

         @h2xmle_bitfield			{0x00000004}
  			@h2xmle_bitName			{PCM_CHANNEL_RSD}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

  	@h2xmle_bitfield			{0x0000FFF8}
  			@h2xmle_bitName			{Bit_4_16}
  			@h2xmle_default                 {0x0}
  			@h2xmle_visibility		{hide}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00010000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_1}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00020000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_2}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00040000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_3}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

         @h2xmle_bitfield			{0x00080000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_4}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00100000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_5}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00200000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_6}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00400000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_7}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x00800000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_8}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x01000000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_9}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x02000000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_10}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x04000000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_11}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x08000000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_12}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x10000000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_13}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x20000000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_14}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x40000000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_15}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd

        @h2xmle_bitfield			{0x80000000}
  			@h2xmle_bitName			{PCM_CUSTOM_CHANNEL_MAP_16}
  			@h2xmle_default                 {0x1}
  			@h2xmle_rangeList		{"Disable"=0; "Enable"=1}
  	@h2xmle_bitfieldEnd
        */
}
#include "spf_end_pack.h"
;
/* Structure type def for above payload. */
typedef struct data_logging_select_channels_t data_logging_select_channels_t;

/** @h2xmlp_subStruct */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/** @ingroup ar_spf_mod_data_log_macros
    Select data logging channel type mask. */
struct data_logging_channel_type_mask_t
{
   uint32_t channel_type_group_mask;
   /**< Indicates the mask for channel type group array.
        Each bit in channel_type_group_mask corresponds to a channel group.
        Read as:
        - Bit 0 corresponds to channel group 1, which includes channel map for channels 1-31.
        - Bit 1 corresponds to channel group 2, which includes channel map for channels 32-63.
        - Bit 2 corresponds to channel group 3, which includes channel map for channels 64-95.
        - Bit 3 corresponds to channel group 4, which includes channel map for channels 96-127.
        - Bit 4 corresponds to channel group 5, which includes channel map for channel 128-159.

        A set bit (1) in channel_type_group_mask indicates that the channels in that channel group are configured. */

   /**< @h2xmle_description  {Indicates the mask for channel type group array.
                             Each bit in channel_type_group_mask corresponds to a channel group.
                             Read as
                             Bit 0 corresponds to channel group 1, which includes channel map for channels 1-31.
                             Bit 1 corresponds to channel group 2, which includes channel map for channels 32-63.
                             Bit 2 corresponds to channel group 3, which includes channel map for channels 64-95.
                             Bit 3 corresponds to channel group 4, which includes channel map for channels 96-127.
                             Bit 4 corresponds to channel group 5, which includes channel map for channel 128-159.

                             A set bit (1) in channel_type_group_mask indicates that the channels in that channel group are configured. \n
                             }
       @h2xmle_range             {0..31}
       @h2xmle_default          {0x00000003} */

   uint32_t channel_type_mask_list[0];
   /**< An array used to configure the channels for different channel groups. The array size depends on the number of
        bits set in channel_type_group_mask.

        For group 1, each bit of channel_type_mask_list corresponds to channel map from 1 (PCM_CHANNEL_L) to 31 (PCM_CHANNEL_LW).
        Bit 0 of group 1 channel_type_mask_list is reserved and must always be set to zero.
        For any other group, each bit of channel_mask corresponds to channel map from [32(group_no -1) to 32(group_no)-1].

        Bit position of the channel-map for channel_type_mask_list of defined group is obtained by left shifting (1 (left shift) Channel_map%32). */

   /**< @h2xmle_description  {An array used to configure the channels for different channel groups. The array size depends on the number of
                             bits set in channel_type_group_mask.\n

                             For group 1, each bit of channel_type_mask_list corresponds to channel map from 1 (PCM_CHANNEL_L) to 31 (PCM_CHANNEL_LW). \n
                             Bit 0 of group 1 channel_type_mask_list is reserved and must always be set to zero.\n
                             For any other group, each bit of channel_mask corresponds to channel map from [32(group_no -1) to 32(group_no)-1]. \n

                             Bit position of the channel-map for channel_type_mask_list of defined group is obtained by left shifting (1 (left shift) Channel_map%32).\n
                             }
         @h2xmle_variableArraySizeFunction {GET_SET_BITS_COUNT, channel_type_group_mask}
         @h2xmle_copySrcList{channel_type_mask_lsw, channel_type_mask_msw}
         @h2xmlx_expandStructs {false}
         @h2xmle_defaultList {0xfffffffe, 0xffffffff}*/

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
/* Structure type def for above payload. */
typedef struct data_logging_channel_type_mask_t data_logging_channel_type_mask_t;

/** @h2xmlp_subStruct */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/** @ingroup ar_spf_mod_data_log_macros
    Data logging channel index mask. */
struct data_logging_channel_index_mask_t
{
   uint32_t channel_index_group_mask;
   /**< Indicates the mask for channel index group array.
        Each bit in channel_index_group_mask corresponds to a channel group.
        Read as:
        - Bit 0 corresponds to channel group 1, which includes position index for channels 1-32.
        - Bit 1 corresponds to channel group 2, which includes position index for channels 33-64.
        - Bit 2 corresponds to channel group 3, which includes position index for channels 65-96.
        - Bit 3 corresponds to channel group 4, which includes position index for channels 97-128.

        A set bit (1) in channel_index_group_mask indicates that the channels in that channel group are configured. */

   /**< @h2xmle_description  {Indicates the mask for channel index group array. \n
                             Each bit in channel_index_group_mask corresponds to a channel group. \n
                             Read as \n
                             Bit 0 corresponds to channel group 1, which includes position index for channels 1-32. \n
                             Bit 1 corresponds to channel group 2, which includes position index for channels 33-64. \n
                             Bit 2 corresponds to channel group 3, which includes position index for channels 65-96. \n
                             Bit 3 corresponds to channel group 4, which includes position index for channels 97-128. \n

                             A set bit (1) in channel_index_group_mask indicates that the channels in that channel group are configured. \n
                             }
        @h2xmle_range             {0..15}
        @h2xmle_default          {0x00000001} */

   uint32_t channel_index_mask_list[0];
   /**< An array used to configure the channels for different channel groups. The array size depends on the number of
        bits set in channel_index_group_mask.

        For any group, each bit of channel_index_mask corresponds to channel index from [32(group_no -1)+1 to 32(group_no)].

        Bit position of the channel-index for channel_index_mask of defined group is obtained by (1 (left shift) (Channel_index-1)%32). */

   /**< @h2xmle_description  {An array used to configure the channels for different channel groups. The array size depends on the number of
                             bits set in channel_index_group_mask.\n

                             For any group, each bit of channel_index_mask corresponds to channel index from [32(group_no -1)+1 to 32(group_no)].

                             Bit position of the channel-index for channel_index_mask of defined group is obtained by (1 (left shift) (Channel_index-1)%32).\n
                             }
         @h2xmle_variableArraySizeFunction {GET_SET_BITS_COUNT, channel_index_group_mask}
         @h2xmle_copySrcList{channel_index_mask}
         @h2xmlx_expandStructs {false}
         @h2xmle_defaultList {0xffffffff}*/

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
/* Structure type def for above payload. */
typedef struct data_logging_channel_index_mask_t data_logging_channel_index_mask_t;

/** @ingroup ar_spf_mod_data_log_macros
    ID of the parameter used to configure selective channel data logging. */
#define PARAM_ID_DATA_LOGGING_SELECT_CHANNELS_V2         0x08001A66

/** @h2xmlp_parameter   {"PARAM_ID_DATA_LOGGING_SELECT_CHANNELS_V2", PARAM_ID_DATA_LOGGING_SELECT_CHANNELS_V2}
    @h2xmlp_copySrc     {0x080014E8}
    @h2xmlp_description  {Select channels to log.\n}
    @h2xmlp_toolPolicy   {Calibration; RTC} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/** @ingroup ar_spf_mod_data_log_macros
    Select channels to log. */

struct data_logging_select_channels_v2_t
{
   uint32_t mode;
   /**< Mode to decide if selective channel logging is based on channel-index or channel-type */

   /**< @h2xmle_description  {mode to decide if selective channel logging is based on channel-index or channel-type.}
        @h2xmle_default      {#DATA_LOGGING_SELECT_CHANNEL_INDEX_MASK}
        @h2xmle_copySrc      {mode}
        @h2xmle_rangeList    {"channel index config" = #DATA_LOGGING_SELECT_CHANNEL_INDEX_MASK,
                             "channel type config" = #DATA_LOGGING_SELECT_CHANNEL_TYPE_MASK}
        @h2xmle_policy       {Basic} */
#ifdef __H2XML__
   data_logging_channel_index_mask_t channel_index_config;
   /**< Mask to select the channels to log based on channel index.
        This is valid only if mode is selected as channel-index-mask. */

   /**< @h2xmle_description  {Mask to select the channels to log based on channel index.
                              This is valid only if mode is selected as channel-index-mask.}
        @h2xmle_policy       {Basic} */

   data_logging_channel_type_mask_t channel_type_config;
   /**< Mask to select the channels to log based on channel type.
        This is valid only if mode is selected as channel-type-mask. */

   /**< @h2xmle_description  {Mask to select the channels to log based on channel type.
                              This is valid only if mode is selected as channel-type-mask. }
        @h2xmle_policy       {Basic} */
   #endif //H@XML
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
/* Structure type def for above payload. */
typedef struct data_logging_select_channels_v2_t data_logging_select_channels_v2_t;


/** @ingroup ar_spf_mod_data_log_macros
    ID of the Logging module.
 
    @subhead4{Supported input media format ID}
    - Data Format          : any @lstsp1
    - fmt_id               : Don't care @lstsp1
    - Sample Rates         : 1-384 kHz @lstsp1
    - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels) @lstsp1
    - Bit Width            : 16 (bits per sample 16 and Q15),
                             24 (bits per sample 24 and Q23, bits per sample 32 and Q23 or Q27 or Q31),
                             32 (bits per sample 32 and Q31) @lstsp1
    - Interleaving         : interleaved, deinterleaved unpacked, deinterleaved packed @lstsp1
    - Endianess            : little, big
 */
#define MODULE_ID_DATA_LOGGING                   0x0700101A

/** @h2xml_title1          {Module Data Logging}
    @h2xml_title_agile_rev {Module Data Logging}
    @h2xml_title_date      {March 29, 2019}
  */

/**
    @h2xmlm_module         {"MODULE_ID_DATA_LOGGING", MODULE_ID_DATA_LOGGING}
    @h2xmlm_displayName    {"Data Logging"}
    @h2xmlm_modSearchKeys  {Debug}
    @h2xmlm_description    {
                            This module is used to log PCM data and bit stream data
                            This module has only one input and one output port.
                            This module is an in place module.
                            This module supports the following parameter IDs, \n
                            - PARAM_ID_DATA_LOGGING_CONFIG\n
                            - PARAM_ID_DATA_LOGGING_ISLAND_CFG\n
                            - PARAM_ID_DATA_LOGGING_SELECT_CHANNELS\n
                            }

    @h2xmlm_dataMaxInputPorts    {DATA_LOGGING_MAX_INPUT_PORTS}
    @h2xmlm_dataInputPorts       {IN = 2}
    @h2xmlm_dataMaxOutputPorts   {DATA_LOGGING_MAX_OUTPUT_PORTS}
    @h2xmlm_dataOutputPorts      {OUT = 1}
    @h2xmlm_supportedContTypes  { APM_CONTAINER_TYPE_SC,APM_CONTAINER_TYPE_GC,APM_CONTAINER_TYPE_PTC}
    @h2xmlm_isOffloadable        {true}
    @h2xmlm_stackSize            { DATA_LOGGING_STACK_SIZE_REQUIREMENT }
    @h2xmlm_toolPolicy           {Calibration}
    @{                     <-- Start of the Module -->
    @h2xml_Select          {"data_logging_config_t"}
    @h2xmlm_InsertParameter

    @h2xml_Select              {param_id_module_enable_t}
    @h2xmlm_InsertParameter
    @h2xml_Select              {param_id_module_enable_t::enable}
    @h2xmle_default            {1}

    @h2xml_Select          {"data_logging_island_t"}
    @h2xmlm_InsertParameter

    @h2xml_Select          {"data_logging_select_channels_v2_t"}
    @h2xmlm_InsertParameter

    @h2xml_Select          {"data_logging_channel_index_mask_t"}
    @h2xmlm_InsertParameter

	@h2xml_Select          {"data_logging_channel_type_mask_t"}
    @h2xmlm_InsertParameter

    @}                     <-- End of the Module -->
*/

#endif /* _DATA_LOGGING_API_H_ */
