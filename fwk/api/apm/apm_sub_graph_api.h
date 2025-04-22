#ifndef _APM_SUB_GRAPH_API_H_
#define _APM_SUB_GRAPH_API_H_
/**
 * \file apm_sub_graph_api.h
 * \brief
 *     This file contains APM sub-graph related commands and events data structures definitions
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "apm_graph_properties.h"

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

/*====================================================================================================================*/
/*====================================================================================================================*/

/** @ingroup spf_apm_subgraph_props
    Identifier for the parameter that configures the sub-graph properties.
    This ID must be used with the #APM_CMD_GRAPH_OPEN command.

    @msgpayload
    apm_param_id_sub_graph_cfg_t \n
    @indent{12pt} apm_sub_graph_cfg_t
*/
#define APM_PARAM_ID_SUB_GRAPH_CONFIG             0x08001001

/*# h2xmlp_parameter   {"Sub-Graph Config", APM_PARAM_ID_SUB_GRAPH_CONFIG}
     h2xmlp_description {ID for the parameter that configures the sub-graph
                          properties. This ID must be used with the
                          APM_CMD_GRAPH_OPEN command.}
     h2xmlp_toolPolicy  {RTC; Calibration} */

/** @ingroup spf_apm_subgraph_props
    Payload for #APM_PARAM_ID_SUB_GRAPH_CONFIG.

    Immediately following this structure is a variable length array of
    apm_sub_graph_cfg_t objects that correspond to the configuration and
    properties of each sub-graph. The length of the array is determined by the
    number of sub-graphs being configured.
*/
#include "spf_begin_pack.h"
struct apm_param_id_sub_graph_cfg_t
{
   uint32_t num_sub_graphs;
   /**< Number of sub-graphs being configured. */

   /*#< h2xmle_description {Number of sub-graphs being configured.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_param_id_sub_graph_cfg_t apm_param_id_sub_graph_cfg_t;


/** @ingroup spf_apm_subgraph_props
    Sub-graph configuration payload for #APM_PARAM_ID_SUB_GRAPH_CONFIG.

    Immediately following this structure is a variable length array of
    sub-graph property structures. The length of the array is determined by
    the number of properties being configured.
*/
#include "spf_begin_pack.h"
struct apm_sub_graph_cfg_t
{
   uint32_t sub_graph_id;
   /**< Unique identifier for the sub-graph. */

   /*#< h2xmle_description {Unique identifier for the sub-graph.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t num_sub_graph_prop;
   /**< Number of properties being configured for this sub-graph
        (default&nbsp;=&nbsp;0). @newpagetable */

   /*#< h2xmle_description {Number of properties being configured for this
                             sub-graph.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_sub_graph_cfg_t apm_sub_graph_cfg_t;

/*# @h2xmlgp_config      {"SG_CFG", "APM_PARAM_ID_SUB_GRAPH_CONFIG",
                           APM_PARAM_ID_SUB_GRAPH_CONFIG}
    @h2xmlgp_description {ID for the parameter that configures the sub-graph
                          properties. This ID must be used with the
                          APM_CMD_GRAPH_OPEN command.}
    @{                   <-- Start of subgraph_cfg --> */

/** @ingroup spf_apm_subgraph_props
    Sub-graph property identifier for the performance mode.

    @msgpayload
    apm_sg_prop_id_perf_mode_t
*/
#define APM_SUB_GRAPH_PROP_ID_PERF_MODE           0x0800100E

/** @ingroup spf_apm_subgraph_props
    Low Power performance mode for the sub-graph. */
#define APM_SG_PERF_MODE_LOW_POWER                0x1

/** @ingroup spf_apm_subgraph_props
    Low Latency performance mode for the sub-graph. */
#define APM_SG_PERF_MODE_LOW_LATENCY              0x2


/*# @h2xmlp_property    {"Performance Mode", APM_SUB_GRAPH_PROP_ID_PERF_MODE}
    @h2xmlp_description {Sub-graph property ID for the performance mode.} */

/** @ingroup spf_apm_subgraph_props
    Payload for #APM_SUB_GRAPH_PROP_ID_PERF_MODE.
*/
#include "spf_begin_pack.h"
struct apm_sg_prop_id_perf_mode_t
{
   uint32_t perf_mode;
   /**< Performance mode of the sub-graph.

        @valuesbul
        - #APM_SG_PERF_MODE_LOW_POWER
        - #APM_SG_PERF_MODE_LOW_LATENCY @tablebulletend */

   /*#< @h2xmle_rangeList   {"Low Power"=APM_SG_PERF_MODE_LOW_POWER,
                             "Low Latency"=APM_SG_PERF_MODE_LOW_LATENCY}
        @h2xmle_default     {APM_SG_PERF_MODE_LOW_POWER}
        @h2xmle_description {Performance mode of the sub-graph.} */
}
#include "spf_end_pack.h"
;
typedef struct apm_sg_prop_id_perf_mode_t apm_sg_prop_id_perf_mode_t;


/*--------------------------------------------------------------------------------------------------------------------*/
/** @ingroup spf_apm_subgraph_props
    Direction of the sub-graph property ID.

    @msgpayload
    apm_sg_prop_id_direction_t
*/
#define APM_SUB_GRAPH_PROP_ID_DIRECTION           0x0800100F

/** @ingroup spf_apm_subgraph_props
    Transmit direction for the sub-graph. */
#define APM_SUB_GRAPH_DIRECTION_TX                0x1

/** @ingroup spf_apm_subgraph_props
    Receive direction for the sub-graph. */
#define APM_SUB_GRAPH_DIRECTION_RX                0x2


/*# @h2xmlp_property    {"SG Direction", APM_SUB_GRAPH_PROP_ID_DIRECTION}
    @h2xmlp_description {Direction of the sub-graph property ID.} */

/** @ingroup spf_apm_subgraph_props
    Payload for #APM_SUB_GRAPH_PROP_ID_DIRECTION.
*/
#include "spf_begin_pack.h"
struct apm_sg_prop_id_direction_t
{
   uint32_t direction;
   /**< Data flow direction for the sub-graph.

        @valuesbul
        - #APM_SUB_GRAPH_DIRECTION_TX
        - #APM_SUB_GRAPH_DIRECTION_RX
        - #APM_PROP_ID_DONT_CARE (Default) @tablebulletend */

   /*#< @h2xmle_rangeList   {"DONT_CARE"=0xFFFFFFFF,
                             "TX"=APM_SUB_GRAPH_DIRECTION_TX,
                             "RX"=APM_SUB_GRAPH_DIRECTION_RX}
        @h2xmle_default     {0xFFFFFFFF}
        @h2xmle_description {Data flow direction for the sub-graph.} */
}
#include "spf_end_pack.h"
;
typedef struct apm_sg_prop_id_direction_t apm_sg_prop_id_direction_t;


/*--------------------------------------------------------------------------------------------------------------------*/
/** @ingroup spf_apm_subgraph_props
    Scenario identifier for the sub-graph property ID.

    @msgpayload
    apm_sg_prop_id_scenario_id_t
*/
#define APM_SUB_GRAPH_PROP_ID_SCENARIO_ID         0x08001010

/** @ingroup spf_apm_subgraph_props
    Scenario ID for audio playback . */
#define APM_SUB_GRAPH_SID_AUDIO_PLAYBACK          0x1

/** @ingroup spf_apm_subgraph_props
    Scenario ID for audio record. */
#define APM_SUB_GRAPH_SID_AUDIO_RECORD            0x2

/** @ingroup spf_apm_subgraph_props
    Scenario ID for a voice call. */
#define APM_SUB_GRAPH_SID_VOICE_CALL              0x3


/*# @h2xmlp_property    {"Scenario ID", APM_SUB_GRAPH_PROP_ID_SCENARIO_ID}
    @h2xmlp_description {Scenario ID for the sub-graph property ID.} */

/** @ingroup spf_apm_subgraph_props
    Payload for #APM_SUB_GRAPH_PROP_ID_SCENARIO_ID.
*/
#include "spf_begin_pack.h"
struct apm_sg_prop_id_scenario_id_t
{
   uint32_t scenario_id;
   /**< Scenario identifier for the sub-graph.

        @valuesbul
        - #APM_SUB_GRAPH_SID_AUDIO_PLAYBACK
        - #APM_SUB_GRAPH_SID_AUDIO_RECORD
        - #APM_SUB_GRAPH_SID_VOICE_CALL
        - #APM_PROP_ID_DONT_CARE (Default) @tablebulletend */

   /*#< @h2xmle_rangeList   {"DONT_CARE"=0xFFFFFFFF,
                             "Audio Playback"=APM_SUB_GRAPH_SID_AUDIO_PLAYBACK,
                             "Audio Record"=APM_SUB_GRAPH_SID_AUDIO_RECORD,
                             "Voice Call"=APM_SUB_GRAPH_SID_VOICE_CALL}
        @h2xmle_default     {0xFFFFFFFF}
        @h2xmle_description {Scenario ID for the sub-graph.} */
}
#include "spf_end_pack.h"
;
typedef struct apm_sg_prop_id_scenario_id_t apm_sg_prop_id_scenario_id_t;

/*--------------------------------------------------------------------------------------------------------------------*/

/** @ingroup spf_apm_subgraph_props
    Voice System identifier (VSID) for the sub-graph property ID.

    @msgpayload
    apm_sg_prop_id_vsid_t
*/
#define APM_SUB_GRAPH_PROP_ID_VSID                0x080010CC

/** @ingroup spf_apm_subgraph_props
    VSID voice call with Subscription 1 and VSID 0x11C05000. */
#define APM_SUB_GRAPH_VSID_SUB1                   AR_NON_GUID(0x11C05000)

/** @ingroup spf_apm_subgraph_props
    VSID voice call with Subscription 2 and VSID 0x11DC5000. */
#define APM_SUB_GRAPH_VSID_SUB2                   AR_NON_GUID(0x11DC5000)

/** @ingroup spf_apm_subgraph_props
    Applications-only mode with a vocoder loopback on Subscription 1. */
#define APM_SUB_GRAPH_VSID_VOC_LB_SUB1            AR_NON_GUID(0x12006000)

/** @ingroup spf_apm_subgraph_props
    Applications-only mode with a vocoder loopback on Subscription 2. */
#define APM_SUB_GRAPH_VSID_VOC_LB_SUB2            AR_NON_GUID(0x121C6000)

/** @ingroup spf_apm_subgraph_props
    <em>Don't care</em> value for the VSID, which the HLOS client enumerates at
    a later time. */
#define APM_SUB_GRAPH_VSID_DONT_CARE              AR_NON_GUID(0xFFFFFFFF)


/*# @h2xmlp_property    {"PROP_ID_VSID", APM_SUB_GRAPH_PROP_ID_VSID}
    @h2xmlp_description {Voice System ID for the sub-graph property ID.}
    @h2xmlp_isVoice     {true} */

/** @ingroup spf_apm_subgraph_props
    Payload for #APM_SUB_GRAPH_PROP_ID_VSID.
*/
#include "spf_begin_pack.h"
struct apm_sg_prop_id_vsid_t
{
   uint32_t vsid;
   /**< Voice System identifier for the sub-graph.

        @valuesbul
        - #APM_SUB_GRAPH_VSID_SUB1
        - #APM_SUB_GRAPH_VSID_SUB2
        - #APM_SUB_GRAPH_VSID_VOC_LB_SUB1
        - #APM_SUB_GRAPH_VSID_VOC_LB_SUB2
        - #APM_SUB_GRAPH_VSID_DONT_CARE (Default) @tablebulletend */

   /*#< @h2xmle_rangeList   {"VSID_DONT_CARE"=0xFFFFFFFF,
                             "VSID_SUB1"=0x11C05000,
                             "VSID_SUB2"=0x11DC5000,
                             "VSID_LB_SUB1"=0x12006000,
                             "VSID_LB_SUB2"=0x121C6000}
        @h2xmle_default     {0xFFFFFFFF}
        @h2xmle_description {Voice System ID for the sub-graph.} */
}
#include "spf_end_pack.h"
;
typedef struct apm_sg_prop_id_vsid_t apm_sg_prop_id_vsid_t;

/*--------------------------------------------------------------------------------------------------------------------*/

#define APM_SUBGRAPH_PROP_ID_CLOCK_SCALE_FACTOR 0x08001374
/** @h2xmlp_property {"Clock Scale Factor", APM_SUBGRAPH_PROP_ID_CLOCK_SCALE_FACTOR}
    @h2xmlp_description {Sub-graph property ID Clock Scale factor}
    @h2xmlp_isVoice     {false} */

#include "spf_begin_pack.h"
struct apm_subgraph_prop_id_clock_scale_factor_t
{
   uint16_t enable_duty_cycling;
   /**< @h2xmle_rangeList 	{"Supports Duty Cycling" = 1,
                             "Does Not support Duty Cycling" = 0}
         @h2xmle_default 	{0}
         @h2xmle_description{Duty Cycling Mode} */

   uint16_t duty_cycling_clock_scale_factor_q4;
   /**< @h2xmle_range       {0x0000..0xFFFF}
    	@h2xmle_default		{0x10}
        @h2xmle_description {Duty Cycling Clock Scale Factor, to be modified only if enable flag is set\n
                         	 *i.e. only for Duty Cycling Subgraphs.\n
                             *clock scale factor of 2.5 means clock has to be bumped up by 2.5 times.\n
                             *So, duty cycling is 1/2.5=0.4 or 40 percent.\n
                             *100 ms data can thus be accumulated in 40 ms before going into island.\n
                             *This value is 12-bit integer, 4-bit fraction.\n
                             *Client sends as fixed point representation in param payload.
                             *This clock scale factor conditionally applied based on Non Real Time \n
                             * propagated from subgraph downstream to this subgraph\n
                             * and other properties based on usecase \n
                             *Suppose same stream subgraph used for both LPI and non-LPI device.\n
                             *So, enable_duty_cycling set for the subgraph always.\n
                             *This clock scale factor to be used when device is LPI\n
                             *Next clock scale factor to be used when device is non-LPI} */

   uint16_t clock_scale_factor_q4;
   /**< @h2xmle_range 		{0x0000..0xFFFF}
        @h2xmle_default 	{0x10}
        @h2xmle_description {clock scale factors to be set only for non duty cycling subgraphs.	\n
        					 *However, if client sets it for duty cycling subgraphs as well, then
        					 *final clock scaling factor is product of the two factors} */
   uint16_t reserved;
   /**< Used for alignment; must be set to 0. */
}
#include "spf_end_pack.h"
;
typedef struct apm_subgraph_prop_id_clock_scale_factor_t apm_subgraph_prop_id_clock_scale_factor_t;


/*--------------------------------------------------------------------------------------------------------------------*/

#define APM_SUBGRAPH_PROP_ID_BW_SCALE_FACTOR 0x08001531
/** @h2xmlp_property {"Bus Bandwidth Scale Factor", APM_SUBGRAPH_PROP_ID_BW_SCALE_FACTOR}
    @h2xmlp_description {Sub-graph property ID Bus Bandwidth Scale factor}
    @h2xmlp_isVoice     {false} */

#include "spf_begin_pack.h"
struct apm_subgraph_prop_id_bw_scale_factor_t
{
   uint16_t bus_scale_factor_q4;
   /**< @h2xmle_range 		{0x0000..0xFFFF}
        @h2xmle_default 	{0x10}
        @h2xmle_description {bw scale factors to be set for any subgraphs. } */
   uint16_t reserved;
   /**< Used for alignment; must be set to 0. */
}
#include "spf_end_pack.h"
;
typedef struct apm_subgraph_prop_id_bw_scale_factor_t apm_subgraph_prop_id_bw_scale_factor_t;
/*--------------------------------------------------------------------------------------------------------------------*/
/** @} <-- End of subgraph_cfg -->*/

/*====================================================================================================================*/
/*====================================================================================================================*/

/** @ingroup spf_apm_subgraph_props
    Identifier for the parameter that configures the list of sub-graphs.
    This parameter ID must be used with the #APM_CMD_GRAPH_OPEN command.

    @msgpayload
    apm_param_id_sub_graph_list_t \n
    @indent{12pt} apm_sub_graph_id_t
*/
#define APM_PARAM_ID_SUB_GRAPH_LIST             0x08001005


/*# h2xmlp_parameter   {"Sub Graph List", APM_PARAM_ID_SUB_GRAPH_LIST}
    h2xmlp_description {ID for the parameter that configures the sub-graph
                         list. This ID must be used with the
                         APM_CMD_GRAPH_OPEN command.}
    h2xmlp_toolPolicy  {RTC; Calibration} */

/** @ingroup spf_apm_subgraph_props
    Payload for #APM_PARAM_ID_SUB_GRAPH_LIST.

    Immediately following this structure is a variable length array of
    apm_sub_graph_id_t objects that correspond to each sub-graph configuration
    and properties. The length of the array is determined by the number of
    sub-graphs being configured.
*/
#include "spf_begin_pack.h"
struct apm_param_id_sub_graph_list_t
{
   uint32_t num_sub_graphs;
   /**< Number of sub-graphs being configured. */

   /*#< h2xmle_description {Number of sub-graphs being configured.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_param_id_sub_graph_list_t apm_param_id_sub_graph_list_t;


/** @ingroup spf_apm_subgraph_props
    Sub-graph payload for #APM_PARAM_ID_SUB_GRAPH_LIST.
*/
#include "spf_begin_pack.h"
struct apm_sub_graph_id_t
{
   uint32_t sub_graph_id;
   /**< Unique identifier for the sub-graph being configured. */

   /*#< h2xmle_description {Unique identifier for the sub-graph being
                             configured.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_sub_graph_id_t apm_sub_graph_id_t;

/*====================================================================================================================*/
/*====================================================================================================================*/

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* _APM_SUB_GRAPH_API_H_ */
