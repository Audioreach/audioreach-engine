#ifndef _APM_DEBUG_INFO_H_
#define _APM_DEBUG_INFO_H_
/**
 * \file apm_debug_info.h
 * \brief
 *     This file contains APM sub-graph related commands and events data structures definitions
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "media_fmt_api_basic.h"
#include "ar_defs.h"
#include "apm_graph_properties.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/


/*====================================================================================================================*/
/*====================================================================================================================*/
//#ifdef __H2XML__
/** @ingroup spf_apm_subgraph_props
    Payload for #APM_PARAM_ID_SUB_GRAPH_STATE.

   Immediately following this are the number of sub graphs and structure apm_sub_graph_id_state_t
   repeats as many times as num_sub_graphs
   This param can be only be used with APM_CMD_SET_CFG and not with APM_CMD_GET_CFG
*/
#define APM_PARAM_ID_SG_STATE_REPORT_CFG             0x08001506
/*# h2xmlp_parameter   {"Debug Info Configuration",
                         APM_PARAM_ID_SG_STATE_REPORT_CFG}
    h2xmlp_description {ID for the parameter that configures the container
                        properties.}
    h2xmlp_toolPolicy  {RTM} */
#include "spf_begin_pack.h"
struct apm_param_id_sub_graph_state_t
{
   uint32_t is_sg_state_report_cfg_enabled;
   /**< SG state report enabled or disabled */

   /*#< h2xmle_description {Number of sub-graphs being configured.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_param_id_sub_graph_state_t apm_param_id_sub_graph_state_t;


/** @ingroup spf_apm_subgraph_props
    Sub-graph payload for #APM_PARAM_ID_SUB_GRAPH_STATE.
*/

/** @ingroup spf_apm_sub_graph_states
   APM Sub Graph States*/
#define APM_SG_STATE_INVALID  0
   /**< Sub-graph state INVALID */
/** @ingroup spf_apm_sub_graph_states
   APM Sub Graph States*/
#define APM_SG_STATE_STOPPED 1
   /**< Sub-graph state STOPPED */
/** @ingroup spf_apm_sub_graph_states
   APM Sub Graph States*/
#define APM_SG_STATE_SUSPENDED 2
   /**< Sub-graph state SUSPENDED */
/** @ingroup spf_apm_sub_graph_states
   APM Sub Graph States*/
#define APM_SG_STATE_PREPARED 3
   /**< Sub-graph state PREPARED */
/** @ingroup spf_apm_sub_graph_states
   APM Sub Graph States*/
#define APM_SG_STATE_STARTED 4
   /**< Sub-graph state STARTED */

#include "spf_begin_pack.h"
struct apm_num_sub_graphs_state_t

{
   uint32_t num_of_sub_graphs;
   /**< Unique identifier for the sub-graph being queried. */

   /*#< h2xmle_description {Unique identifier for number of sub-graph being
                             reported.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

}
#include "spf_end_pack.h"
;
typedef struct apm_num_sub_graphs_state_t apm_num_sub_graphs_state_t;

#include "spf_begin_pack.h"
struct apm_sub_graph_id_state_t

{
   uint32_t sub_graph_id;
   /**< Unique identifier for the sub-graph being queried. */

   /*#< h2xmle_description {Unique identifier for the sub-graph being
                             queried.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t state;
   /**< Unique identifier for the sub-graph state.

      Supports the following states
      APM_SG_STATE_INVALID,

      APM_SG_STATE_STOPPED,

      APM_SG_STATE_SUSPENDED,

      APM_SG_STATE_PREPARED,

      APM_SG_STATE_STARTED */

   /*#< h2xmle_description {Unique identifier for the sub-graph being
                             configured.}
        h2xmle_range       {{0x0000..0xFFFF}
        h2xmle_default     {0} */


}
#include "spf_end_pack.h"
;
typedef struct apm_sub_graph_id_state_t apm_sub_graph_id_state_t;

/** @ingroup spf_apm_port_media_format_props
    Payload for #APM_PARAM_ID_PORT_MEDIA_FMT_REPORT_CFG**/
#define APM_PARAM_ID_PORT_MEDIA_FMT_REPORT_CFG 0x08001507

/*< This param can be only be used with APM_CMD_SET_CFG */

/** All container cfg should be contiguous for optimality*/

#include "spf_begin_pack.h"

struct apm_param_id_port_media_fmt_report_cfg_enable_t
{
   uint32_t is_port_media_fmt_report_cfg_enabled;

   /**< port media fmt report cfg enable/disable */
   /*#< h2xmle_description {port media fmt report cfg enable/disable.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct apm_param_id_port_media_fmt_report_cfg_enable_t apm_param_id_port_media_fmt_report_cfg_enable_t;

#include "spf_begin_pack.h"

struct apm_param_id_port_media_fmt_list_t
{
   uint32_t container_id;
   /*#< h2xmle_description {Unique identifier for the container being
                             queried.}
        h2xmle_range       {0..0xFFFFFFFF}
        h2xmle_default     {0} */

   uint32_t num_port_media_format;
   /*#< h2xmle_description {the number of ports.}
        h2xmle_range       {{0x0000..0xFFFF}
        h2xmle_default     {0} */

   /**< number of apm_param_id_port_media_format_t structs */

}
#include "spf_end_pack.h"
;
typedef struct apm_param_id_port_media_fmt_list_t apm_param_id_port_media_fmt_list_t;

#include "spf_begin_pack.h"

/**apm_param_id_port_media_fmt_list_t
   The below structures exist depending on the data_format
*/
struct apm_pcm_pack_med_fmt_t
{
   uint32_t sample_rate;
   /*< sampling rate ranging from 8k to 384k defined in media_fmt_api_basic */

   uint8_t bit_width;
   /**< 16, 24, 32 (actual bit width independent of the word size)*/

   uint8_t bits_per_sample;
   /**< bits per sample 16 or 32. This is actually word size in bits*/

   uint8_t q_factor;
   /**< 15, 27, 31 */

   uint8_t num_channels;
   /*< number of channels ranges from 0 to 32*/

   uint8_t interleaving;
   /*< interleaving from media_fmt_api_basic.h
    * 	Supported Values
    * 	INVALID_VALUE
    * 	PCM_INTERLEAVED
    * 	PCM_DEINTERLEAVED_PACKED
    * 	PCM_DEINTERLEAVED_UNPACKED */

   uint8_t endianness;
   /*< endianness from media_fmt_api_basic.h
    * 	Supported Values
    * 	 INVALID_VALUE
    * 	 PCM_LITTLE_ENDIAN
    * 	 PCM_BIG_ENDIAN
    */
#ifdef __H2XML__
   uint8_t channel_map[0];
/**< @h2xmle_description {   Sub structure containing per channel mapping. Each value can be uint8_t.}
     @h2xmle_range       { 0..63}
     @h2xmle_variableArraySize  { "num_channels" } */
#endif
}
#include "spf_end_pack.h"
;
typedef struct apm_pcm_pack_med_fmt_t apm_pcm_pack_med_fmt_t;

#include "spf_begin_pack.h"

struct apm_channel_mask_t
{
   uint32_t channel_mask_lsw;
   /**< lsw of the channel mask */
   uint32_t channel_mask_msw;
   /**< msw of the channel mask */
}
#include "spf_end_pack.h"
;
typedef struct apm_channel_mask_t apm_channel_mask_t;

#include "spf_begin_pack.h"

struct apm_deint_raw_med_fmt_t
{
   uint32_t num_buffers;
/*< num of buffers */

#ifdef __H2XML__
   apm_channel_mask_t channel_mask[0];
   /**< @h2xmle_description {   Sub structure containing per channel mask . channel mapping can be L,C,R. Each channel mask is of type apm_channel_mask_t .}
        @h2xmle_variableArraySize  { "num_buffers" } */
#endif
}
#include "spf_end_pack.h"
;
typedef struct apm_deint_raw_med_fmt_t apm_deint_raw_med_fmt_t;

/**different ports of a module should be contiguous for optimality.*/

#include "spf_begin_pack.h"

struct apm_media_format_t
{

   uint32_t data_format;
   /*< data format from the macros defined in media_fmt_api_basic.h
    *
    @values
        *  INVALID_VALUE
        *  DATA_FORMAT_FIXED_POINT
        *  DATA_FORMAT_IEC61937_PACKETIZED
        *  DATA_FORMAT_IEC60958_PACKETIZED
        *  DATA_FORMAT_DSD_OVER_PCM
        *  DATA_FORMAT_GENERIC_COMPRESSED
        *  DATA_FORMAT_RAW_COMPRESSED
        *  DATA_FORMAT_COMPR_OVER_PCM_PACKETIZED
        *  DATA_FORMAT_IEC60958_PACKETIZED_NON_LINEAR
    *  */

   uint32_t fmt_id;
/**< Media format ID of the data stream.

  @valuesbul
  - #INVALID_VALUE (Default)
  - #MEDIA_FMT_ID_PCM @tablebulletend */

#ifdef __H2XML__
   union
   {
      apm_pcm_pack_med_fmt_t pcm;
      /**< when PACKETIZED_OR_PCM(data_fmt is true)*/

      apm_deint_raw_med_fmt_t deint_raw;
      /**< for DEINTERLEAVED_RAW_COMPRESSED */
   };
#endif
}
#include "spf_end_pack.h"
;
typedef struct apm_media_format_t apm_media_format_t;

#include "spf_begin_pack.h"

struct apm_param_id_port_media_format_t
{
   uint32_t module_instance_id;
   /**< Instance ID of the module */

   uint32_t port_id;
   /**< port ID of the module being queried. */
#ifdef __H2XML__
   apm_media_format_t num_port_media_format[];
   /*#< @h2xmle_description {number of port media format .}
        @h2xmle_default     {0}
        @h2xmle_policy      {Basic} */
#endif
}
#include "spf_end_pack.h"
;
typedef struct apm_param_id_port_media_format_t apm_param_id_port_media_format_t;
/*====================================================================================================================*/
/*====================================================================================================================*/
/*--------------------------------------------------------------------------------------------------------------------*/
//#endif /* __H2XML__*/
#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_DEBUG_INFO_H_ */
