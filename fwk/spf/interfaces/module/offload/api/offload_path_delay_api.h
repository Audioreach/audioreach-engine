/**
 *   \file offload_path_delay_api.h
 *   \brief
 *        This file contains API's which are extended for path delay implementation
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef OFFLOAD_PATH_DELAY_API_H_
#define OFFLOAD_PATH_DELAY_API_H_

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "ar_defs.h"
#include "ar_guids.h"

/**
 * Parameter ID for getting path delay from Satellite APM
 *
 * The graphs corresponding to the modules mentioned in the
 * payload of this command must be opened using #APM_CMD_GRAPH_OPEN
 *
 * This param is used with APM_CMD_GET_CFG.
 * Usually APM_CMD_GET_CFG doesn't carry any parameter specific payload, however,
 * this command contains payload to specify the path.
 *
 * For the delay to be accurate calibration should already be done for the path.
 * Media format must have been propagated. Typically, all subgraphs must be in PREPARE state.
 *
 * Maximum possible delay from source to destination is returned.
 * In some cases, delay may not be exactly same as returned. For example, in FTRT paths, or
 * processing time is smaller than expected. Also, delay can be obtained only for PCM or packetized data paths.
 * If raw compressed data is involved, then delay cannot be known correctly for the portion that's compressed.
 * Query however, won't fail as delay of the rest of the path (if any) can be determined.
 *
 * Under split-merge scenarios, if there are multiple paths from source to destination,
 * first path will be chosen arbitrarily.
 *
 * If path is not complete, error is returned.
 * Delay is >= 0.
 *
 * Client can leave the port-ids zeros to be autofilled by spf.
 * When port-ids are zero src and dst module's delay may or may not be included as explained below
 *
 *  -  If src has only outputs and max input ports = 0, then it's delay is included.
 *       Since src has no input ports, this is the only way to account for its delay.
 *  -  If dst has only input ports and max output ports = 0, then it's delay is included.
 *       Since dst has no output ports, this is the only way to account for its delay.
 *  -  For all other cases, delay is from the output port of src to output port of dst.
 *       This way of defining helps in concatenation. E.g.,  A -> B -> C can be obtained as A to B + B to C
 *       and delay of B is not considered twice.
 *       When dst has multiple outputs, an output will be selected arbitrarily.
 *
 * Payload struct -- apm_module_event_t
 *
 */
#define APM_PARAM_ID_OFFLOAD_GRAPH_PATH_DELAY 0x08001206

#include "spf_begin_pack.h"
/**
 * This struct defines the start and end of path for which delay is queried.
 *
 */
struct apm_offload_graph_path_defn_for_delay_t
{
   uint32_t src_module_instance_id;
   /**< Source module instance ID
    * input parameter. */
   uint32_t src_port_id;
   /**< Source port-ID. Set to zero if port-id is not known.
    * Knowing port-id helps reduce the complexity of graph search.
    * Also, if specific path delays are needed in split-merge scenarios, port-id helps
    * (E.g. module A's port 1 & 2 ends up in port 1 & 2 of end point B. Since there are 2 paths from A to B
    * having port-id helps).
    *
    *
    * If port-id is zero at the time of query, it will be populated with proper port-id when APM responds.
    * This is an input & output parameter. */
   uint32_t dst_module_instance_id;
   /**< Destination module instance ID.
    * This is an input parameter. */
   uint32_t dst_port_id;
   /**< Destination port-ID. Set to zero if port-id is not known.
    * If port-id is zero at the time of query, it will be populated with proper port-id when APM responds.
    * This is an input & output parameter. */
   uint32_t delay_us;
   /**< Microsecond delay
    * This is an output parameter.*/

   uint32_t is_client_query;
   /**< This field would specify if the query is from the HLOS or from an internal module.
    *   supported values
    *   1 - indicates that the query is from the client. This is implies that the query is one-time request.
    *   0 - indicates that the query is based on the request for delay from a module.
    *       This implies that the query is persistent.
    */

   uint32_t get_sat_path_id;
   /**< The satellite APM needs to update the path ID assigned in the response payload
    */
}
#include "spf_end_pack.h"
;

typedef struct apm_offload_graph_path_defn_for_delay_t apm_offload_graph_path_defn_for_delay_t;

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/**
 * defines the main payload for the APM_PARAM_ID_OFFLOAD_GRAPH_PATH_DELAY query
 */
struct apm_param_id_offload_graph_path_delay_t
{
   uint32_t num_paths;
   /** number of paths to be queried */

   apm_offload_graph_path_defn_for_delay_t paths[0];
   /** definition of each path */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

typedef struct apm_param_id_offload_graph_path_delay_t apm_param_id_offload_graph_path_delay_t;

/**
 * Event to communicate the satellite container delay to the OLC in the master
 * process domain. OLC will register all the containers which are part of a
 * path for which the get path delay query is initiated.
 *
 * Payload get_container_delay_event_t
 *
 * Event must be registered with APM_CMD_REGISTER_MODULE_EVENTS
 */
#define OFFLOAD_EVENT_ID_GET_CONTAINER_DELAY 0x03001008

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/**
 *  Used to communicate the satellite container delay to the OLC.
 *  Payload is of type state_cfg_event_t
 *
 * Some opcodes that use this payload are:
 * -OFFLOAD_EVENT_ID_GET_CONTAINER_DELAY
 */
struct get_container_delay_event_t
{
   uint32_t prev_delay_in_us;
   /**< @h2xmle_description {specifies the previous delay in us}
        @h2xmle_default     {0}
        @h2xmle_range       {0-0x7FFFFFFF}
        @h2xmle_policy      {advanced} */

   uint32_t new_delay_in_us;
   /**< @h2xmle_description {specifies the new delay in us}
        @h2xmle_default     {0}
        @h2xmle_range       {0-0x7FFFFFFF}
        @h2xmle_policy      {advanced} */

   uint32_t path_id;
   /**< @h2xmle_description {path id for which the delay is updated}
        @h2xmle_default     {0}
        @h2xmle_range       {0-7FFFFFFF}
        @h2xmle_policy      {advanced} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct get_container_delay_event_t get_container_delay_event_t;

#endif // OFFLOAD_PATH_DELAY_API_H_
