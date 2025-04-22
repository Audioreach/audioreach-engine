/**
 *   \file offload_sp_api.h
 *   \brief
 *        This file contains API's used for state propagation from Offload container in Master PD and offloaded graph in the
 *  satellite PD.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef OFFLOAD_STATE_PROPAGATION_API_H_
#define OFFLOAD_STATE_PROPAGATION_API_H_

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "ar_defs.h"
#include "ar_guids.h"

/*==============================================================================
   DATA command ID
==============================================================================*/
/**
 * DATA command to communicate the up stream state configuration to the
 * shared memory write end-point module in the satellite graph.
 * This API is used for sending the internal EOS.
 */
#define DATA_CMD_WR_SH_MEM_EP_PEER_PORT_PROPERTY 0x04001007
/*==============================================================================
   Param structure definitions
==============================================================================*/

/** @h2xmlp_parameter   {"DATA_CMD_WR_SH_MEM_EP_PEER_PORT_PREOPERTY", DATA_CMD_WR_SH_MEM_EP_PEER_PORT_PREOPERTY}
    @h2xmlp_description { Data command for setting the upstream state configuration information
                    to the WR_EP module in the satellite graph.\n }
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct data_cmd_wr_sh_mem_ep_peer_port_property_t
{
   uint32_t num_properties;
   /**< @h2xmle_description {determines the number of peer port properties
                             following this payload.}
        @h2xmle_default     {1}
        @h2xmle_range       {0-7FFFFFFF}
        @h2xmle_policy      {Basic} */

   spf_msg_peer_port_property_info_t peer_port_property_payload[0];
   /**< @h2xmle_description {Specifies the peer port property payload }
        @h2xmle_policy      {advanced}
   */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Type definition for the above structure. */
typedef struct data_cmd_wr_sh_mem_ep_peer_port_property_t data_cmd_wr_sh_mem_ep_peer_port_property_t;
/*==============================================================================
   PARAM ID
==============================================================================*/

/**
   Configuration parameter ID to set the peer port property to the WR/RD_EP module in the
   satellite graph from OLC in master PD
*/
#define PARAM_ID_PEER_PORT_PROPERTY_UPDATE 0x080011C3
/*==============================================================================
   Param structure definitions
==============================================================================*/

/** @h2xmlp_parameter   {"PARAM_ID_PEER_PORT_PROPERTY_UPDATE", PARAM_ID_PEER_PORT_PROPERTY_UPDATE}
    @h2xmlp_description {Parameter for setting the up/downstream state configuration information
                   to the WR/RD_EP module in the satellite graph.\n }
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

struct param_id_peer_port_property_t
{
   uint32_t num_properties;
   /**< @h2xmle_description {determines the number of peer port properties
                             following this payload.}
        @h2xmle_default     {0}
        @h2xmle_range       {0-7FFFFFFF}
        @h2xmle_policy      {advanced} */

   spf_msg_peer_port_property_info_t peer_port_property_payload[0];
   /**< @h2xmle_description {Specifies the peer port property payload }
        @h2xmle_policy      {advanced}
   */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
/* Type definition for the above structure. */
typedef struct param_id_peer_port_property_t param_id_peer_port_property_t;

/**
   Configuration parameter ID to set the upstream stopped to the WR_EP module in the
   satellite graph from OLC in master PD
*/
#define PARAM_ID_UPSTREAM_STOPPED 0x080011E5

/* there is no specific payload for the PARAM_ID_UPSTREAM_STOPPED */

/**
 * Event to communicate the downstream state configuration change in the
 * satellite graph to the OLC in the master PD. ie., from WR_EP module in
 * satellite graph to the WR CLIENT module in the OLC
 *
 * Payload state_cfg_event_t
 *
 * Event must be registered with APM_CMD_REGISTER_MODULE_EVENTS
 */
#define OFFLOAD_EVENT_ID_DOWNSTREAM_PEER_PORT_PROPERTY 0x03001005

/**
 * Event to communicate the upstream state configuration change in the
 * satellite graph to the OLC in the master PD. ie., from RD_EP module in
 * satellite graph to the RD CLIENT module in the OLC
 *
 * Payload state_cfg_event_t
 *
 * Event must be registered with APM_CMD_REGISTER_MODULE_EVENTS
 */
#define OFFLOAD_EVENT_ID_UPSTREAM_PEER_PORT_PROPERTY 0x03001006

/**
 * Event to propagate the upstream state configuration change in the
 * satellite graph to the OLC in the master PD in the data path.
 * i.e., from RD_EP module in satellite graph to the RD CLIENT module in the OLC
 *
 * Payload state_cfg_event_t
 *
 * Event must be registered with APM_CMD_REGISTER_MODULE_EVENTS
 */
#define OFFLOAD_DATA_EVENT_ID_UPSTREAM_PEER_PORT_PROPERTY 0x06001003

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/**
 *  Used to indicates downstream state configuration details to the WR CLIENT in OLC.
 *  and to indicates upstream state configuration details to the RD CLIENT in OLC.
 *  Payload is of type state_cfg_event_t
 *
 * Some opcodes that use this payload are:
 * -OFFLOAD_DATA_EVENT_ID_UPSTREAM_PEER_PORT_PROPERTY
 * -OFFLOAD_EVENT_ID_UPSTREAM_PEER_PORT_PROPERTY
 * -OFFLOAD_EVENT_ID_DOWNSTREAM_PEER_PORT_PROPERTY
 */
struct state_cfg_event_t
{
   uint32_t ep_miid;
   /**< @h2xmle_description {module instance ID of the EP module}
        @h2xmle_default     {0}
        @h2xmle_range       {0-0x7FFFFFFF}
        @h2xmle_policy      {advanced} */

   uint32_t num_properties;
   /**< @h2xmle_description {determines the number of peer port properties
                             following this payload.}
        @h2xmle_default     {0}
        @h2xmle_range       {0-7FFFFFFF}
        @h2xmle_policy      {advanced} */

   spf_msg_peer_port_property_info_t peer_port_property_payload[0];
   /**< @h2xmle_description {Specifies the peer port property payload }
        @h2xmle_policy      {advanced}
   */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct state_cfg_event_t state_cfg_event_t;

/**
 * Event to communicate the upstream started/stopped configuration change in the
 * satellite graph to the OLC in the master PD. ie., from RD_EP module in
 * satellite graph to the RD CLIENT module in the OLC
 *
 * Payload upstream_state_cfg_event_t
 *
 * Event must be registered with APM_CMD_REGISTER_MODULE_EVENTS
 */
#define OFFLOAD_EVENT_ID_UPSTREAM_STATE 0x03001007

/**
 *  Param that indicates upstream state configuration details to the RD CLIENT in OLC.
 *  Payload is of type upstream_state_cfg_event_t
 */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
/**
 *  Used to indicates upstream started/stopped state configuration details to the RD CLIENT in OLC.
 *  Payload is of type upstream_state_cfg_event_t
 *
 * Some opcodes that use this payload are:
 * -OFFLOAD_EVENT_ID_UPSTREAM_STATE
 */
struct upstream_state_cfg_event_t
{
   uint32_t ep_miid;
   /**< @h2xmle_description {module instance ID of the EP module}
        @h2xmle_default     {0}
        @h2xmle_range       {0-0x7FFFFFFF}
        @h2xmle_policy      {advanced} */

   uint32_t num_properties;
   /**< @h2xmle_description {determines the number of peer port properties
                             following this payload.}
        @h2xmle_default     {0}
        @h2xmle_range       {0-7FFFFFFF}
        @h2xmle_policy      {advanced} */

   spf_msg_peer_port_property_info_t peer_port_property_payload;
   /**< @h2xmle_description {Specifies the peer port property payload }
        @h2xmle_policy      {advanced}
   */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct upstream_state_cfg_event_t upstream_state_cfg_event_t;

#endif // OFFLOAD_STATE_PROPAGATION_API_H_
