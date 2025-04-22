#ifndef _PROXY_APM_IF_H_
#define _PROXY_APM_IF_H_

/**
 * \file vcpm_cntr_if.h
 *  
 * \brief
 *     This file defines APM APIs with proxy.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal_types.h"
#include "apm_sub_graph_api.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/********************************************************************************************************/
/*                                          Messages                                                    */
/********************************************************************************************************/

/**
   Payload structure for the following graph management commands from Proxy Manager to APM.
      SPF_MSG_CMD_PROXY_GRAPH_PREPARE
      SPF_MSG_CMD_PROXY_GRAPH_START
      SPF_MSG_CMD_PROXY_GRAPH_STOP
**/
typedef struct spf_msg_cmd_proxy_graph_mgmt_t spf_msg_cmd_proxy_graph_mgmt_t;

struct spf_msg_cmd_proxy_graph_mgmt_t
{
   uint32_t      num_sgs;
   /**< Number of Sub graphs to be started. */

   apm_sub_graph_id_t *sg_array_ptr;
   /** Pointer to array of subgraph IDs,
       sent from proxy manager for command processing.*/
	   
   bool_t             is_direct_cmd;
   /**< TRUE: Command is being sent directly and not in 
        response to any of the APM cmd
        FALSE: Cmd is sent in response to APM cmd */
};

/** APM uses this command to send the Proxy Graph information to the Proxy Manager. */
#define SPF_MSG_CMD_PROXY_GRAPH_INFO 0x0100103B

/** Proxy Manager sends this command to APM to Prepare the proxy subgraphs. */
#define SPF_MSG_CMD_PROXY_GRAPH_PREPARE 0x0100103C

/** Proxy Manager sends this command to APM to start the proxy subgraphs. */
#define SPF_MSG_CMD_PROXY_GRAPH_START 0x0100103D

/** Proxy Manager sends this command to APM to stop the proxy subgraphs. */
#define SPF_MSG_CMD_PROXY_GRAPH_STOP 0x0100103E


/**
   Payload structure for SPF_MSG_RSP_PROXY_MGMT_PERMISSION message.
   Proxy Manger responds to APM with this payload when no error in command handling.
   APM shall operate on corresponding Proxy Manager subgraphs,
   according to the "proxy_permission_granted" value.
**/
typedef struct spf_msg_rsp_proxy_permission_t spf_msg_rsp_proxy_permission_t;

struct spf_msg_rsp_proxy_permission_t
{

   uint32_t num_permitted_subgraphs;
   /** Number of Subgraphs permitted by Proxy Manager
       for command processing. */

   apm_sub_graph_id_t *permitted_sg_array_ptr;
   /** Pointer to array of subgraph IDs,
       permitted by proxy manager for command processing.*/
};

/** Permission response from proxy Manager for graph management command from APM.*/
#define SPF_MSG_RSP_PROXY_MGMT_PERMISSION 0x0200100B

/** Event from VCPM to VCPM, indicating that clock vote is changed.*/
#define SPF_EVT_ID_CLOCK_VOTE_CHANGE 0x0300100B

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _PROXY_APM_IF_H_
