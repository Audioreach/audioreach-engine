/*========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */
/**
@file platform_internal_dcm_if.h

@brief Interface between containers and buffering module to release/request resources
 */

#ifndef PLATFORM_INTERNAL_DCM_IF_H
#define PLATFORM_INTERNAL_DCM_IF_H

#include "spf_utils.h"

/* -----------------------------------------------------------------------
 ** Global definitions/forward declarations
 ** ----------------------------------------------------------------------- */
 #define DCM_CLIENT_NAME_LENGTH 16

/* Information about a client used for registration */
typedef struct dcm_client_info_t
{
   spf_handle_t 			spf_handle; /**< Handle to this container. Must be the first element. q_ptr points to response queue (no longer
                     	 	 	 	 	 supported).*/
   char_t                   client_name[DCM_CLIENT_NAME_LENGTH]; /* Name of client that is registering */
   uint32_t                 client_log_id; /* Container log ID */

} dcm_client_info_t;

typedef enum dcm_island_entry_exit_result_t
{
   ISLAND_ENTRY_ALLOWED,
   ISLAND_EXIT_ALLOWED,
   ISLAND_ENTRY_BLOCKED_GAPLESS
}  dcm_island_entry_exit_result_t;

/**
   This payload structure is used for the following opcodes
     -#DCM_CMD_REGISTER
     -#DCM_CMD_DEREGISTER
   Subgraph will send these commands to PM Server, PM server
   will interact with DCM

   Immediately following this structure contains the signal
   pointer and client info.

   As part of #DCM_CMD_REGISTER,  Duty Cycling Manager (DCM)
   allocates this payloads and register clients with DCM

   As part of #DCM_CMD_DEREGISTER de-registe clients with DCM.

*/
typedef struct dcm_payload_t
{
   posal_signal_t    signal_ptr;
   dcm_client_info_t client_info;
} dcm_payload_t;

#define DCM_CMD_REGISTER (0x01001054)                     /* Register new client with Duty Cycling Manager */
#define DCM_CMD_DEREGISTER (0x01001057)					  /* De-Register new client with Duty Cycling Manager */

/**
   This payload structure is used for the following opcodes
     -#SPF_MSG_CMD_DCM_REQ_FOR_UNBLOCK_ISLAND_ENTRY
     -#SPF_MSG_CMD_DCM_REQ_FOR_ISLAND_EXIT

   Buffering module will send these commands to PM Server,
   PM server will interact with DCM

   Immediately following this structure contains the signal
   pointer and client info.

   As part of #SPF_MSG_CMD_DCM_REQ_FOR_UNBLOCK_ISLAND_ENTRY,
   DCM requests container to unblock itself for island entry


   APM executes #SPF_MSG_CMD_DCM_REQ_FOR_ISLAND_EXIT
    DCM requests container to exit from island.
*/
typedef struct dcm_island_control_payload_t
{
   posal_signal_t signal_ptr;
   uint32_t       dcm_island_change_request;
} dcm_island_control_payload_t;


#define SPF_MSG_CMD_DCM_REQ_FOR_UNBLOCK_ISLAND_ENTRY (0x01001055)	/*DCM request to container for Island Entry */
#define SPF_MSG_CMD_DCM_REQ_FOR_ISLAND_EXIT (0x01001056)	/*DCM request to container for Island Exit */


/**
   This payload structure is used for the following opcodes
     -#DCM_ACK_FROM_CLIENT_FOR_ISLAND_ENTRY

   Container will send this reponse command to PM Server,
   PM Server will interact with DCM

   Immediately following this structure contains client info
   and acknowledgement result.

   Container sends #DCM_ACK_FROM_CLIENT_FOR_ISLAND_ENTRY in response to
   #SPF_MSG_CMD_DCM_REQ_FOR_UNBLOCK_ISLAND_ENTRY for a given DCM enabled
   container.

*/
typedef struct dcm_island_entry_exit_ack_t
{
   dcm_client_info_t              client_info;
   dcm_island_entry_exit_result_t ack_result;
} dcm_island_entry_exit_ack_t;

#define DCM_ACK_FROM_CLIENT_FOR_ISLAND_ENTRY (0x0200100E) /*Client(Cntr) Ack to DCM for Island Entry */



#endif // PLATFORM_INTERNAL_DCM_IF_H
