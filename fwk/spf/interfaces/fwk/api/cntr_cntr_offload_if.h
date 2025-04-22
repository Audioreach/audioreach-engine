#ifndef _SPF_MSG_OFFLOAD_DEF_H_
#define _SPF_MSG_OFFLOAD_DEF_H_

/**
 * \file spf_msg_offload_def.h
 * \brief
 *    This file contains message opcodes and their payload definitions
 *  specifically for communication across processors
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*-------------------------------------------------------------------------
Include Files
-------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**< GUID for the GPR command used to send a Triggered IMCL message
across proc domains. Payload points to IMCL msg.*/
#define IMCL_INTER_PROC_TRIGGER_MSG_GPR 0x03001001

/**< GUID for the GPR command used to send a Polled IMCL message
across proc domains. Payload points to IMCL msg.*/
#define IMCL_INTER_PROC_POLLING_MSG_GPR 0x03001002

/**< GUID for the GPR command used to inform the PEER CTRL PORT STATE
across proc domains. Upon receiving this cmd, the containers know
that the imcl peer of this has changed.*/
#define IMCL_INTER_PROC_PEER_STATE_UPDATE 0x03001003

typedef struct imcl_inter_proc_peer_state_update_t
{
   uint32_t ctrl_port_id;
   /*ctrl port id of the port on the
   container that receives this cmd.
   This indicates that the imcl peer
   of this port is updated.*/

   uint32_t sg_ops;
   /*peer's sg operation that caused
   a state change*/

} imcl_inter_proc_peer_state_update_t;

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef _SPF_MSG_OFFLOAD_DEF_H_