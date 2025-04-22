#ifndef _AMDB_THREAD_I_H_
#define _AMDB_THREAD_I_H_
/**
 * \file amdb_thread_i.h
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal.h"
#include "spf_cmn_if.h"
#include "spf_list_utils.h"
#include "spf_sys_util.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
/****************************************************************************
 * Macro Declarations
 ****************************************************************************/
#define AMDB_MAX_CMD_Q_ELEMENTS 8
#define AMDB_CMD_Q_BYTE_SIZE (POSAL_QUEUE_GET_REQUIRED_BYTES(AMDB_MAX_CMD_Q_ELEMENTS))
#define AMDB_MAX_RSP_Q_ELEMENTS 16
#define AMDB_RSP_Q_BYTE_SIZE (POSAL_QUEUE_GET_REQUIRED_BYTES(AMDB_MAX_RSP_Q_ELEMENTS))
#define AMDB_MSG_PREFIX "AMDB:"
/****************************************************************************
 * Enumerations
 ****************************************************************************/

/****************************************************************************
 * Structure Declarations
 ****************************************************************************/

/** PRM Module Struct */
typedef struct amdb_thread_t amdb_thread_t;

struct amdb_thread_t
{
   spf_handle_t           amdb_handle;          /**< AMDB thread handle */
   spf_cmd_handle_t       amdb_cmd_handle;      /**< AMDB thread command handle */
   uint32_t               amdb_cmd_q_wait_mask; /**< Signal wait mask for cmd Q */
   uint32_t               amdb_rsp_q_wait_mask; /**< Signal wait mask for cmd Q */
   posal_channel_t        cmd_channel;          /**< Channel for amd/rsp/kill signal */
   posal_signal_t         kill_signal;          /**< Signal to destroy AMDB module thread */
   posal_queue_t *        amdb_cmd_q_ptr;       /**< AMDB Command queue */
   posal_queue_t *        amdb_rsp_q_ptr;       /**< AMDB Command queue */
   uint32_t               gpr_handle;           /**< GPR handle */
   bool_t                 thread_launched;      /**< if the thread launched */
   POSAL_HEAP_ID          heap_id;              /**< heap id */
   spf_sys_util_handle_t *sys_util_handle_ptr;  /**< Sys util handle */
   spf_list_node_t *      cmd_ctrl_list;        /**< List of commands under process (obj type: amdb_cmd_ctrl_t).
                                                    The following is to store the cmd ctx based on token.
                                                    This is only used on the Master DSP*/
};

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* _AMDB_THREAD_I_H_ */
