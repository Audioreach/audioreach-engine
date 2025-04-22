/**
 * \file apm_cmd_sequencer.h
 * \brief
 *     This file contains APM Command Sequencer Data structure and handlers
 *     declaration
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _APM_CMD_SEQUENCER_H_
#define _APM_CMD_SEQUENCER_H_

/*==========================================================================
  Include files
  ========================================================================== */

#include "apm_i.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/* clang-format off */

/****************************************************************************
 * Structure Declarations
 ****************************************************************************/
/* this is defined in apm_msg_utils, but is needed here*/
typedef struct apm_cont_msg_opcode_t  apm_cont_msg_opcode_t;

enum
{
   /** Common operation enums across different commands */
   APM_CMN_CMD_OP_HANDLE_FAILURE = 0x4000,
   APM_CMN_CMD_OP_COMPLETED = 0x5000,

   /** Invalid operation enum  */
   APM_CMN_CMD_OP_INVALID = 0xFFFFFFFF,

   /** Enum for enforcing enum size to be atleast 4 bytes   */
   APM_CMN_CMD_OP_ENUM_SIZE = APM_CMN_CMD_OP_INVALID
};

/** Enumeration for operations performed as part of
 *  APM_CMD_GRAPH_OPEN */
enum
{
   /** Command operation enum list for normal processing */
   APM_OPEN_CMD_OP_SEND_CONT_OPEN_MSG = 0,
   APM_OPEN_CMD_OP_HDL_DATA_PATHS,
   APM_OPEN_CMD_OP_PROXY_MGR_OPEN,
   APM_OPEN_CMD_PROXY_MGR_PREPROCESS,
   APM_OPEN_CMD_OP_PROXY_MGR_CFG,
   APM_OPEN_CMD_OP_HDL_LINK_OPEN_INFO,
   APM_OPEN_CMD_OP_HDL_LINK_START,
   APM_OPEN_CMD_OP_HDL_DB_QUERY_PREPROCESS,
   APM_OPEN_CMD_OP_HDL_DB_QUERY_SEND_INFO,
   APM_OPEN_CMD_OP_COMPLETED,
   APM_OPEN_CMD_OP_MAX = APM_OPEN_CMD_OP_COMPLETED,

   /** Command operation enum for command error handling  */
   APM_OPEN_CMD_OP_ERR_HDLR = APM_CMN_CMD_OP_HANDLE_FAILURE,
   APM_OPEN_CMD_OP_HDL_CREATE_FAIL,
   APM_OPEN_CMD_OP_PREPPROC_OPEN_CONNECT_FAIL,
   APM_OPEN_CMD_HDLR_CLOSE_SG_LIST,
   APM_OPEN_CMD_OP_ERR_HDLR_COMPLETED,

   /** Enum value to enforce enum size to be 4-bytes */
   APM_OPEN_CMD_OP_ENUM_SIZE = APM_CMN_CMD_OP_ENUM_SIZE,

   /** Enum for invalid command operation */
   APM_OPEN_CMD_OP_INVALID = APM_CMN_CMD_OP_INVALID

};

/** Enumeration for operations performed as part of
 *  all the graph management commands e.g. graph START, STOP
 *  etc */
enum
{
   /** Command operation enum list for normal processing */
   APM_GM_CMD_OP_PRE_PROCESS = 0,
   APM_GM_CMD_OP_DB_QUERY_SEND_INFO,
   APM_GM_CMD_OP_PROCESS_REG_GRAPH,
   APM_GM_CMD_OP_PROCESS_PROXY_GRAPH,
   APM_GM_CMD_OP_CMN_HDLR,
   APM_GM_CMD_OP_HDL_DATA_PATHS,
   APM_GM_CMD_OP_COMPLETED,
   APM_GM_CMD_OP_MAX = APM_GM_CMD_OP_COMPLETED,

   /** Command operation enum for command error handling  */
   APM_GM_CMD_OP_HANDLE_FAILURE = APM_CMN_CMD_OP_HANDLE_FAILURE,

   /** Enum value to enforce enum size to be 4-bytes */
   APM_GM_CMD_OP_ENUM_SIZE = APM_CMN_CMD_OP_ENUM_SIZE,

   /** Enum for invalid command operation */
   APM_GM_CMD_OP_INVALID = APM_CMN_CMD_OP_INVALID

};

/** Enumeration for operations performed as part of
 *  close all command */
enum
{
   /** Command operation enum list for normal processing */
   APM_CLOSE_ALL_CMD_OP_PRE_PROCESS,
   APM_CLOSE_ALL_CMD_OP_CLOSE_SG,
   APM_CLOSE_ALL_CMD_OP_SAT_CLOSE,
   APM_CLOSE_ALL_CMD_OP_POST_PROCESS,
   APM_CLOSE_ALL_CMD_OP_COMPLETED,
   APM_CLOSE_ALL_CMD_OP_MAX = APM_CLOSE_ALL_CMD_OP_COMPLETED,

   /** Command operation enum for command error handling  */
   APM_CLOSE_ALL_CMD_OP_HANDLE_FAILURE = APM_CMN_CMD_OP_HANDLE_FAILURE,

   /** Enum value to enforce enum size to be 4-bytes */
   APM_CLOSE_ALL_CMD_OP_ENUM_SIZE = APM_CMN_CMD_OP_ENUM_SIZE,

   /** Enum for invalid command operation */
   APM_CLOSE_ALL_CMD_OP_INVALID = APM_CMN_CMD_OP_INVALID

};

/** Enumeration for operations performed as part of
 *  set/get config command */
enum
{
   /** Command operation enum list for normal processing */
   APM_SET_GET_CFG_CMD_OP_SEND_CONT_MSG = 0,
   APM_SET_GET_CFG_CMD_OP_SEND_PROXY_MGR_MSG,
   APM_SET_GET_CFG_CMD_OP_HDL_DBG_INFO,
   APM_SET_GET_CFG_CMD_OP_CLOSE_ALL,
   APM_SET_GET_CFG_CMD_OP_COMPLETED,
   APM_SET_GET_CFG_CMD_OP_MAX = APM_SET_GET_CFG_CMD_OP_COMPLETED,

   /** Command operation enum for command error handling  */
   APM_SET_GET_CFG_CMD_OP_HANDLE_FAILURE = APM_CMN_CMD_OP_HANDLE_FAILURE,

   /** Enum value to enforce enum size to be 4-bytes */
   APM_SET_GET_CFG_CMD_OP_ENUM_SIZE = APM_CMN_CMD_OP_ENUM_SIZE,

   /** Enum for invalid command operation */
   APM_SET_GET_CFG_CMD_OP_INVALID = APM_CMN_CMD_OP_INVALID

};

typedef enum apm_cmd_seq_idx
{
   /** Invalid command operation sequence */
   APM_CMD_SEQ_IDX_INVALID = 0,

   /** Common command operation sequences */
   APM_SEQ_SET_UP_CONT_MSG_SEQ = 1,
   APM_SEQ_SEND_MSG_TO_CONTAINERS,
   APM_SEQ_CONT_SEND_MSG_COMPLETED,

   /** Open command operation sequences */
   APM_SEQ_RECONFIG_DATA_PATHS,

   /** Graph Management command operation sequences */
   APM_SEQ_VALIDATE_SG_LIST,
   APM_SEQ_PREPROC_GRAPH_MGMT_MSG,
   APM_SEQ_PREPARE_FOR_NEXT_CONT_MSG,
   APM_SEQ_REG_SG_PROC_COMPLETED,

   /** Proxy Manager related sequences done as part of APM GRAPH
    *  OPEN, Graph management, set/get config etc. commands  */
   APM_SEQ_SET_UP_PROXY_MGR_MSG_SEQ,
   APM_SEQ_SEND_OPEN_CMD_TO_PROXY_MGR,
   APM_SEQ_SEND_CFG_CMD_TO_PROXY_MGR,
   APM_SEQ_SEND_MSG_TO_PROXY_MGR_COMPLETED,
   APM_SEQ_SEEK_PROXY_MGR_PERMISSION,
   APM_SEQ_PROCESS_PROXY_MGR_SUB_GRAPHS,
   APM_SEQ_PROXY_MGR_SG_PROC_COMPLETED,

   APM_SEQ_DESTROY_DATA_PATHS,

   /** Max to force enum size uint32_t */
   APM_CMD_SEQ_IDX_MAX = 0xFFFFFFFF

} apm_cmd_seq_idx_t;

typedef struct apm_op_seq_t apm_op_seq_t;

struct apm_op_seq_t
{
   uint32_t    op_idx;
   /**< Operation index */

   uint32_t    curr_seq_idx;
   /**< Current seq index for op_idx */

   bool_t      curr_cmd_op_pending;
   /**< Flag to indicate if the processing of current
        operation is pending */

   ar_result_t status;
   /**< Current status of this op */
};

/**
 * Function pointer for the cmd/rsp queue handlers
 */
typedef ar_result_t (*apm_cmd_seq_entry_func_t)(apm_t *apm_info_ptr);

typedef struct apm_cmd_seq_info_t apm_cmd_seq_info_t;

struct apm_cmd_seq_info_t
{
   apm_op_seq_t              graph_open_seq;
      /** Graph open sequencer */

   apm_op_seq_t              set_get_cfg_cmd_seq;
   /** Set get config sequencer  */

   apm_op_seq_t              graph_mgmt_seq;
   /** Graph mgmt sequencer */

   apm_op_seq_t              err_hdlr_seq;
   /** Error handler sequencer */

   apm_op_seq_t              close_all_seq;
   /** Close all sequencer  */

   apm_op_seq_t             *pri_op_seq_ptr;
   /**< Primary op sequencer pointer 
        for the command in process */

   apm_op_seq_t             *curr_op_seq_ptr;
   /**< Current op sequencer pointer 
        for the command in process */
   
   apm_op_seq_t             *parent_op_seq_ptr;
   /**< Parent op sequencer pointer
        for the command in process */

   apm_cmd_seq_entry_func_t  cmd_seq_entry_fptr;
   /**< Entry point function for the commmand sequencer
        corresponding to current client command under
        process */
};

/* clang-format on */

/****************************************************************************
 * Function Declarations
 ****************************************************************************/
/**
 * End the current operating sequence.
 *
 * @param[in] apm_info_ptr: Pointer to APM global info object
 * @param[in] apm_op_seq_t: Current sequence ptr
 *
 * @return return code
 */
void apm_end_cmd_op_sequencer(apm_cmd_ctrl_t *cmd_ctrl_ptr, apm_op_seq_t *curr_op_seq_ptr);

/**
 * Clear the current sequence cmd operation pending status.
 *
 * @param[in] apm_op_seq_t: Current sequence ptr
 * @return return code
 */
void apm_clear_curr_cmd_op_pending_status(apm_op_seq_t *op_seq_ptr);

/**
 * Init the cmd sequence into.
 *
 * @param[in] apm_info_ptr: Pointer to APM global info object
 * @param[in] apm_cmd_seq_entry_func_t: Sequence function ptr
 * @param[in] apm_op_seq_t: Primary sequence ptr
 * @return return code
 */
void apm_init_cmd_seq_info(apm_cmd_ctrl_t *         cmd_ctrl_ptr,
                           apm_cmd_seq_entry_func_t seq_fptr,
                           apm_op_seq_t *           prim_op_seq_ptr);

/**
 * Set the sequencer for the current client command under process.
 *
 * @param[in] apm_info_ptr: Pointer to APM global info object
 *
 * @return return code
 */
ar_result_t apm_set_cmd_seq_func(apm_t *apm_info_ptr);

/**
 * Common entry point function for all the command sequencers
 *
 * @param[in] apm_info_ptr: Pointer to APM global info object
 *
 * @return return code
 */
ar_result_t apm_cmd_sequencer_cmn_entry(apm_t *apm_info_ptr);

/**
 * Common entry point function for all the command sequencers
 *
 * @param[in] apm_info_ptr: Pointer to APM global info object
 *
 * @return return code
 */
ar_result_t apm_cmd_graph_mgmt_cmn_sequencer(apm_t *apm_info_ptr);

/**
 * Common entry point function for all the command sequencers
 *
 * @param[in] apm_info_ptr: Pointer to APM global info object
 *
 * @return return code
 */
ar_result_t apm_cmd_graph_mgmt_sequencer(apm_t *apm_info_ptr);

/**
 * Common entry point function for set get sequencer
 *
 * @param[in] apm_info_ptr: Pointer to APM global info object
 *
 * @return return code
 */
ar_result_t apm_cmd_set_get_cfg_sequencer(apm_t *apm_info_ptr);

/**
 * Common entry point function for all the command sequencers
 *
 * @param[in] apm_info_ptr: Pointer to APM global info object
 *
 * @return return code
 */
void apm_abort_cmd_op_sequencer(apm_cmd_ctrl_t *cmd_ctrl_ptr);

/**
 * Common entry point function for all the command sequencers
 *
 * @param[in] apm_info_ptr: Pointer to APM global info object
 *
 * @return return code
 */
ar_result_t apm_cmd_cmn_sequencer(apm_t *apm_info_ptr);

/**
 * Function to set up error handling
 *
 * @param[in] apm_info_ptr: Pointer to APM global info object
 *
 * @return return code
 */
ar_result_t apm_cmd_seq_set_up_err_hdlr(apm_cmd_ctrl_t *cmd_ctrl_ptr);

/**
 * Common entry point function for all the command sequencers
 *
 * @param[in] apm_info_ptr: Pointer to APM global info object
 *
 * @return return code
 */
ar_result_t apm_populate_cont_graph_mgmt_cmd_seq(apm_t *apm_info_ptr);

ar_result_t apm_populate_graph_close_proc_seq(apm_cmd_ctrl_t *       apm_cmd_ctrl_ptr,
                                              apm_sub_graph_state_t  sub_graph_list_state,
                                              apm_cont_msg_opcode_t *cont_msg_opcode_ptr);
#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /*  _APM_CMD_SEQUENCER_H_ */
