/**
 * \file amdb_offload_utils.c
 * \brief
 *     This file contains offload related utilities for AMDB
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/****************************************************************************
 * INCLUDE HEADER FILES                                                     *
 ****************************************************************************/

#include "amdb_offload_utils.h"
#include "amdb_api.h"
#include "apm_offload_mem.h"
#include "spf_hashtable.h"

ar_result_t amdb_set_cmd_ctrl(amdb_thread_t *   amdb_info_ptr,
                              spf_msg_t *       msg_ptr,
                              void *            master_payload_ptr,
                              bool_t            is_out_of_band,
                              amdb_cmd_ctrl_t **curr_cmd_ctrl_pptr,
                              uint32_t          dst_domain_id)
{
   ar_result_t result = AR_EUNSUPPORTED;

   return result;
}

ar_result_t amdb_clear_cmd_ctrl(amdb_thread_t *amdb_info_ptr, amdb_cmd_ctrl_t *amdb_cmd_ctrl_ptr)
{
   ar_result_t result = AR_EUNSUPPORTED;

   return result;
}

/*
 * Function to route the command to the destination domain.
 * Caller will free the GPR packet in case of any error.
 * This function should only manage the memory allocated within.
 */
ar_result_t amdb_route_cmd_to_satellite(amdb_thread_t *amdb_info_ptr,
                                        spf_msg_t *    msg_ptr,
                                        uint32_t       dst_domain_id,
                                        uint8_t *      amdb_payload_ptr)
{
   ar_result_t result = AR_EUNSUPPORTED;
   return result;
}

ar_result_t amdb_route_basic_rsp_to_client(amdb_thread_t *amdb_info_ptr, spf_msg_t *msg_ptr)
{
   ar_result_t result = AR_EUNSUPPORTED;

   return result;
}

ar_result_t amdb_route_load_rsp_to_client(amdb_thread_t *amdb_info_ptr, spf_msg_t *msg_ptr)
{
   ar_result_t result = AR_EUNSUPPORTED;
   return result;
}

void amdb_load_handle_holder_free(void *void_ptr, spf_hash_node_t *node)
{
   return;
}
