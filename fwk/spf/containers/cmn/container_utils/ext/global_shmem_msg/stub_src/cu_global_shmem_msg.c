
/**
 * \file cu_global_shmem_msg.c
 *
 * \brief
 *
 *     CU utility for global shared  memory message handling.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "cu_global_shmem_msg.h"
#include "gpr_packet.h"
#include "gpr_api_inline.h"

void cu_handle_global_shmem_msg(cu_base_t *cu_ptr, gpr_packet_t *packet_ptr)
{
   __gpr_cmd_end_command(packet_ptr, AR_EUNSUPPORTED);
}
