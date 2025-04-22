#ifndef CU_GLOBAL_SHMEM_MSG_H
#define CU_GLOBAL_SHMEM_MSG_H

/**
 * \file cu_global_shmem_msg.h
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

#include "posal.h"
#include "gpr_packet.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

typedef struct cu_base_t cu_base_t;

void cu_handle_global_shmem_msg(cu_base_t *cu_ptr, gpr_packet_t *packet_ptr);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef CU_GLOBAL_SHMEM_MSG_H
