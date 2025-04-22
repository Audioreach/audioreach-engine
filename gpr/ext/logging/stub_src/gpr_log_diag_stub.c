/**
 * \file gpr_log_diag_stub.c
 * \brief
 *    This file contains stubs of the gpr diag logging files
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/

#include "ar_osal_error.h"

#include "gpr_api_i.h"
#include "gpr_log.h"

uint32_t gpr_log_get_diag_log_code(uint32_t host_domain_id, uint32_t src_domain_id, uint32_t dest_domain_id)
{
   return 0;
}

uint32_t gpr_log_diag_packet(uint32_t log_code, gpr_packet_t *packet, uint32_t packet_size)
{
   return AR_EOK;
}