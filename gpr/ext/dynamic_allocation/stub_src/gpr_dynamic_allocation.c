/**
 * \file gpr_dynamic_allocation.c
 * \brief
 *  	This file contains stub implementation for dynamic packet allocation.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gpr_dynamic_allocation.h"
#include "ar_osal_error.h"

GPR_INTERNAL uint32_t gpr_allocate_dynamic_packet(gpr_packet_t **packet, uint32_t size)
{
   return AR_EFAILED;
}

GPR_INTERNAL uint32_t gpr_check_and_free_dynamic_packet(gpr_packet_t *packet)
{
   return AR_EFAILED;
}

GPR_INTERNAL uint32_t gpr_dynamic_packet_init(void)
{
   return AR_EOK;
}

GPR_INTERNAL uint32_t gpr_dynamic_packet_deinit(void)
{
   return AR_EOK;
}
