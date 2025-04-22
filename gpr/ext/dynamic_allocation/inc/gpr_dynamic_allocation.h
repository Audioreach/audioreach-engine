#ifndef __GPR_DYN_ALLOCATION_H__
#define __GPR_DYN_ALLOCATION_H__

/**
 * \file gpr_dynamic_allocation.h
 * \brief
 *  	This file contains APIs to allocate GPR packets dynamically.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gpr_packet.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

GPR_INTERNAL uint32_t gpr_allocate_dynamic_packet(gpr_packet_t **packet,
                                                  uint32_t      size);

GPR_INTERNAL uint32_t gpr_check_and_free_dynamic_packet(gpr_packet_t *packet);

GPR_INTERNAL uint32_t gpr_dynamic_packet_init(void);

GPR_INTERNAL uint32_t gpr_dynamic_packet_deinit(void);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* __GPR_DYN_ALLOCATION_H__ */