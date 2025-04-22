#ifndef __GPR_LOG_H__
#define __GPR_LOG_H__

/**
 * \file gpr_log.h
 * \brief
 *  	This file contains GPR APIs for logging packets in case of a crash.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gpr_api.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/


/**
 * Initialize gpr logging
 * \return AR_EOK (0) when successful.
 *
 */
GPR_INTERNAL uint32_t gpr_log_init(void);

/**
 * De-initialize gpr logging, and clean up resources
 * \return AR_EOK (0) when successful.
 *
 */
GPR_INTERNAL uint32_t gpr_log_deinit(void);

/**
 * Log the given gpr packet to gpr_log_history_t
 * \return AR_EOK (0) when successful.
 *
 */
GPR_INTERNAL int32_t gpr_log_packet (gpr_packet_t* packet);
#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* __GPR_LOG_H__ */