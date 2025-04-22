#ifndef __GPR_LOG_I_H__
#define __GPR_LOG_I_H__

/**
 * \file gpr_log_i.h
 * \brief
 *  	This file contains internal structures for gpr logging
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*****************************************************************************
 * Defines                                                                   *
 ****************************************************************************/

#define GPR_LOG_HISTORY_BYTE_SIZE_V (4 * 1024)

/**
 * Indicates that the entire payload should be logged - not just the header
 */
#define GPR_LOG_PACKET_PAYLOAD 1

#define GPR_LOG_MAGIC_WORD_LEN 8
// Used to indicate start of gpr packets in the cicrular buffer
#define GPR_LOG_SYNC_WORD 0xE91111E9

/*****************************************************************************
 * Structure definitions                                                     *
 ****************************************************************************/

typedef struct gpr_log_history_t gpr_log_history_t;
struct gpr_log_history_t
{
   uint8_t  start_marker[GPR_LOG_MAGIC_WORD_LEN];
   uint32_t size;
   uint32_t offset;
   uint32_t log_payloads;
   uint8_t  log[GPR_LOG_HISTORY_BYTE_SIZE_V];
   uint8_t  end_marker[GPR_LOG_MAGIC_WORD_LEN];
};

void gpr_log_diag_packet(gpr_packet_t *packet, uint32_t packet_size);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* __GPR_LOG_I_H__ */