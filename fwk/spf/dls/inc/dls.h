#ifndef _DLS_H_
#define _DLS_H_
/**
 * \file dls.h
 * \brief
 *  	This file contains the DLS service public functions declarations
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "ar_error_codes.h"
#include "gpr_packet.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/* ----------------------------------------------------------------------------
 * Public Functions Declaration
 * ------------------------------------------------------------------------- */

/** Initializes dls and creates the command handler thread for the DLS

   return:  on success, or error code otherwise.
 */
ar_result_t dls_init();

/** Deinitializes dls and destroys the DLS command handler thread

   return: None
 */
ar_result_t dls_deinit();

/** Goes through the list of buffers and acquires an available buffer

   return: available buffer starting address
 */
uint64_t dls_acquire_buffer(uint16_t log_code,
                            uint32_t log_packet_size);

/** Commit the buffer and send a data event to the DLS client

   return: success/failure
 */
uint32_t dls_commit_buffer(void *log_pkt_ptr);

/** Goes through the current list of log codes and return the status

   return:
        TRUE  : if the log code exist
        FASLE : if the log code doesn't exist
 */
ar_result_t dls_log_code_status(uint32_t log_code);

/** Zero out the buffer and mark it as available for next buffer request

   return: None
 */
void dls_log_buf_free(void *log_pkt_ptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifdef _DLS_H_ */
