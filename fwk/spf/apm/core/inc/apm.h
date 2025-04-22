#ifndef _APM_H_
#define _APM_H_

/**
 * \file apm.h
 * \brief
 *     This file contains declarations of the APM public API's
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_error_codes.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

typedef struct spf_handle_t spf_handle_t;

/*==============================================================================
 Public Function definitions
==============================================================================*/

/** Creates the command handler thread for the APM.

   return:  on success, or error code otherwise.

*/
ar_result_t apm_create();

/** Destroys the APM command handler thread and clean up
    resources.

   return: None
*/

void apm_destroy();

/** Get the registered memory map client

   return: memory map client
*/
uint32_t apm_get_mem_map_client();

/**
 * Ge the GK handle of APM
 *
 * return: gk handle
 */
spf_handle_t *apm_get_apm_handle();

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifdef _APM_H_ */
