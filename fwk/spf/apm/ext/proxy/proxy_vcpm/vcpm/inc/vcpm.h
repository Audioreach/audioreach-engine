#ifndef _VCPM_H_
#define _VCPM_H_

/**
 * \file vcpm.h
 * \brief
 *     This file contains declarations of the VCPM public API's
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

/*==============================================================================
 Public Function definitions
==============================================================================*/

/** Creates the command handler thread for the VCPM.

   return:  on success, or error code otherwise.

*/
ar_result_t vcpm_create();

/** Destroys the VCPM command handler thread and clean up
    resources.

   return: None
*/

void vcpm_destroy();

ar_result_t vcpm_get_spf_handle(spf_handle_t **spf_handle_pptr);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifdef _VCPM_H_ */
