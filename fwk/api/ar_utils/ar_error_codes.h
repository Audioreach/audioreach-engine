#ifndef _AR_ERROR_CODES_H_
#define _AR_ERROR_CODES_H_
/**
 * \file ar_error_codes.h
 * \brief 
 *    This file contains common error code definitions to be used across multimedia code bases
 * 
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

#include "ar_defs.h"
#include "ar_osal_error.h"

/** @addtogroup spf_utils_errors
@{ */

/** Status messages (error codes) returned by command responses. */
typedef uint32_t ar_result_t;

/** Macro used to evaluate the result of an operation (if it succeeded or
    failed). */
#define AR_DID_FAIL(x)    ( AR_EOK != (x) )

/** @} */ /* end_addtogroup spf_utils_errors */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* _AR_ERROR_CODES_H_ */
