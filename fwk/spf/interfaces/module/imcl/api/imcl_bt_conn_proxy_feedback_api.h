/**========================================================================
 @file imcl_bt_conn_proxy_feedback_api.h

 @brief This file contains API's to send the connectivity proxy feedback

 Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause-Clear
 ====================================================================== */

#ifndef _IMCL_BT_CONN_PROXY_FEEDBACK_H_
#define _IMCL_BT_CONN_PROXY_FEEDBACK_H_

/*------------------------------------------------------------------------------
 *  Header Includes
 *----------------------------------------------------------------------------*/

#include "imcl_fwk_intent_api.h"

#ifdef INTENT_ID_BT_CONN_PROXY_FEEDBACK

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/** -----------------------------------------------------------------------
 ** Type Declarations
 ** ----------------------------------------------------------------------- */
/* clang-format off */

/* Param id is sent by conn proxy source module to conn proxy sink module to indicate
 * that the primer sideband has been received by DSP */
#define IMCL_PARAM_ID_PRIMER_FEEDBACK        0x080013C6

typedef struct param_id_primer_feedback_t param_id_primer_feedback_t;

struct param_id_primer_feedback_t
{
	 int64_t    deadline_time_us;
	 /**< Deadline time */

	 uint16_t   frame_interval_us;
	 /**< Frame duration in microseconds. This indicates the encoder and decoder frame durations.*/

	 uint16_t   frame_size_bytes;
	 /**< Frame size of the encoded data. It's the max payload size that encoder is expected to provide for each to-air interval. */
};

/* clang-format on */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif //INTENT_ID_BT_CONN_PROXY_FEEDBACK

#endif /* _IMCL_BT_CONN_PROXY_FEEDBACK_H_ */
