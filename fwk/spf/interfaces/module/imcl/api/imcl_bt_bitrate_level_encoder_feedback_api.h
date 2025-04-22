/**========================================================================
 @file imcl_bt_bitrate_level_encoder_feedback_api.h

 @brief This file contains API's to send the sideband encoder feedback

 Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause-Clear

 ====================================================================== */

#ifndef _IMCL_BT_BITRATE_LEVEL_ENCODER_FEEDBACK_H_
#define _IMCL_BT_BITRATE_LEVEL_ENCODER_FEEDBACK_H_

/*------------------------------------------------------------------------------
 *  Header Includes
 *----------------------------------------------------------------------------*/

#include "imcl_fwk_intent_api.h"

#ifdef INTENT_ID_BT_ENCODER_FEEDBACK


#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/** -----------------------------------------------------------------------
 ** Type Declarations
 ** ----------------------------------------------------------------------- */

/* clang-format off */

/* Param id to send bitrate through imcl to encoder.
 * This is applicable for only COP v1 feedback. For bit rate level feedback from COP v2
 * use IMCL_PARAM_ID_BT_SIDEBAND_ENCODER_FEEDBACK_V2 */
#define IMCL_PARAM_ID_BT_BIT_RATE_LEVEL_ENCODER_FEEDBACK     0x0800115C

typedef struct imcl_param_id_bt_bitrate_encoder_feedback_t
{
   uint32_t bit_rate_level;
   /**< Indicates the bit rate level of the COP frame */

} imcl_param_id_bt_bitrate_encoder_feedback_t;


/* clang-format on */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif //INTENT_ID_BT_ENCODER_FEEDBACK

#endif /* _IMCL_BT_BITRATE_LEVEL_ENCODER_FEEDBACK_H_ */
