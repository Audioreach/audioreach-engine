#ifndef IMCL_STAGE2_STAGE1_API_H
#define IMCL_STAGE2_STAGE1_API_H
/**
  @file imcl_stage2_stage1_api.h

  @brief defines the Intent IDs for communication over Inter-Module Control
  Links (IMCL) betweeen detection engine and stage2 detection engine
*/
/*==========================================================================
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 * =========================================================================*/
#include "imcl_fwk_intent_api.h"

#ifdef INTENT_ID_DETECTION_ENGINE_STAGE2_TO_STAGE1

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/
/*==============================================================================
  Intent ID - INTENT_ID_SVA_SVA_RESET
==============================================================================*/
// Intent ID for the control link between detection engine and stage2 detection
// engine.

/* ============================================================================
   Param ID
==============================================================================*/

#define PARAM_ID_STAGE2_TO_STAGE1_RESET                           0x0800124C

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // INTENT_ID_DETECTION_ENGINE_STAGE2_TO_STAGE1

#endif /* #ifndef IMCL_STAGE2_STAGE1_API_H */
