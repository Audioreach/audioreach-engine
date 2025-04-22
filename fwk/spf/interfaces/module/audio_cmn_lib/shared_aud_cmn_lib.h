#ifndef __SHARED_AUDIO_BASIC_OP_EXT_H_
#define __SHARED_AUDIO_BASIC_OP_EXT_H_

/**
 * \file shared_aud_cmn_lib.h
 * \brief
 *        select APIs from basic ops, math utilities.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "module_cmn_api.h"
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** @addtogroup math_operations
@{ */

/* -----------------------------------------------------------------------
** Macro Definitions
** ----------------------------------------------------------------------- */

/** Shift factor for Q31 <=> Q27 conversion for 32-bit PCM
*/
#define QFORMAT_SHIFT_FACTOR (PCM_Q_FACTOR_31 - PCM_Q_FACTOR_27)

/** Shift factor for Q27 <=> Q15 conversion
*/
#define BYTE_UPDOWN_CONV_SHIFT_FACT (PCM_Q_FACTOR_27 - PCM_Q_FACTOR_15) // (Q27 - Q15)

int16_t div_s(int16_t var1, int16_t var2);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /*__BASIC_OP_EXT_H_*/
