/**
 * \file spl_cntr_cmn_utils.h
 * \brief
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "spl_cntr.h"
#include "capi.h"
#include "posal.h"
#include "spf_macros.h"
#include "shared_lib_api.h"
#include "gpr_packet.h"
#include "gpr_api_inline.h"
#include "ar_guids.h"
#include "spl_cntr_sync_fwk_ext.h"
#include "spl_cntr_voice_delivery_fwk_ext.h"
#include "spl_cntr_proc_dur_fwk_ext.h"
#include "spl_topo_trigger_policy_fwk_ext.h"

#ifndef SPL_CNTR_CMN_UTIL_H
#define SPL_CNTR_CMN_UTIL_H

// clang-format off

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#define SPL_CNTR_MSG_PREFIX                         "SC  :%lX: "
#define SPL_CNTR_LOG_PREFIX                         0x000000AB
#define SPL_CNTR_MSG(ID, xx_ss_mask, xx_fmt, ...)   AR_MSG(xx_ss_mask, SPL_CNTR_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif //__cplusplus

// clang-format on

#endif // #ifndef SPL_CNTR_CMN_UTIL_H
