/**
 * \file gen_cntr_cmn_utils.h
 * \brief
 *   This file contains declarations of GEN_CNTR common utilities.
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "gen_cntr.h"
#include "spf_macros.h"

#ifndef GEN_CNTR_CMN_UTIL_H
#define GEN_CNTR_CMN_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#define GEN_CNTR_MSG_PREFIX "GC  :%08lX: "

#define GEN_CNTR_MSG(ID, xx_ss_mask, xx_fmt, ...) AR_MSG(xx_ss_mask, GEN_CNTR_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)
#define GEN_CNTR_MSG_ISLAND(ID, xx_ss_mask, xx_fmt, ...)                                                               \
   AR_MSG_ISLAND(xx_ss_mask, GEN_CNTR_MSG_PREFIX xx_fmt, ID, ##__VA_ARGS__)
#ifdef SIM
#define GEN_CNTR_SIM_DEBUG(dbg_log, msg, ...)                                                                          \
   do                                                                                                                  \
   {                                                                                                                   \
      AR_MSG(DBG_LOW_PRIO, GEN_CNTR_MSG_PREFIX msg, dbg_log, ##__VA_ARGS__);                                           \
   } while (0)

#else
#define GEN_CNTR_SIM_DEBUG(dbg_log, msg, ...)                                                                          \
   do                                                                                                                  \
   {                                                                                                                   \
   } while (0)
#endif

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef AUDCMNUTIL_H
