#ifndef POSAL_ROOT_MSG_H
#define POSAL_ROOT_MSG_H

/*==============================================================================
  @brief Posal Root logging

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

/* =======================================================================
    INCLUDE FILES FOR MODULE
========================================================================== */
#if !defined(SIM)
#include <assert.h>
#include "qurt.h"
#include "msgcfg.h"
#include "msg.h"
#else
#include <stdio.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#if defined(SIM)

#define POSAL_ROOT_MSG(xx_fmt, message, ...)                                                                           \
   printf("POSAL_ROOT: %d of %s : " message "\n", __LINE__, __FILENAME__, ##__VA_ARGS__);

#define POSAL_ROOT_MSG_ISLAND POSAL_ROOT_MSG_ISLAND

#else

#define POSAL_ROOT_VA_NUM_ARGS_IMPL(a, b, c, d, e, f, g, h, i, j, _N, ...) _N
#define POSAL_ROOT_VA_NUM_ARGS(...) POSAL_ROOT_VA_NUM_ARGS_IMPL(, ##__VA_ARGS__, _9, _8, _7, _6, _5, _4, _3, _2, _1, )
#define POSAL_ROOT_TOKENPASTE(x, y) x##y

#define POSAL_ROOT_MSG_x(_N) POSAL_ROOT_TOKENPASTE(MSG, _N)

#define POSAL_ROOT_MSG(xx_ss_mask, xx_fmt, ...)                                                                        \
   POSAL_ROOT_MSG_x(POSAL_ROOT_VA_NUM_ARGS(__VA_ARGS__))(MSG_SSID_QDSP6, xx_ss_mask, xx_fmt, ##__VA_ARGS__)

#define POSAL_ROOT_MSG_ISLAND(xx_ss_mask, xx_fmt, ...) MICRO_MSG(MSG_SSID_QDSP6, xx_ss_mask, xx_fmt, ##__VA_ARGS__)

#endif

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_ROOT_MSG_H