#ifndef CONTAINER_UTILS_TGT_H_
#define CONTAINER_UTILS_TGT_H_

/**
 * \file container_utils.h
 *
 * \brief
 *     Common container framework code.
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "shared_lib_api.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/**--------------------------- General utilities ---------------------------*/
/**
 * returns conventional bit index (bit 0 for right most bit)
 */
static inline uint32_t cu_get_bit_index_from_mask(uint32_t mask)
{
   return (31 - s32_cl0_s32(mask)); // count leading zeros starting from MSB
   // (subtracting from 31 gives index of the 1 from right, (conventional bit index))
}

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef CONTAINER_UTILS_TGT_H_
