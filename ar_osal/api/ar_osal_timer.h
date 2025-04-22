#ifndef AR_OSAL_TIMER_H
#define AR_OSAL_TIMER_H

/**
 * \file ar_osal_timer.h
 * \brief
 *      Defines public APIs for timer operations.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_osal_types.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/**
 * \brief ar_timer_get_time_in_us
 *        Gets the wall clock time in microseconds
 * \return
 *        Wall clock time in microseconds.
 */
uint64_t ar_timer_get_time_in_us(void);


/**
 * \brief ar_timer_get_time_in_ms
 *        Gets the wall clock time in milliseconds
 * \return
 *        Wall clock time in milliseconds.
 */
uint64_t ar_timer_get_time_in_ms(void);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef AR_OSAL_TIMER_H */
