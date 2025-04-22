#ifndef AR_OSAL_LOG_H
#define AR_OSAL_LOG_H

/**
 * \file ar_osal_log.h
 * \brief
 *      Defines public APIs for message logging.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_osal_types.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

#define AR_CRITICAL         (0x1) /**< critical message, log unrecoverable condition .*/
#define AR_ERROR            (0x2) /**< error message, represents code bugs that should be debugged and fixed.*/
#define AR_DEBUG            (0x4) /**< debug message, required for debug. Not enabled by default*/
#define AR_INFO             (0x8) /**< Use for vital info that needs to exist in default logs */
#define AR_VERBOSE          (0x10)/**< verbose message, useful primarily to help developers debug low-level code */

extern uint32_t ar_log_lvl; /*gobal variable to control the log levels*/

/* Initialize logging */
void ar_log_init(void);

/* Deinitialize logging*/
void ar_log_deinit(void);

/* log function implemented by each platform*/
/* NOTE: New line ("\n") escape sequence is not expected to be provided with the end of each log message,
   each platform implementation is expected to support this if not implicitly supported, by default every
   new log message will be on a new line*/
void ar_log(uint32_t level, const char_t* log_tag, const char_t* file,
	const char_t* fn, int32_t ln, const char_t* format, ...);

/* Set log level dynamically */
/* This is designed so that a debugging user can turn on specific logs at
 * runtime. Call needs to be implemented in some higher layer or debugging tool
 * Pass as input the bitwise OR of the log levels desired.
 * Example: AR_CRITICAL|AR_ERROR|AR_INFO
 */
void ar_set_log_level(uint32_t level);

#define AR_LOG_VERBOSE(log_tag, ...)                                    \
    if (ar_log_lvl & AR_VERBOSE) {                                    \
        ar_log(AR_VERBOSE, log_tag, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__); \
    }

#define AR_LOG_INFO(log_tag,...)                                     \
    if (ar_log_lvl & AR_INFO) {                                    \
        ar_log(AR_INFO, log_tag, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__); \
    }

#define AR_LOG_DEBUG(log_tag, ...)                                    \
    if (ar_log_lvl & AR_DEBUG) {                                    \
        ar_log(AR_DEBUG, log_tag, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__); \
    }
#define AR_LOG_ERR(log_tag, ...)                                      \
    if (ar_log_lvl & AR_ERROR) {                                    \
        ar_log(AR_ERROR, log_tag, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__); \
    }

#define AR_LOG_CRITICAL(log_tag, ...)                                    \
    if (ar_log_lvl & AR_CRITICAL) {                                    \
        ar_log(AR_CRITICAL, log_tag, __FILE__, __FUNCTION__, __LINE__, __VA_ARGS__); \
    }

#define AR_FATAL_PRIO          (AR_CRITICAL)   /**< Fatal priority debug message. */
#define AR_ERROR_PRIO          (AR_ERROR   )   /**< Error priority debug message. */
#define AR_HIGH_PRIO           (AR_INFO   )   /**< High priority debug message. */
#define AR_MED_PRIO            (AR_DEBUG    )   /**< Medium priority debug message. */
#define AR_LOW_PRIO            (AR_VERBOSE )   /**< Low priority debug message. */

#define AR_LOG_FATAL(log_tag, ...)   AR_LOG_CRITICAL(log_tag, __VA_ARGS__)
#define AR_LOG_HIGH(log_tag, ...)    AR_LOG_INFO(log_tag, __VA_ARGS__)
#define AR_LOG_ERROR(log_tag, ...)   AR_LOG_ERR(log_tag, __VA_ARGS__)
#define AR_LOG_MED(log_tag, ...)     AR_LOG_DEBUG(log_tag, __VA_ARGS__)
#define AR_LOG_LOW(log_tag, ...)     AR_LOG_VERBOSE(log_tag, __VA_ARGS__)

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif //AR_OSAL_LOG_H
