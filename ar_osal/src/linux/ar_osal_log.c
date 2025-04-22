/**
 * \file ar_osal_log.c
 *
 * \brief
 *      This file has implementation of logging related helper functions.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include <stdio.h>
#include <stdarg.h>
#ifdef AR_OSAL_USE_CUTILS
#include <cutils/properties.h>
#endif

#ifdef AR_OSAL_USE_SYSLOG
#include <syslog.h>

#ifndef ANDROID_LOG_DEBUG
#define ANDROID_LOG_DEBUG 3
#endif

#ifndef ANDROID_LOG_INFO
#define ANDROID_LOG_INFO 4
#endif

#ifndef ANDROID_LOG_ERROR
#define ANDROID_LOG_ERROR 6
#endif

#ifndef ANDROID_LOG_VERBOSE
#define ANDROID_LOG_VERBOSE 2
#endif

#ifndef ANDROID_LOG_FATAL
#define ANDROID_LOG_FATAL 7
#endif

static inline void __android_log_write(int prio, const char *tag, const char *msg)
{
    syslog(prio, "%s - %s", tag, msg);
}
#else
#include <log/log.h>
#endif

#include "ar_osal_log.h"
#define LOG_BUF_SIZE 1024

uint32_t ar_log_lvl = (AR_CRITICAL|AR_ERROR|AR_INFO);

_IRQL_requires_max_(DISPATCH_LEVEL)
void ar_log_init(void)
{
#ifdef AR_OSAL_USE_CUTILS
    //set this property to change the args debug logging enabled.
    if(property_get_bool("vendor.audio.args.enable.debug.logs", 0)) {
        ar_log_lvl = (AR_CRITICAL|AR_ERROR|AR_INFO|AR_DEBUG);
    }
    //set this property to change the args verbose logging enabled.
    if(property_get_bool("vendor.audio.args.enable.verbose.logs", 0)) {
        ar_log_lvl = (AR_CRITICAL|AR_ERROR|AR_INFO|AR_DEBUG|AR_VERBOSE);
    }
#endif
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void ar_log(uint32_t level, const char_t* log_tag, const char_t* file,
        const char_t* fn, int32_t ln, const char_t* format, ...)
{
    va_list ap;
    char buf_temp[LOG_BUF_SIZE];
    char buf[LOG_BUF_SIZE];

    va_start(ap, format);
    snprintf(buf_temp, LOG_BUF_SIZE, "%s:%s:%d %s", file, fn, ln, format);
    vsnprintf(buf, LOG_BUF_SIZE, buf_temp, ap);
    va_end(ap);

    if (level == AR_DEBUG) {
        __android_log_write(ANDROID_LOG_DEBUG, log_tag, buf);
    } else if (level == AR_INFO){
        __android_log_write(ANDROID_LOG_INFO, log_tag, buf);
    } else if (level == AR_ERROR) {
        __android_log_write(ANDROID_LOG_ERROR, log_tag, buf);
    }
    else if (level == AR_VERBOSE) {
        __android_log_write(ANDROID_LOG_VERBOSE, log_tag, buf);
    }
    else if (level == AR_CRITICAL) {
        __android_log_write(ANDROID_LOG_FATAL, log_tag, buf);
    }
}

_IRQL_requires_max_(DISPATCH_LEVEL)
void ar_log_deinit(void)
{

}
