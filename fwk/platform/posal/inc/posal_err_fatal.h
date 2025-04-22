#ifndef _POSAL_ERR_FATAL_H
#define _POSAL_ERR_FATAL_H
/**
 * \file posal_err_fatal.h
 * \brief 
 *  	 Contains API to call force crash.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "posal_types.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

void posal_err_fatal(const char *err_str);


#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* #ifndef _POSAL_ERR_FATAL_H */