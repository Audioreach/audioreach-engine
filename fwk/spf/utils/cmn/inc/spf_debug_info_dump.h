#ifndef _SPF_DEBUG_INFO_DUMP_H_
#define _SPF_DEBUG_INFO_DUMP_H_

/**
 * \file spf_minidump.h
 * \brief
 *    This file provides an API wrapper for minidump in spf frame work 
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*===========================================================================
NOTE: The @brief description above does not appear in the PDF.
      The description that displays in the PDF is located in the
      spf_mainpage.dox file. Contact Tech Pubs for assistance.
===========================================================================*/

/*-------------------------------------------------------------------------
Include Files
-------------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/*-------------------------------------------------------------------------
Preprocessor Definitions and Constants
-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------
Type Declarations
-------------------------------------------------------------------------*/

void spf_debug_info_dump(void *callback_data,int8_t *start_address,uint32_t max_size);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifdef _SPF_DEBUG_INFO_DUMP_H_
