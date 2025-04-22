#ifndef _SPF_MAIN_H_
#define _SPF_MAIN_H_

/**
 * \file spf_main.h
 * \brief
 *    This file provides an API wrapper for high level spf frame work functions
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

/** @ingroup gkmain_func_init
  Initializes the spf framework.

  @return
  Success or failure code.

  @dependencies
  None.
*/
ar_result_t spf_framework_pre_init(void);

/** @ingroup gkmain_func_init
  Initializes APM as part of spf framework init.

  @return
  Success or failure code.

  @dependencies
  None.
*/
ar_result_t spf_framework_post_init(void);

/** @ingroup gkmain_func_deinit
  De-initializes the spf framework.

  @return
  Success or failure code.

  @dependencies
  None. @newpage
*/
//ar_result_t spf_framework_deinit(void);
ar_result_t spf_framework_pre_deinit(void);

/** @ingroup gkmain_func_deinit
  De-initializes APM static service.

  @return
  Success or failure code.

  @dependencies
  None. @newpage
*/
//ar_result_t spf_framework_deinit(void);
ar_result_t spf_framework_post_deinit(void);

/** @ingroup gkmain_func_start
  Starts the spf framework, including static services.

  @return
  Success or failure code.

  @dependencies
  None.
*/
ar_result_t spf_framework_start(void);

/** @ingroup gkmain_func_stop
  Stops the spf framework, including static services.

  @return
  Success or failure code.

  @dependencies
  None.
*/
ar_result_t spf_framework_stop(void);

/** @ingroup gkmain_func_tests
  Performs the framework unit test cases and also tests the framework test
  case.

  @return
  Success or failure code.

  @dependencies
  None.
*/
ar_result_t spf_framework_unit_tsts(void);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifdef _SPF_MAIN_H_
