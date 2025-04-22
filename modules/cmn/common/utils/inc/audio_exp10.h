/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/**
@file audio_exp10.h

This file contains declaration for exponential routine.
*/

/*===========================================================================
NOTE: The @brief description and any detailed descriptions above do not appear 
      in the PDF. 

      The elite_audio_mainpage.dox file contains all file/group descriptions 
      that are in the output PDF generated using Doxygen and Latex. To edit or 
      update any of the file/group text in the PDF, edit the 
      elite_audio_mainpage.dox file or contact Tech Pubs.
===========================================================================*/

#ifndef _EXP10_H_
#define _EXP10_H_

#include "audio_comdef.h"

/*=============================================================================
      Function Declarations 
=============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/** @addtogroup math_operations
@{ */

/**
  Exponential function. It computes 10^x for a given value of x.

  @param[in] x   Exponent.

  @return
  int32_t 10^x.
  
  @dependencies
  None.
*/
int32_t exp10_fixed(int32_t x); 

/** @} */ /* end_addtogroup math_operations */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /*_EXP10_H_*/

