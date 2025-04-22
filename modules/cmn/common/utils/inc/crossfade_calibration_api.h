#ifndef CROSSFADE_CALIBRATION_API_H
#define CROSSFADE_CALIBRATION_API_H

/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*============================================================================
  FILE:          crossfade_calibration_api.h

  OVERVIEW:      This file has the configaration and data structures, API for cross fade algorithm.

  DEPENDENCIES:  None
============================================================================*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/


#include "ar_defs.h"



#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */


/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/


// this param id is for cross_fade_lib_ver_t
#define CROSS_FADE_PARAM_LIB_VER (0) // read only
typedef int32_t cross_fade_lib_ver_t;  


// this param id is for cross_fade_mode_t
#define CROSS_FADE_PARAM_MODE (1) // read/write
typedef uint32_t cross_fade_mode_t;	// 0 is no cross fade, 1 is cross fade


// this param id is for cross_fade_config_t
#define CROSS_FADE_PARAM_CONFIG (2) // read/write
typedef struct cross_fade_config_t
{
	uint32_t  converge_num_samples;      //  number of samples from old output
	uint32_t  total_period_msec;         //  T_convergence + T_crossfade
	
} cross_fade_config_t;




#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* #ifndef CROSSFADE_CALIBRATION_API_H */
