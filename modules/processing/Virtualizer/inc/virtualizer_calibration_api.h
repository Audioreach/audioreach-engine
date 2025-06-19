/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef VIRTUALIZER_CALIBRATION_API_H
#define VIRTUALIZER_CALIBRATION_API_H

#include "AudioComdef.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
   Parameters with IDs
----------------------------------------------------------------------------*/
#define VIRTUALIZER_PARAM_LIB_VER        (0)  // ** param: library version
typedef int32 virtualizer_version_t;          //    access: get only

#define VIRTUALIZER_PARAM_ENABLE         (1)  // ** param: on/off switch
typedef int32 virtualizer_enable_t;           //    access: get & set, runtime
                                              //    range: [0 (OFF) or 1 (ON)]

#define VIRTUALIZER_PARAM_STRENGTH       (2)  // ** param: effect strength
typedef int32 virtualizer_strength_t;         //    access: get & set, runtime
                                              //    range: [0, 1000]

#define VIRTUALIZER_PARAM_OUT_TYPE       (3)  // ** param: output type
typedef enum virtualizer_output_t {           //    access: get & set
   OUTPUT_HEADPHONES = 0,                     //    stereo headphone
   OUTPUT_DESKTOP_SPKS,                       //    speaker with large distance
   OUTPUT_DIPOLE_SPKS                         //    closely-spaced speakers
} virtualizer_output_t;

#define VIRTUALIZER_PARAM_DIPOLE_SPACING (4)  // ** param: output type
typedef int32 virtualizer_dipole_spacing_t;   //    access: get & set
// currently only 2 spacing tuned:            //    range: [1, 500] (millimeter)
// 50 mm and 100 mm          

#define VIRTUALIZER_PARAM_GAIN_ADJUST    (5)  // ** param: overall gain adjust
typedef int32 virtualizer_gain_adjust_t;      //    access: get & set
                                              //    range: [-600, 600] (mB)

#define VIRTUALIZER_PARAM_RESET          (6)  // ** param: reset library
                                              //    access: set only

#define VIRTUALIZER_PARAM_ALG_DELAY      (7)  // ** param: alg delay (samples)
typedef int32 virtualizer_alg_delay_t;        //    access: get only

#define VIRTUALIZER_PARAM_CROSSFADE_FLAG  (8)  // ** param: whether in transition state
typedef int32 virtualizer_crossfade_t;    //    access: get only
                                              //    range: [0 (N) or 1 (YES)]

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* VIRTUALIZER_CALIBRATION_API_H */
