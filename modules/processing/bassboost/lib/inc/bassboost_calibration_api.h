#ifndef BASSBOOST_CALIBRATION_API_H
#define BASSBOOST_CALIBRATION_API_H
/*============================================================================
  @file bassboost_calibration_api.h

  Calibration (Public) API for Bass Boost Effect

        Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
        SPDX-License-Identifier: BSD-3-Clause-Clear
============================================================================*/
#include "AudioComdef.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
   Parameters with IDs
----------------------------------------------------------------------------*/
#define BASSBOOST_PARAM_LIB_VER    (0) // ** param: library version
typedef int32 bassboost_version_t;     //    access: get only

#define BASSBOOST_PARAM_ENABLE     (1) // ** param: on/off switch
typedef int32 bassboost_enable_t;      //    access: get & set, runtime
                                       //    range: [0 (OFF) or 1 (ON)]

#define BASSBOOST_PARAM_MODE       (2) // ** param: processing mode
typedef enum bassboost_mode_t {        //    access: get & set
   PHYSICAL_BOOST = 0,                 //    0: physical boost (headphone)
   VIRTUAL_BOOST,                      //    1: virtual boost (small speaker)
   INVALID_BOOST = 0x7FFFFFFF
} bassboost_mode_t;                    //       (mode 1 is now placeholder)

#define BASSBOOST_PARAM_STRENGTH   (3) // ** param: effect strength
typedef int32 bassboost_strength_t;    //    access: get & set, runtime
                                       //    range: [0, 1000] (per millie)

#define BASSBOOST_PARAM_RESET      (4) // ** param: reset internal memories 
                                       //    access: set only

#define BASSBOOST_PARAM_DELAY      (5) // ** param: report alg delay (samples)
typedef int32 bassboost_delay_t;       //    access: get only

#define BASSBOOST_PARAM_CROSSFADE_FLAG (6)//**param: whether in transition state
typedef int32 bassboost_crossfade_t;   //    access: get only
                                       //    range: [0 (NO) or 1 (YES)]

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* BASSBOOST_CALIBRATION_API_H */
