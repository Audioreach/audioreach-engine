#ifndef MSIIR_CALIBRATION_API_H
#define MSIIR_CALIBRATION_API_H
/*============================================================================
  @file msiir_calibration_api.h

  Calibration (Public) API for Multi Stage IIR Filter (single channel).

        Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
        SPDX-License-Identifier: BSD-3-Clause-Clear
============================================================================*/
#include "AudioComdef.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
   Constants
----------------------------------------------------------------------------*/
#define MSIIR_NUM_COEFFS   (3)            // numerator count
#define MSIIR_DEN_COEFFS   (2)            // denominator count
#define MSIIR_COEFF_LENGTH (MSIIR_NUM_COEFFS+MSIIR_DEN_COEFFS) // 5

/*----------------------------------------------------------------------------
   Parameters with IDs
----------------------------------------------------------------------------*/
#define MSIIR_PARAM_LIB_VER      (0)   // ** param: library version
typedef int32 msiir_version_t;         //    access: get only

#define MSIIR_PARAM_PREGAIN      (1)   // ** param: filter pregain
typedef int32 msiir_pregain_t;         //    range: [0, 0x7fffffff] (Q27)
                                       //    access: get & set

#define MSIIR_PARAM_CONFIG       (2)   // ** param: stages, coeffs & shift
                                       //    access: get & set
typedef struct msiir_coeffs_t {        
   int32 iir_coeffs[MSIIR_COEFF_LENGTH]; //  [b0, b1, b2, a1, a2]
   int32 shift_factor;                 //    shift factor for this stage
} msiir_coeffs_t;                      //
                                       //
typedef struct msiir_config_t {        //    
   int32             num_stages;       //    stage range: [0, max_stages]
   // msiir_coeffs_t  coeffs_struct[num_stages]
} msiir_config_t;
   // coeffs struct will follow in memory immediately after this config struct

#define MSIIR_PARAM_RESET        (3)   // ** param: reset filter memory
                                       //    access: set only
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MSIIR_CALIBRATION_API_H */

