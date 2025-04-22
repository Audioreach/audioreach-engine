#ifndef MSIIR_H
#define MSIIR_H
/*============================================================================
  @file msiir.h

  Private Header for Multi Stage IIR Filter.

        Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
        SPDX-License-Identifier: BSD-3-Clause-Clear
============================================================================*/
#include "msiir_api.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/*----------------------------------------------------------------------------
   Constants
----------------------------------------------------------------------------*/
#define MSIIR_FILTER_STATES      (2)         // mem length per biquad
#define MSIIR_DEN_SHIFT          (2)         // fixed denominator shift factor

static const int32 msiir_max_stack_size = 2000;    // worst case stack mem
static const int32 c_unity_pregain = 134217728;    // unity gain (Q27)

/*----------------------------------------------------------------------------
   Private Types
----------------------------------------------------------------------------*/
typedef struct iir_data_t {                  // ** second order section type
   int64             states[MSIIR_FILTER_STATES];
   int32             coeffs[MSIIR_COEFF_LENGTH];
   int32             shift_factor;
} iir_data_t;

typedef struct mult_stage_iir_t{             // ** multi stage IIR 
   msiir_static_vars_t  static_vars;         //    copy of static vars
   msiir_pregain_t      pre_gain;            //    filter pre gain
   int32                num_stages;             //    num of stages in use
   iir_data_t*          sos;                    //    second order sections
} mult_stage_iir_t;

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* MSIIR_H */

