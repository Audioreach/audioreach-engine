/* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef EQUALIZER_PRESET_H
#define EQUALIZER_PRESET_H


#include "equalizer_api.h"
#include "equalizer_filter_design.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */



/*===========================================================================
*     Function Declarations
* ==========================================================================*/


EQ_RESULT equalizer_load_preset(
	eq_band_internal_specs_t    *pBands,         // array of eq band struct
	uint32                      *pNbands,        // ptr to number of bands
	eq_settings_t               preset          // preset index
	);





#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif  /* EQUALIZER_PRESET_H */
