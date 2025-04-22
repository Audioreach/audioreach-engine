#ifndef _CAPI_FWK_EXTNS_ECNS_H
#define _CAPI_FWK_EXTNS_ECNS_H

/**
 * \file capi_fwk_extns_ecns.h
 *
 * \brief
 *     Frame work extensions for Echo canceller and Noise suppression (ECNS)
 *    This file defines a framework extensions and corresponding private propeties needed for ECNS in voice.
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

#include "capi_types.h"

/** @addtogroup capi_fw_ext_ecns
The ECNS framework extension (FWK_EXTN_ECNS) provides support for the echo
cancellation and noise suppression (ECNS) feature.

ECNS is a fundamental part of voice uplink processing. When a person uses a
phone to make a voice call, the sound played back on the speaker is echoed back
to the microphone electrically and acoustically. This echo can be perceived by
the far end and can vary from mildly annoying to unacceptable, depending on how
much coupling exists.

EC algorithms cancel this echoed signal from the microphone input with an
adaptive filter that models the path taken by the echo. When this model is
combined with the signal played on the speaker, a replica of the echo can be
created, which is then subtracted from the microphone signal. The noise
suppressor suppresses the near-end noise.
*/

/** @ingroup capi_fw_ext_ecns
  Unique identifier of the custom framework extension used by modules that
  support the ECNS feature.
*/
#define FWK_EXTN_ECNS 0x0A00101E

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // _CAPI_FWK_EXTNS_ECNS_H
