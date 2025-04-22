/**
 * \file dtmf_detection_api.h
 * \brief 
 *  	 This file contains DTMF Detection Event api
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef DTMF_DETECTION_API_H
#define DTMF_DETECTION_API_H

#include "ar_defs.h"

#define EVENT_ID_DTMF_DETECTION                      0x0800113B

/*==============================================================================
   Type definitions
==============================================================================*/

/** @h2xmlp_parameter   {"EVENT_ID_DTMF_DETECTION",
                          EVENT_ID_DTMF_DETECTION}
    @h2xmlp_description { Detection event raised by the DTMF Detector Module.}
    @h2xmlp_toolPolicy  { NO_SUPPORT}*/


typedef struct dtmf_detect_event_t dtmf_detect_event_t;

/* Payload returned by DTMF module in response to the EVENT_ID_DTMF_DETECTION
 * registered by the client     */
#include "spf_begin_pack.h"
struct dtmf_detect_event_t
{
    uint16_t tone_low_freq;
    /**< @h2xmle_description {Low frequency (in Herz) from the below list}
         @h2xmle_default     {0}
         @h2xmle_rangeList   {"INVALID_VALUE" = 0,697,770,852,941}
         @h2xmle_policy      {Basic} */

    uint16_t tone_high_freq;
    /**< @h2xmle_description {High frequency (in Herz) from the below list}
         @h2xmle_default     {0}
         @h2xmle_rangeList   {"INVALID_VALUE" = 0,1209,1336,1477,1633}
         @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;

#endif // DTMF_DETECTION_API_H
