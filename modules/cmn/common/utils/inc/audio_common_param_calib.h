#ifndef AUDIO_COMMON_PARAM_CALIB_H
#define AUDIO_COMMON_PARAM_CALIB_H

/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*==============================================================================
  @file audio_common_param_calib.h
  @brief This file contains common parameters
==============================================================================*/

#include "ar_defs.h"

/*==============================================================================
   Constants
==============================================================================*/
/**
    ID of the lib version parameter used by any audio processing module.

    This generic/common parameter is used to query the
    lib version of any audio processing module.

 */
#define AUDPROC_PARAM_ID_LIB_VERSION                                   0x00010937

/* Structure for Querying module lib version of any Audio processing modules. */
typedef struct audproc_lib_version_t audproc_lib_version_t;
/** @h2xmlp_parameter   {"AUDPROC_PARAM_ID_LIB_VERSION", AUDPROC_PARAM_ID_LIB_VERSION}
    @h2xmlp_description {To query the lib version of any audio processing module.}  
    @h2xmlp_toolPolicy  {RTC}
    @h2xmlp_readOnly    {true}*/

#include "spf_begin_pack.h"

/* Payload of the AUDPROC_PARAM_ID_LIB_VERSION parameter used by
 any Audio Processing module
 */
struct audproc_lib_version_t
{
   uint32_t lib_version_low;
   /**< @h2xmle_description  {Version of the module LSB.} */


   uint32_t lib_version_high;
    /**< @h2xmle_description  { Version of the module MSB} */

}
#include "spf_end_pack.h"
;
#endif


