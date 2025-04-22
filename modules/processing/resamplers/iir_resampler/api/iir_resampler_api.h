/*========================================================================*/
/**
@file iir_resampler_api.h

@brief iir_resampler_api.h: This file contains the Module Id, Param IDs and configuration
    structures exposed by the IIR Resampler Module. 
*/
/*=======================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
=========================================================================*/

#ifndef IIR_RESAMPLER_API_H
#define IIR_RESAMPLER_API_H

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "module_cmn_api.h"

 /**
     @h2xml_title1          {IIR RESAMPLER API}
     @h2xml_title_agile_rev  {IIR RESAMPLER API}
     @h2xml_title_date      {July 31, 2018}
  */
  
/*==============================================================================
   Constants
==============================================================================*/
/** @ingroup ar_spf_mod_iir_resam_macros
    Size of the stack for the IIR Resampler module. */
#define IIR_RS_STACK_SIZE              1024

/** @ingroup ar_spf_mod_iir_resam_macros
    Maximum ports for the IIR Resampler module. */
#define IIR_RS_MAX_PORTS               1

/** @ingroup ar_spf_mod_iir_resam_macros
    ID of the IIR RESAMPLER module.

    This module converts audio sampling rate. Uses IIR filters to do it.
    Therefore it renders output with a lower Delay compared to FIR.

    @subhead4{Supported input sample rates}
    - 8000 Hz
    - 16000 Hz
    - 24000 Hz
    - 32000 Hz
    - 48000 Hz

    @subhead4{Supported parameter IDs}
    - #PARAM_ID_IIR_RESAMPLER_OUT_CFG @lstsp1
    - #PARAM_ID_IIR_RESAMPELR_VERSION

    @subhead4{Supported input media format ID}
    - Data Format          : FIXED_POINT @lstsp1
    - fmt_id               : Don't care @lstsp1
    - Sample Rates         : 8000, 16000, 24000, 32000, 48000 (Hz) @lstsp1
    - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels) @lstsp1
    - Channel type         : 0 to 128 @lstsp1
    - Bits per sample      : 16 @lstsp1
    - Q format             : 15, 27, 31 @lstsp1
    - Interleaving         : de-interleaved unpacked @lstsp1
    - Signed/unsigned      : Don't care
 */
#define MODULE_ID_IIR_RESAMPLER                         0x07001018
/**
    @h2xmlm_module       {"MODULE_ID_IIR_RESAMPLER",
                          MODULE_ID_IIR_RESAMPLER}
    @h2xmlm_displayName  {"IIR Resampler"}
	@h2xmlm_modSearchKeys{resampler, Audio, Voice}
    @h2xmlm_description  {ID of the IIR RESAMPLER module.\n
    
     -This module converts audio sampling rate. Uses IIR filters to do it.
     Therefore it renders output with a lower Delay compared to FIR.\n
     - Input Sample Rates Supported: 8000, 16000, 24000, 32000, 48000 (Hz) \n
     - This module supports the following parameter IDs\n
     - #PARAM_ID_IIR_RESAMPLER_OUT_CFG\n
     - #PARAM_ID_IIR_RESAMPELR_VERSION\n

 *  - Supported Input Media Format: \n
 *  - Data Format          : FIXED_POINT \n
 *  - fmt_id               : Don't care\n
 *  - Sample Rates         : 8000, 16000, 24000, 32000, 48000 (Hz)\n
 *  - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels)\n
 *  - Channel type         : 0 to 128\n
 *  - Bits per sample      : 16\n
 *  - Q format             : 15, 27, 31\n
 *  - Interleaving         : de-interleaved unpacked\n
 *  - Signed/unsigned      : Don't care}

    @h2xmlm_dataMaxInputPorts    {IIR_RS_MAX_PORTS}
    @h2xmlm_dataInputPorts       {IN=2}
    @h2xmlm_dataMaxOutputPorts   {IIR_RS_MAX_PORTS}
    @h2xmlm_dataOutputPorts      {OUT=1}
    @h2xmlm_supportedContTypes  {APM_CONTAINER_TYPE_SC, APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable        {true}
    @h2xmlm_stackSize            {IIR_RS_STACK_SIZE}
    @h2xmlm_ToolPolicy              {Calibration}

    @{                   <-- Start of the Module -->
*/
/*------------------------------------------------------------------------
 * Macros, Defines, Type declarations
 * -----------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_iir_resam_macros
    Set param for sending output sample rate.
    IIR Resampler will allocate channel memory and configure the algorithm
    with this.*/ 
#define PARAM_ID_IIR_RESAMPLER_OUT_CFG                  0x08001033
typedef struct param_id_iir_resampler_out_cfg_t param_id_iir_resampler_out_cfg_t;

/** @h2xmlp_parameter   {"PARAM_ID_IIR_RESAMPLER_OUT_CFG", PARAM_ID_IIR_RESAMPLER_OUT_CFG}
    @h2xmlp_description {Specifies the output sample rate.IIR Resampler will allocate channel memory and 
                         configure the algorithm with this.}  
    @h2xmlp_toolPolicy   {Calibration; RTC } */

/** @ingroup ar_spf_mod_iir_resam_macros
    Specifies the output sample rate.IIR Resampler will allocate channel memory and 
    configure the algorithm with this. */

#include "spf_begin_pack.h"
struct param_id_iir_resampler_out_cfg_t
{
   int32_t sampling_rate;
   /**< Specifies the output sample rate; 0 is invalid. */

   /**< @h2xmle_description  {Specifies the output sample rate; 0 is invalid}
         @h2xmle_rangeList   {"PARAM_VAL_NATIVE"=-1;
                              "PARAM_VAL_UNSET"=-2;
                              "8 kHz"= 8000;
                              "16 kHz"=16000;
                              "24 kHz"=24000;
                              "32 kHz"=32000;
                              "48 kHz"=48000;
                              "96 kHz"=96000;
                              "192 kHz"=192000;
                              "384 kHz"=384000}  
         @h2xmle_default     {-1}           */
}
#include "spf_end_pack.h"
;


/** @ingroup ar_spf_mod_iir_resam_macros
    IIR Resampler module version. */
#define  PARAM_ID_IIR_RESAMPELR_VERSION         0x08001319
typedef struct iir_resampler_lib_version_param_t iir_resampler_lib_version_param_t;

/** @h2xmlp_parameter   {"PARAM_ID_IIR_RESAMPELR_VERSION", PARAM_ID_IIR_RESAMPELR_VERSION}
    @h2xmlp_description {The current version of the library is returned}  
    @h2xmlp_toolPolicy   {RTC_READONLY} */

/** @ingroup ar_spf_mod_iir_resam_macros
    The current version of the library is returned. */

#include "spf_begin_pack.h"
struct iir_resampler_lib_version_param_t
{
    uint32_t lib_version_lsb;
    /**< Lower 32 bits of the 64-bit library version number. */

    /**< @h2xmle_description {Lower 32 bits of the 64-bit library version number} */

    uint32_t lib_version_msb;
    /**< Higher 32 bits of the 64-bit library version number. */

    /**< @h2xmle_description {Higher 32 bits of the 64-bit library version number} */
}

#include "spf_end_pack.h"
;

/** @}                   <-- End of the Module -->*/
#endif /* IIR_RESAMPLER_API_H */
