#ifndef BASS_BOOST_CALIB_H
#define BASS_BOOST_CALIB_H
/*==============================================================================
  @file bass_boost_calib.h
  @brief This file contains BASS_BOOST API
==============================================================================*/

/*=======================================================================
* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
=========================================================================*/

#include "module_cmn_api.h"

/** @h2xml_title1           {BASS_BOOST Module API}
    @h2xml_title_agile_rev  {BASS_BOOST Module API}
    @h2xml_title_date       {May 30, 2019} */
/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/
/*==============================================================================
   Constants
==============================================================================*/
#define CAPI_BASSBOOST_MAX_IN_PORTS 1
#define CAPI_BASS_BOOST_MAX_OUT_PORTS 1
#define CAPI_BASSBOOST_DEFAULT_PORT 0
#define BASS_BOOST_STACK_SIZE 4096


/* Global unique Module ID definition
   Module library is independent of this number, it defined here for static
   loading purpose only */

#define MODULE_ID_BASS_BOOST                             0x07001062
/**
    @h2xmlm_module       {"MODULE_ID_BASS_BOOST",
                          MODULE_ID_BASS_BOOST}
    @h2xmlm_displayName  {"Bass Boost"}
    @h2xmlm_modSearchKeys{effects, Audio}
    @h2xmlm_description  {ID of the Bass Boost module.\n

      This module supports the following parameter IDs:\n
	  - #PARAM_ID_MODULE_ENABLE \n
      - #PARAM_ID_BASS_BOOST_MODE\n
      - #PARAM_ID_BASS_BOOST_STRENGTH\n

      All parameter IDs are device independent.\n

	  Supported Input Media Format:\n
*  - Data Format          : FIXED_POINT\n
*  - fmt_id               : Don't care\n
*  - Sample Rates         : Any (>0)\n
*  - Number of channels   : 1 to 8\n
*  - Channel type         : 1 to 63\n
*  - Bits per sample      : 16, 32\n
*  - Q format             : 15 for bits per sample = 16 and 27 for bps = 32\n
*  - Interleaving         : de-interleaved unpacked\n
*  - Signed/unsigned      : Signed }

     @h2xmlm_toolPolicy              {Calibration}

	 @h2xmlm_dataMaxInputPorts    {CAPI_BASSBOOST_MAX_IN_PORTS}
     @h2xmlm_dataInputPorts       {IN=2}
     @h2xmlm_dataMaxOutputPorts   {CAPI_BASS_BOOST_MAX_OUT_PORTS}
     @h2xmlm_dataOutputPorts      {OUT=1}
     @h2xmlm_supportedContTypes  {APM_CONTAINER_TYPE_SC}
     @h2xmlm_isOffloadable       {true}
     @h2xmlm_stackSize            {BASS_BOOST_STACK_SIZE}
     @h2xmlm_ToolPolicy              {Calibration}

 @{                   <-- Start of the Module -->
     @h2xml_Select        {"param_id_module_enable_t"}
     @h2xmlm_InsertParameter
*/

/* ID of the Bass Boost Mode parameter used by MODULE_ID_BASS_BOOST. */
#define PARAM_ID_BASS_BOOST_MODE                         0x0800112C

/* Structure for the mode parameter of Bass Boost module. */
typedef struct bass_boost_mode_t bass_boost_mode_t;
/** @h2xmlp_parameter   {"PARAM_ID_BASS_BOOST_MODE", PARAM_ID_BASS_BOOST_MODE}
    @h2xmlp_description {Configures the bass boost mode}  */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/* Payload of the PARAM_ID_BASS_BOOST_MODE parameter used by the Bass Boost module. */
struct bass_boost_mode_t
{
   uint32_t bass_boost_mode;
    /**< @h2xmle_description     {Specifies the bass boost mode. This parameter is device dependent.\n
    Physical boost enhances the low frequency contents.\n
    Currently, only Physical Boost mode is supported.\n}

    @h2xmle_rangeList        {"Physical boost (used by the headphone)"= 0}
    @h2xmle_default          {0}   */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;



/* ID of the Bass Boost Strength parameter used by MODULE_ID_BASS_BOOST. */
#define PARAM_ID_BASS_BOOST_STRENGTH                     0x0800112D


/* Structure for the strength parameter of Bass Boost module. */
typedef struct bass_boost_strength_t bass_boost_strength_t;
/** @h2xmlp_parameter   {"PARAM_ID_BASS_BOOST_STRENGTH", PARAM_ID_BASS_BOOST_STRENGTH}
    @h2xmlp_description {Specifies the effects of bass boost.} */


#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"


/* Payload of the PARAM_ID_BASS_BOOST_STRENGTH parameter used by the Bass Boost module. */
struct bass_boost_strength_t
{
   uint32_t strength;
   /**< @h2xmle_description {Specifies the effects of bass boost. This parameter affects the audio
        stream and is device independent.\n}

        @h2xmle_range        {0..1000}
	@h2xmle_default      {1000} */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/** @}                   <-- End of the Module -->*/

#endif
