#ifndef DRC_CALIB_API_H
#define DRC_CALIB_API_H
/*============================================================================
  @file CDrcCalibApi.h

  Public api for DRC.

        Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
		SPDX-License-Identifier: BSD-3-Clause
============================================================================*/
/*============================================================================

 $Header: //components/rel/audioreach_spm_pp.cmn/0.0/gain_control/drc/lib/inc/drc_calib_api.h#1 $

  when       who     what, where, why
  ---------- ------- ---------------------------------------------------------
  2012-11-12 juihuaj   Initial revision. 
  2012-11-28 juihuaj   Added param IDs and corresponding structures
============================================================================*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/


#include "ar_defs.h"







#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */




/*----------------------------------------------------------------------------
 * Type Declarations
 * -------------------------------------------------------------------------*/
// param ID and the corresponding payload for lib version
#define DRC_PARAM_GET_LIB_VER (0)		// read only
typedef int64_t	drc_lib_ver_t;		// lib version(major.minor.bug); (8bits.16bits.8bits)




typedef enum drc_mode_t
{
    DRC_BYPASSED =	0,     // DRC processing bypassed; no DRC processing and only delay is implemented
    DRC_ENABLED		       // DRC processing enabled; normal DRC processing

} drc_mode_t;

// param ID and the corresponding payload for DRC feature mode
#define DRC_PARAM_FEATURE_MODE (1)	// read/write
typedef drc_mode_t	drc_feature_mode_t;	// 1 is with DRC processing; 0 is no DRC processing(bypassed, only delay is implemented)



typedef enum drc_channel_linking_t
{
    CHANNEL_NOT_LINKED	=	0,      
    CHANNEL_LINKED      

} drc_channel_linking_t;

// param ID and the corresponding payload for DRC processing
#define DRC_PARAM_CONFIG (2)	// read/write
typedef struct drc_config_t
{

   // below two should not change during Reinit
   int16_t	channelLinked;				// Q0 channel mode -- Linked(1) or Not-Linked(0)   
   int16_t	downSampleLevel;			// Q0 Down Sample Level to save MIPS


   uint16_t	rmsTavUL16Q16;			    // Q16 Time Constant used to compute Input RMS
   uint16_t	makeupGainUL16Q12;		    // Q12 Makeup Gain Value

   
   int16_t	dnExpaThresholdL16Q7;
   int16_t	dnExpaSlopeL16Q8;
   uint32_t	dnExpaAttackUL32Q31;
   uint32_t	dnExpaReleaseUL32Q31;  
   int32_t	dnExpaMinGainDBL32Q23;
   uint16_t	dnExpaHysterisisUL16Q14;
   
   int16_t	upCompThresholdL16Q7;
   uint32_t	upCompAttackUL32Q31;
   uint32_t	upCompReleaseUL32Q31;
   uint16_t	upCompSlopeUL16Q16;
   uint16_t	upCompHysterisisUL16Q14;

   int16_t	dnCompThresholdL16Q7;
   uint16_t	dnCompSlopeUL16Q16;
   uint32_t	dnCompAttackUL32Q31;
   uint32_t	dnCompReleaseUL32Q31;
   uint16_t	dnCompHysterisisUL16Q14;
   


   int16_t dummy;  // avoid memory hole

} drc_config_t;


// param ID for reset(to flush memory)
// no payload needed for this ID
#define DRC_PARAM_SET_RESET (3)		// write only



// param ID and the corresponding payload for delay(in samples)
#define DRC_PARAM_GET_DELAY (4)		// read only
typedef uint32_t	drc_delay_t;	// Q0 Delay in samples per channel      










#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* #ifndef DRC_CALIB_API_H */
