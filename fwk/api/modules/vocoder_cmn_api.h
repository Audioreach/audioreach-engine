#ifndef _VOCODER_CMN_API_H_
#define _VOCODER_CMN_API_H_

/**
 * \file vocoder_cmn_api.h
 * \brief 
 *  	 This file contains vocoder modules' commmon APIs
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/** @h2xml_title1          {Vocoder Common API}
    @h2xml_title_agile_rev {Vocoder Common API}
    @h2xml_title_date      {April, 2019} */

#include "module_cmn_api.h"

/*==============================================================================
   Constants
==============================================================================*/
#define VOICE_TTY_MODE_OFF           0 /* TTY OFF or disabled */
#define VOICE_TTY_MODE_HCO           1 /* Hearing carry over mode. */
#define VOICE_TTY_MODE_VCO           2 /* Voice carry over mode. */
#define VOICE_TTY_MODE_FULL          3 /* FULL mode. */

#define VOCODER_OPERATING_MODE_NB    0 /* Vocoder bandwidth mode is 8 kHz. */
#define VOCODER_OPERATING_MODE_WB    1 /* Vocoder bandwidth mode is 16 kHz. */
#define VOCODER_OPERATING_MODE_SWB   2 /* Vocoder bandwidth mode is 32 kHz. */
#define VOCODER_OPERATING_MODE_FB    3 /* Vocoder bandwidth mode is 48 kHz. */
#define VOCODER_OPERATING_MODE_NONE  AR_NON_GUID(0xffffffff) 

typedef enum vocoder_client_id_t
{
  AUDIO_CLIENT             = 0x0,
  VOICE_CLIENT             = 0x1,
} vocoder_client_id_t;


typedef enum tty_cdma_vocoder_type_t {
   TTY_CDMA_V13K = 0,
   TTY_CDMA_EVRC,
   TTY_CDMA_FOURGV
}tty_cdma_vocoder_type_t;

/*==============================================================================
   Type definitions
==============================================================================*/

/* Unique Paramter id */
#define PARAM_ID_VOCODER_ENABLE_DTX_MODE 0x08001123

/* Structure definition for Parameter */

/** @h2xmlp_parameter   {"PARAM_ID_VOCODER_ENABLE_DTX_MODE",
                         PARAM_ID_VOCODER_ENABLE_DTX_MODE}
    @h2xmlp_description {Enables/Disables the Discontinous Transmission Mode}
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
struct param_id_vocoder_enable_dtx_mode_t
{
	uint32_t enable;
	/**< @h2xmle_description {Enables the DTX transmission mode}
     @h2xmle_rangeList   {Disabled=0;Enabled=1}
     @h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_vocoder_enable_dtx_mode_t param_id_vocoder_enable_dtx_mode_t;


/** ID of the parameter used to configure the encoder minimum and maximum rates */
#define PARAM_ID_VOCODER_MINMAX_RATE 0x08001124


/** @h2xmlp_parameter   {"PARAM_ID_VOCODER_MINMAX_RATE", PARAM_ID_VOCODER_MINMAX_RATE}
    @h2xmlp_description {Parameter for setting the min and max rates on the encoder module}
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
struct param_id_vocoder_minmax_rate_param_t
{
   uint32_t min_rate;
   /**< @h2xmle_description {Minimum supported encoding rate}
        @h2xmle_rangeList   {"SILENCE_RATE"=0,"QUARTER_RATE"=1,"HALF_RATE"=3,"FULL_RATE"=4 }
        @h2xmle_policy      {Basic} */

   uint32_t max_rate;
   /**< @h2xmle_description {Maximum supported encoding rate}
        @h2xmle_rangeList   {"SILENCE_RATE"=0,"QUARTER_RATE"=1,"HALF_RATE"=3,"FULL_RATE"=4 }
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_vocoder_minmax_rate_param_t param_id_vocoder_minmax_rate_param_t;


/** ID of the parameter used to configure the encoding rate */
#define PARAM_ID_VOCODER_ENC_RATE 0x080011B1

/** @h2xmlp_parameter   {"PARAM_ID_VOCODER_ENC_RATE", PARAM_ID_VOCODER_ENC_RATE}
    @h2xmlp_description {Parameter for setting encoder rate}
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
struct param_id_vocoder_enc_rate_param_t
{
   uint32_t       rate;
   /**<     
    @h2xmle_description {Encoding rate to be set. The exact interpretation
                         of the value and valid range depends on the type of encoder}
    @h2xmle_policy      {Basic}  */
}
#include "spf_end_pack.h"
;
typedef struct param_id_vocoder_enc_rate_param_t param_id_vocoder_enc_rate_param_t;


/** ID of the parameter used to configure the encoding rate for modulation */
#define PARAM_ID_VOCODER_RATE_MODULATION 0x08001125


/** @h2xmlp_parameter   {"PARAM_ID_VOCODER_RATE_MODULATION", PARAM_ID_VOCODER_RATE_MODULATION}
    @h2xmlp_description {Parameter for setting the encoding rate for modulation
                         The exact interpretation of the value and valid range depends on the
  	  	  	  	  	  	 type of encoder}
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
struct param_id_vocoder_rate_modulation_param_t
{
   uint32_t rate_modulation_param;
   /**< @h2xmle_description {Encoder rate modulation command}
        @h2xmle_policy      {Basic} */

}
#include "spf_end_pack.h"
;
typedef struct param_id_vocoder_rate_modulation_param_t param_id_vocoder_rate_modulation_param_t;

/** ID of the parameter used to set the TTY mode */
#define PARAM_ID_TTY_MODE 0x08001127

/** @h2xmlp_parameter   {"PARAM_ID_TTY_MODE", PARAM_ID_TTY_MODE}
    @h2xmlp_description {Parameter for setting the TTY mode of operation}
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
struct param_id_tty_mode_t
{
   uint32_t mode;
   /**< @h2xmle_description {TTY mode to be set}
        @h2xmle_rangeList   {"VOICE_TTY_MODE_OFF"=0,
                             "VOICE_TTY_MODE_HCO"=1,
                             "VOICE_TTY_MODE_VCO"=2,
                             "VOICE_TTY_MODE_FULL"=3}
        @h2xmle_policy      {Basic} */ // TODO : Check the policy

}
#include "spf_end_pack.h"
;
typedef struct param_id_tty_mode_t param_id_tty_mode_t;

/* Unique Paramter id */
/**
 * Parameter to reset voice encoder/decoder.
   This param id can be used reset vocoder whenever client sends homing sequence to network.
   Since these homing sequence reset far end decoder, client can use this param id to reset
   near end encoder so that both encoder and decoder are in sync.
   This param doesn't have any payload associated with it. Once module receives this param, it is expected to
   RESET itself
 */
#define PARAM_ID_VOCODER_RESET 0x08001128

/**
 * Event ID that is raised by the vocoder when client registers for operating mode updates
 * with the vocoder. Payload of the event contains the operating mode details.
 * We have 4 operating modes - one for each of 8k/16k/32k/48k bandwidths
 */

#define EVENT_ID_VOCODER_OPERATING_MODE 0x08001197


/** @h2xmlp_parameter   {"EVENT_ID_VOCODER_OPERATING_MODE",
                            EVENT_ID_VOCODER_OPERATING_MODE}
      @h2xmlp_description { Operating mode that is returned by the vocoder.}
      @h2xmlp_toolPolicy  { Calibration}*/

#include "spf_begin_pack.h"
struct event_id_vocoder_operating_mode_t
{
   uint32_t operating_mode;
   /**< @h2xmle_description {Operating mode returned by the vocoder}
        @h2xmle_rangeList   {"VOCODER_OPERATING_MODE_NB"=0,
                             "VOCODER_OPERATING_MODE_WB"=1,
                             "VOCODER_OPERATING_MODE_SWB"=2,
                             "VOCODER_OPERATING_MODE_FB"=3}
        @h2xmle_policy      {Basic} */ // TODO : Check the policy
    uint32_t client_data;
    /**< @h2xmle_description {Client data to be used by the modem}
        @h2xmle_policy      {Basic} */ // TODO : Check the policy

}
#include "spf_end_pack.h"
;
typedef struct event_id_vocoder_operating_mode_t event_id_vocoder_operating_mode_t;

#endif /* _VOCODER_CMN_API_H_ */
