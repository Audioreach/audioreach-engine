#ifndef _RATE_ADAPTED_TIMER_API_H_
#define _RATE_ADAPTED_TIMER_API_H_
/**
 * \file rate_adapted_timer_api.h
 * \brief 
 *  	 This file contains RATE_ADAPTED_TIMER module APIs
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "module_cmn_api.h"
#include "imcl_fwk_intent_api.h"

/**@h2xml_title1          {RATE_ADAPTED_TIMER Module API}
   @h2xml_title_agile_rev {RATE_ADAPTED_TIMER Module API}
   @h2xml_title_date      {April 8, 2019}  */

/*==============================================================================
   Constants
==============================================================================*/
/** @ingroup ar_spf_mod_rat_macros
    IMCL static port IDs to receive drifts. */
#define QT_REFCLK_TIMING_INPUT AR_NON_GUID(0xC0000001)
#define REFCLK_REMOTE_TIMING_INPUT AR_NON_GUID(0xC0000002)
#define QT_REMOTE_TIMING_INPUT AR_NON_GUID(0xC0000003)

/** @ingroup ar_spf_mod_rat_macros
    Timer duration is based on Sub-graph performance mode. */
#define    RAT_TIMER_DURATION_DEFAULT_MODE             0

/** @ingroup ar_spf_mod_rat_macros
    Timer duration is specified in time. */
#define    RAT_TIMER_DURATION_CONFIGURED_IN_TIME       2

/** @ingroup ar_spf_mod_rat_macros
    Timer duration configured in samples, time is derived based on sample rate. */
#define    RAT_TIMER_DURATION_CONFIGURED_IN_SAMPLES    3

/** @ingroup ar_spf_mod_rat_macros
    Sample rate is set as the sample rate configured in the PARAM_ID_RAT_MEDIA_FORMAT. */
#define RAT_TIMER_DURATION_RESOLUTION_CONTROL_CONFIGURED_SAMPLE_RATE_MODE 0

/** @ingroup ar_spf_mod_rat_macros
    sample rate is set as the minimum sample rate supported by the RAT module. */
#define RAT_TIMER_DURATION_RESOLUTION_CONTROL_MINIMUM_SAMPLE_RATE_MODE 1

/** @ingroup ar_spf_mod_rat_macros
    Minimum value of RAT timer duration configuration specified in time (us). */
#define    RAT_TIMER_DURATION_TIME_IN_US_MIN_VALUE     1000

/** @ingroup ar_spf_mod_rat_macros
    Maximum value of RAT timer duration configuration specified in time (us). */
#define    RAT_TIMER_DURATION_TIME_IN_US_MAX_VALUE     40000

/** @ingroup ar_spf_mod_rat_macros
    Minimum value of RAT timer duration configuration specified in samples. */
#define    RAT_TIMER_DURATION_IN_SAMPLES_MIN_VALUE     8      // 1ms * 8KHz

/** @ingroup ar_spf_mod_rat_macros
    Maximum value of RAT timer duration configuration specified in samples. */
#define    RAT_TIMER_DURATION_IN_SAMPLES_MAX_VALUE     15360  // 40ms * 38KHz

/** @ingroup ar_spf_mod_rat_macros
    Default value of RAT timer duration configuration specified in samples. */
#define    RAT_TIMER_DURATION_IN_SAMPLES_DEFAULT_VALUE     48      // 1ms * 48KHz

/** @ingroup ar_spf_mod_rat_macros
    Minimum fractional sample rate supported by RAT module. */
#define RAT_MIN_SUPPORTED_FRACTIONAL_SAMPLING_RATE 11025

/** @ingroup ar_spf_mod_rat_macros
    Minimum integer sample rate supported by RAT module. */
#define RAT_MIN_SUPPORTED_INTEGER_SAMPLING_RATE 8000

/** @ingroup ar_spf_mod_rat_macros
    RAT module stack size. */
#define RATE_ADAPTED_TIMER_STACK_SIZE 4096

/** @ingroup ar_spf_mod_rat_macros
    Input port ID of Rate Adapted Timer module. */
#define PORT_ID_RATE_ADAPTED_TIMER_INPUT 0x2

/** @ingroup ar_spf_mod_rat_macros
    Output port ID of Rate Adapted Timer module. */
#define PORT_ID_RATE_ADAPTED_TIMER_OUTPUT 0x1

/** @ingroup ar_spf_mod_rat_macros
    ID of the parameter used to configure the RAT media format. */
#define PARAM_ID_RAT_MEDIA_FORMAT 0x080010D0

/** @h2xmlp_parameter   {"PARAM_ID_RAT_MEDIA_FORMAT",
                         PARAM_ID_RAT_MEDIA_FORMAT}
    @h2xmlp_description {Configures the media format of both RAT and MIMO RAT modules and is mandatory\n
                   - For MIMO RAT this media format is applicable only to the static output port}
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/** @ingroup ar_spf_mod_rat_macros
    Configures the media format of both RAT and MIMO RAT modules and is mandatory. */
struct param_id_rat_mf_t
{
   uint32_t sample_rate;
   /**< Defines sample rate. */

   /**< @h2xmle_description {Defines sample rate.}
        @h2xmle_rangelist   {"8 kHz"=8000;
                             "11.025 kHz"=11025;
                             "12 kHz"=12000;
                             "16 kHz"=16000;
                             "22.05 kHz"=22050;
                             "24 kHz"=24000;
                             "32 kHz"=32000;
                             "44.1 kHz"=44100;
                             "48 kHz"=48000;
                             "88.2 kHz"=88200;
                             "96 kHz"=96000;
                             "176.4 kHz"=176400;
                             "192 kHz"=192000;
                             "352.8 kHz"=352800;
                             "384 kHz"=384000}
        @h2xmle_default     {48000}
   */

   uint16_t bits_per_sample;
   /**< Bits per sample. */

   /**< @h2xmle_description {bits per sample.}
        @h2xmle_rangeList   {"16-bit"=16;
                             "32-bit"=32}
        @h2xmle_default     {16}
   */

   uint16_t q_factor;
   /**< Q factor.
        - If bits per sample is 16, q_factor should be 15
        - If bits per sample is 32, q_factor can be 27 or 31 */

   /**< @h2xmle_description {q factor \n
                             -> If bits per sample is 16, q_factor should be 15 \n
                             -> If bits per sample is 32, q_factor can be 27 or 31}
        @h2xmle_rangeList   {"q15"=15;
                             "q27"=27;
                             "q31"=31}
        @h2xmle_default     {15}
   */

   uint32_t data_format;
   /**< Format of the data. */

   /**< @h2xmle_description {Format of the data}
        @h2xmle_rangeList   {"DATA_FORMAT_FIXED_POINT"=1}
        @h2xmle_default     {1}
   */

   uint32_t num_channels;
   /**< Number of channels. */

   /**< @h2xmle_description {Number of channels.}
        @h2xmle_range       {1...MODULE_CMN_MAX_CHANNEL} 
        @h2xmle_default     {1}
   */

   uint16_t channel_map[0];
   /**< Channel mapping array.
        - Specify a channel mapping for each output channel
        - If the number of channels is not a multiple of four, zero padding must be added
          to the channel type array to align the packet to a multiple of 32 bits */

   /**< @h2xmle_description  {Channel mapping array. \n
                              ->Specify a channel mapping for each output channel \n
                              ->If the number of channels is not a multiple of four, zero padding must be added
                              to the channel type array to align the packet to a multiple of 32 bits}
        @h2xmle_variableArraySize {num_channels}
        @h2xmle_rangeEnum   {pcm_channel_map}
        @h2xmle_default      {1}    */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/* Structure type def for above payload. */
typedef struct param_id_rat_mf_t param_id_rat_mf_t;


/** @ingroup ar_spf_mod_rat_macros
    ID of the parameter used to configure the RAT timer duration configuration. */
#define PARAM_ID_RAT_TIMER_DURATION_CONFIG 0x08001A6D

/** @h2xmlp_parameter   {"PARAM_ID_RAT_TIMER_DURATION_CONFIG",
                         PARAM_ID_RAT_TIMER_DURATION_CONFIG}
    @h2xmlp_description {Configures the timer duration of RAT module \n}
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"

/** @ingroup ar_spf_mod_rat_macros
    Configures the timer duration of RAT module. */
struct param_id_rat_timer_duration_config_t
{
	uint32_t timer_config_mode;
    /**< Specifies the options to configure the timer duration.
         Set to default to be backward compatible with existing functionality

	       @valuesbul
	        - 0 -- RAT_TIMER_DURATION_DEFAULT_MODE          - Timer duration is based on Sub-graph performance mode
	        - 2 -- RAT_TIMER_DURATION_CONFIGURED_IN_TIME    - Timer duration is specified in time
	        - 3 -- RAT_TIMER_DURATION_CONFIGURED_IN_SAMPLES - Timer duration configured in samples, time is derived based on sample rate
	         @tablebulletend */

	   /**< @h2xmle_description {Specifies the options to configure the timer duration.
	                             Set to default to be backward compatible with existing functionality.

	                             default - Timer duration is based on Sub-graph performance mode.
	                             configured_in_time - Timer duration is specified in time (us)
	                             configured_in_samples - Timer duration configured in samples, time is derived based on sample rate
	                             }
	        @h2xmle_default     {RAT_TIMER_DURATION_DEFAULT_MODE}
	        @h2xmle_rangeList   {"RAT_TIMER_DURATION_DEFAULT_MODE"= RAT_TIMER_DURATION_DEFAULT_MODE;
	                             "RAT_TIMER_DURATION_CONFIGURED_IN_TIME"= RAT_TIMER_DURATION_CONFIGURED_IN_TIME;
	                             "RAT_TIMER_DURATION_CONFIGURED_IN_SAMPLES"= RAT_TIMER_DURATION_CONFIGURED_IN_SAMPLES}
	     */

	uint32_t timer_duration_resolution_control;
    /**< Specifies the options to configure the resolution control for the timer duration.

         The effective resolution of timer duration is a function of the sample duration, which is dependent on the sample rate.
         This API provides the flexibility to control the sample rate that should be used to derive the sample duration.
         sample_duration_in_us is defined as 10^6/sample_rate.

         The timer duration resolution control has two modes of operation
         CONFIGURED_SAMPLE_RATE_MODE : the sample rate is derived from the sample rate configured in the PARAM_ID_RAT_MEDIA_FORMAT
         MINIMUM_SAMPLE_RATE_MODE    : the sample rate is set as the minimum sample rate supported by the RAT module.
         The minimum sample rate is also based on the family of the configured sample rate set in PARAM_ID_RAT_MEDIA_FORMAT
         For integer rate it would be 8000Hz and for fractional rate it is 11025Hz.

         For MIMO RAT,
         if different inputs of RAT have different sample rates (of the same family), then using minimum_sample_rate_mode is required.
         if all the inputs have same sample rate then configured_sample_rate_mode is desired.

         For SISO RAT, configured_sample_rate_mode is always desired

         By default, the timer_duration_resolution_control would be set to configured_sample_rate_mode to be backward compatible
         with existing functionality

	       @valuesbul
	        - 0 -- configured_sample_rate_mode - timer_duration_resolution is adjusted based on configured sample rate
	        - 1 -- minimum_sample_rate_mode - timer_duration_resolution is adjusted based on minimum supported sample rate
	         @tablebulletend */

	   /**< @h2xmle_description {Specifies the options to configure the resolution control for the timer duration.

         						 The effective resolution of timer duration is a function of the sample duration,
         						 which is dependent on the sample rate. This API provides the flexibility to control
         						 the sample rate that should be used to derive the sample duration.
                                 sample_duration_in_us is defined as 10^6/sample_rate.

         	 	 	 	 	 	 The timer duration resolution control has two modes of operation
         	 	 	 	 	 	 CONFIGURED_SAMPLE_RATE_MODE : the sample rate is derived from the sample rate configured in
         	 	 	 	 	 	                               the PARAM_ID_RAT_MEDIA_FORMAT
         	 	 	 	 	 	 MINIMUM_SAMPLE_RATE_MODE    : the sample rate is set as the minimum sample rate supported
         	 	 	 	 	 	                               by the RAT module.
         	 	 	 	 	 	 The minimum sample rate is also based on the family of the configured sample rate set
         	 	 	 	 	 	 in PARAM_ID_RAT_MEDIA_FORMAT. For integer rate it would be 8000Hz and for fractional rate it is 11025Hz.

         	 	 	 	 	 	 For MIMO RAT,
         	 	 	 	 	 	 if different inputs of RAT have different sample rates (of the same family), then using
         	 	 	 	 	 	 minimum_sample_rate_mode is required.
         	 	 	 	 	 	 if all the inputs have same sample rate then configured_sample_rate_mode is desired.

         	 	 	 	 	 	 For SISO RAT, configured_sample_rate_mode is always desired

         	 	 	 	 	 	 By default, the timer_duration_resolution_control would be set to configured_sample_rate_mode
         	 	 	 	 	 	 to be backward compatible with existing functionality
	                             }
	        @h2xmle_default     {RAT_TIMER_DURATION_RESOLUTION_CONTROL_CONFIGURED_SAMPLE_RATE_MODE}
	        @h2xmle_rangeList   {"CONFIGURED_SAMPLE_RATE_MODE"= RAT_TIMER_DURATION_RESOLUTION_CONTROL_CONFIGURED_SAMPLE_RATE_MODE;
	                             "MINIMUM_SAMPLE_RATE_MODE"= RAT_TIMER_DURATION_RESOLUTION_CONTROL_MINIMUM_SAMPLE_RATE_MODE
	                             }
   */
    uint32_t duration_in_time_us;
   /**< Specifies the configuration of timer duration in time (units of micro-seconds)
        The actual timer duration is would depend on the time resolution of the sample (sample_duration) and can be approximated as below

        Effective operating timer duration can be approximated as duration_in_time - ( duration_in_time % sample_duration),
        sample_duration is defined as 10^6/sample_rate, and sample_rate is controlled by the timer_duration_resolution_control configuration

        This value is used when timer_config_mode is set to RAT_TD_CONFIGURED_IN_TIME and this value is set to non-zero value.

        Supported range of values : 1000 to 40,000 us */

   /**< @h2xmle_description {specifies the configuration of timer duration in time (units of micro-seconds)    *
                             The actual timer duration would depend on the time resolution of the sample (sample_duration) and can be approximated as                                below

                             Effective operating timer duration can be approximated as duration_in_time - ( duration_in_time % sample_duration),
        					 sample_duration is defined as 10^6/sample_rate,
        					 and sample_rate is controlled by the timer_duration_resolution_control configuration

                             This value is used when timer_config_mode is set to RAT_TD_CONFIGURED_IN_TIME and this value is
                             set to non-zero value. Supported range of values : 1000 to 40,000 us
                             }
        @h2xmle_range       {RAT_TIMER_DURATION_TIME_IN_US_MIN_VALUE .. RAT_TIMER_DURATION_TIME_IN_US_MAX_VALUE}
        @h2xmle_default     {RAT_TIMER_DURATION_TIME_IN_US_MIN_VALUE}
   */

    uint32_t duration_in_samples;
   /**< Specifies the configuration of timer duration in units of samples. Time is derived based on static port configured sample rate by the RAT module

        This value is used when timer_config_mode is set to RAT_TD_CONFIGURED_IN_SAMPLES and this value is set to non-zero value.

        Supported range of values : Any value that would effectively map to time duration of 1000 to 40,000 us

        Note, if the time is less than 1000us, it would fail to set the configuration

        The effective duration in sample would be adjusted based on the following criteria
        lets define static port sample rate (fs_sp) and resolution control based sample rate as (fs_cfg)
        fs_ratio = fs_sp/fs_cfg
        effective duration_in_samples = (duration_in_samples/fs_ratio) * fs_ratio

        */

   /**< @h2xmle_description {specifies the configuration of timer duration in units of samples.
                             Time is derived based on static port configured sample rate by the RAT module.
                             This value is used when timer_config_mode is set to RAT_TD_CONFIGURED_IN_SAMPLES and this value is set to non-zero value.
                             Supported range of values : Any value that would effectively map to time duration of 1000 to 40,000 us
                             Note, if the time is less than 1000us, it would fail to set the configuration

        					 The effective duration in sample would be adjusted based on the following criteria
        					 lets define static port sample rate (fs_sp) and resolution control based sample rate as (fs_cfg)
        					 fs_ratio = fs_sp/fs_cfg
        					 effective duration_in_samples = (duration_in_samples/fs_ratio) * fs_ratio

        					}
        @h2xmle_range       {RAT_TIMER_DURATION_IN_SAMPLES_MIN_VALUE .. RAT_TIMER_DURATION_IN_SAMPLES_MAX_VALUE}
        @h2xmle_default     {RAT_TIMER_DURATION_IN_SAMPLES_DEFAULT_VALUE}
 */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_rat_timer_duration_config_t param_id_rat_timer_duration_config_t;

/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/
/** @ingroup ar_spf_mod_rat_macros
    Rate Adapted Timer module.

    @subhead4{Supported parameter IDs}
    - #PARAM_ID_RAT_MEDIA_FORMAT

    @subhead4{Supported input media format ID}
    - Data Format          : FIXED_POINT, IEC61937_PACKETIZED, IEC60958_PACKETIZED,
    -                        DSD_OVER_PCM, GENERIC_COMPRESSED, RAW_COMPRESSED @lstsp1
    - fmt_id               : Don't care @lstsp1
    - Sample Rates         : 8, 11.025, 12, 16, 22.05, 24, 32, 44.1, 48,
                             88.2, 96, 176.4, 192, 352.8, 384 kHz @lstsp1
    - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels) @lstsp1
    - Channel type         : Don't care @lstsp1
    - Bit Width            : 16 (bits per sample 16 and Q15),
                           : 24 (bits per sample 24 and Q27)
                           : 32 (bits per sample 32 and Q31)
    - Q format             : Q15, Q23, Q27, Q31 @lstsp1
    - Interleaving         : Module needs de-interleaved unpacked
*/
#define MODULE_ID_RATE_ADAPTED_TIMER 0x07001041
/** @h2xmlm_module             {"MODULE_ID_RATE_ADAPTED_TIMER",
                                 MODULE_ID_RATE_ADAPTED_TIMER}
   @h2xmlm_displayName         {"Rate Adapted Timer "}
   @h2xmlm_modSearchKeys       {Audio, Voice, Bluetooth}
   @h2xmlm_description         {- Rate Adapted Timer \n
                                 - Supports following params: \n
                                 - PARAM_ID_RAT_MEDIA_FORMAT \n
                                 - Supported Input Media Format: \n
                                 - Data Format          : FIXED_POINT, IEC61937_PACKETIZED, IEC60958_PACKETIZED, \n
                                 -                        DSD_OVER_PCM, GENERIC_COMPRESSED, RAW_COMPRESSED \n
                                 - fmt_id               : Don't care \n
                                 - Sample Rates         : 8, 11.025, 12, 16, 22.05, 24, 32, 44.1, 48, \n
                                 -                        88.2, 96, 176.4, 192, 352.8, 384 kHz \n
                                 - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels) \n
                                 - Channel type         : Don't care \n
                                 - Bit Width            : 16 (bits per sample 16 and Q15), \n
                                 -                      : 24 (bits per sample 24 and Q27) \n
                                 -                      : 32 (bits per sample 32 and Q31)	\n
                                 - Q format             : Q15, Q23, Q27, Q31 \n
                                 - Interleaving         : Module needs de-interleaved unpacked }

    @h2xmlm_dataInputPorts      {IN = PORT_ID_RATE_ADAPTED_TIMER_INPUT}
    @h2xmlm_dataOutputPorts     {OUT = PORT_ID_RATE_ADAPTED_TIMER_OUTPUT}
    @h2xmlm_dataMaxInputPorts   {1}
    @h2xmlm_dataMaxOutputPorts  {1}
    @h2xmlm_ctrlStaticPort      {"QT_REFCLK_TIMING_INPUT" = 0xC0000001,
                                 "Qtimer HW-EP drift info" = INTENT_ID_TIMER_DRIFT_INFO}
    @h2xmlm_ctrlStaticPort      {"REFCLK_REMOTE_TIMING_INPUT" = 0xC0000002,
                                 "HWEP BT drift info" = INTENT_ID_TIMER_DRIFT_INFO}
    @h2xmlm_ctrlStaticPort      {"QT_REMOTE_TIMING_INPUT" = 0xC0000003,
                                 "Qtimer Remote drift info" = INTENT_ID_TIMER_DRIFT_INFO}
    @h2xmlm_ctrlDynamicPortIntent  {"Qtimer to Adapted Rate drift info"=INTENT_ID_TIMER_DRIFT_INFO, maxPorts= -1 }
    @h2xmlm_ctrlDynamicPortIntent  { "HW EP Module Instance info"=INTENT_ID_MODULE_INSTANCE_INFO, maxPorts= -1 }
    @h2xmlm_supportedContTypes { APM_CONTAINER_TYPE_GC }
    @h2xmlm_isOffloadable       {false}
    @h2xmlm_stackSize           { RATE_ADAPTED_TIMER_STACK_SIZE }
    @{                          <-- Start of the Module -->

    @h2xml_Select		        {param_id_rat_mf_t}
    @h2xmlm_InsertParameter
    @}                   <-- End of the Module -->*/

/** @ingroup ar_spf_mod_rat_macros
    Multiple-Input Multiple-Output (MIMO) Rate Adapted Timer module.
    - MIMO RAT has one static input and one static output port. It has
      infinite dynamic input and output ports.
    - It has a one to one mapping between input and output ports i.e. only
      one input corresponds to only one output.
    - Static output port needs mandatory media format config via
      PARAM_ID_RAT_MEDIA_FORMAT.
    - As far as input data is concerned, it behaves as a pass through
      module.
    - If input is not available, if the static output port of MIMO RAT is
      connected and started and media format is configured, RAT will pump zeroes at the output.
    - If input is not available, if dynamic output port of MIMO RAT is
      connected and started, it will not send any output until input media format is received.
      Once input media format is received, it will send silence data with this format if there is
      no input available.
    - RAT can only support one family of rates - either ineger or fractional.
    - If mandatory static port config has integer sample rate then all ports must be integer
      family of rates.

    @subhead4{Supported parameter IDs}
    - #PARAM_ID_RAT_MEDIA_FORMAT

    @subhead4{Supported input media format ID}
    - Data Format          : FIXED_POINT, IEC61937_PACKETIZED, IEC60958_PACKETIZED,
                             DSD_OVER_PCM, GENERIC_COMPRESSED, RAW_COMPRESSED @lstsp1
    - fmt_id               : Don't care @lstsp1
    - Sample Rates         : 8, 11.025, 12, 16, 22.05, 24, 32, 44.1, 48,
                             88.2, 96, 176.4, 192, 352.8, 384 kHz @lstsp1
    - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels) @lstsp1
    - Channel type         : Don't care @lstsp1
    - Bit Width            : 16 (bits per sample 16 and Q15),
                           : 24 (bits per sample 24 and Q27)
                           : 32 (bits per sample 32 and Q31) @lstsp1
    - Q format             : Q15, Q23, Q27, Q31 @lstsp1
    - Interleaving         : Module needs de-interleaved unpacked or interleaved.
*/
#define MODULE_ID_MIMO_RATE_ADAPTED_TIMER 0x07001170
/** @h2xmlm_module             {"MODULE_ID_MIMO_RATE_ADAPTED_TIMER",
                                 MODULE_ID_MIMO_RATE_ADAPTED_TIMER}
   @h2xmlm_displayName         {"MIMO Rate Adapted Timer "}
   @h2xmlm_modSearchKeys       {Audio, Voice, Bluetooth}
   @h2xmlm_description         {- Multiple-Input Multiple-Output Rate Adapted Timer \n
                                                 - MIMO RAT has one static input and one static output port. It has
   infinite dynamic input and output ports. \n
                                                 - It has a one to one mapping between input and output ports i.e. only
   one input corresponds to only one output \n
                                                 - Static output port needs mandatory media format config via
   PARAM_ID_RAT_MEDIA_FORMAT.\n
                                                 - As far as input data is concerned, it behaves as a pass through
   module.\n
                                                 - If input is not available, if the static output port of MIMO RAT is
   connected and started \n
                                                   and media format is configured, RAT will pump zeroes at the output.\n
                                                 - If input is not available, if dynamic output port of MIMO RAT is
   connected and started, \n
                                                   it will not send any output until input media format is received.
   Once input media format \n
                                                   is received, it will send silence data with this format if there is
   no input available. \n
                          - RAT can only support one family of rates - either ineger or fractional.
                          - If mandatory static port config has integer sample rate then all ports must be integer
   family of rates.

                                 - Supports following params: \n
                                 - PARAM_ID_RAT_MEDIA_FORMAT \n
                                 - Supported Input Media Format: \n
                                 - Data Format          : FIXED_POINT, IEC61937_PACKETIZED, IEC60958_PACKETIZED, \n
                                 -                        DSD_OVER_PCM, GENERIC_COMPRESSED, RAW_COMPRESSED \n
                                 - fmt_id               : Don't care \n
                                 - Sample Rates         : 8, 11.025, 12, 16, 22.05, 24, 32, 44.1, 48, \n
                                 -                        88.2, 96, 176.4, 192, 352.8, 384 kHz \n
                                 - Number of channels   : 1 to 128 (for certain products this module supports only 32 channels) \n
                                 - Channel type         : Don't care \n
                                 - Bit Width            : 16 (bits per sample 16 and Q15), \n
                                 -                      : 24 (bits per sample 24 and Q27) \n
                                 -                      : 32 (bits per sample 32 and Q31)	\n
                                 - Q format             : Q15, Q23, Q27, Q31 \n
                                 - Interleaving         : Module needs de-interleaved unpacked or interleaved}

    @h2xmlm_dataInputPorts      {IN = PORT_ID_RATE_ADAPTED_TIMER_INPUT}
    @h2xmlm_dataOutputPorts     {OUT = PORT_ID_RATE_ADAPTED_TIMER_OUTPUT}
    @h2xmlm_dataMaxInputPorts   {INFINITE}
    @h2xmlm_dataMaxOutputPorts  {INFINITE}
    @h2xmlm_ctrlStaticPort      {"QT_REFCLK_TIMING_INPUT" = 0xC0000001,
                                 "Qtimer HW-EP drift info" = INTENT_ID_TIMER_DRIFT_INFO}
    @h2xmlm_ctrlStaticPort      {"REFCLK_REMOTE_TIMING_INPUT" = 0xC0000002,
                                 "HWEP BT drift info" = INTENT_ID_TIMER_DRIFT_INFO}
    @h2xmlm_ctrlStaticPort      {"QT_REMOTE_TIMING_INPUT" = 0xC0000003,
                                 "Qtimer Remote drift info" = INTENT_ID_TIMER_DRIFT_INFO}
    @h2xmlm_ctrlDynamicPortIntent  {"Qtimer to Adapted Rate drift info"=INTENT_ID_TIMER_DRIFT_INFO, maxPorts= -1 }
    @h2xmlm_ctrlDynamicPortIntent  { "HW EP Module Instance info"=INTENT_ID_MODULE_INSTANCE_INFO, maxPorts= -1 }
    @h2xmlm_supportedContTypes { APM_CONTAINER_TYPE_GC }
    @h2xmlm_isOffloadable       {false}
    @h2xmlm_stackSize           { RATE_ADAPTED_TIMER_STACK_SIZE }
    @{                          <-- Start of the Module -->

    @h2xml_Select		        {param_id_rat_mf_t}
    @h2xmlm_InsertParameter
    @h2xml_Select		        {param_id_module_data_interleaving_t}
    @h2xmlm_InsertParameter
    @}                   <-- End of the Module -->*/

#endif //_RATE_ADAPTED_TIMER_API_H_
