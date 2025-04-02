#ifndef API_DRC_H
#define API_DRC_H
/*==============================================================================
  @file drc_api.h
  @brief This file contains DRC APIs
==============================================================================*/
/*=======================================================================
* Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
* Confidential and Proprietary, Qualcomm Technologies, Inc.
=========================================================================*/

/*==============================================================================
   Include Files
==============================================================================*/

#include "module_cmn_api.h"
#include "imcl_spm_intent_api.h"

/*==============================================================================
   Constants
==============================================================================*/

/* Stack size of DRC module */
#define DRC_STACK_SIZE 2048


/*==============================================================================
   Param ID
==============================================================================*/

#define PARAM_ID_DRC_CONFIG  0x0800113E
/** @h2xmlp_parameter   {"PARAM_ID_DRC_CONFIG", PARAM_ID_DRC_CONFIG}
    @h2xmlp_description {DRC config parameters.}
    @h2xmlp_toolPolicy  {Calibration; RTC} */

typedef struct param_id_drc_config_t param_id_drc_config_t;

#include "spf_begin_pack.h"
struct param_id_drc_config_t
{
    uint16_t mode;
    /**< @h2xmle_description {Enable/disables the DRC feature. In Bypass mode, only delay is applied.}
         @h2xmle_rangeList   {"Bypass";0, "Enable";1}
         @h2xmle_default     {1}
         @h2xmle_policy      {Basic}*/

    uint16_t channel_linked_mode;
    /**< @h2xmle_description {Specifies whether all channels have the same applied dynamics(Linked)
        					  or if they process their dynamics independently(Unlinked).}
         @h2xmle_rangeList   {"CHANNEL_UNLINKED";0, "CHANNEL_LINKED";1}
         @h2xmle_default     {0}
         @h2xmle_policy      {Basic}*/

    uint16_t rms_time_avg_const;
    /**< @h2xmle_description {Time-averaging constant for computing energy}
         @h2xmle_range       {0..65535}
		 @h2xmle_dataFormat  {Q16}
		 @h2xmle_default     {299}
         @h2xmle_policy      {Basic} */

    uint16_t makeup_gain;
    /**< @h2xmle_description {Makeup gain in dB applied after DRC.}
         @h2xmle_range       {0..65535}
		 @h2xmle_dataFormat  {Q12}
		 @h2xmle_default     {4096}
		 @h2xmle_displayType {dbtextbox}
         @h2xmle_policy      {Basic} */

    int16_t down_expdr_threshold;
    /**< @h2xmle_description {Downward expander threshold in dB}
         @h2xmle_range       {-32768..32767}
         @h2xmle_dataFormat  {Q7}
         @h2xmle_default     {3239}
         @h2xmle_policy      {Basic} */

    int16_t down_expdr_slope;
    /**< @h2xmle_description {Downward expander slope.}
         @h2xmle_range       {-32768..32767}
         @h2xmle_dataFormat  {Q8}
         @h2xmle_default     {-383}
         @h2xmle_policy      {Basic} */

    uint32_t down_expdr_attack;
    /**< @h2xmle_description {Down expander attack constant.}
         @h2xmle_range       {0..2147483648}
		 @h2xmle_dataFormat  {Q31}
		 @h2xmle_default     {1604514739}
         @h2xmle_policy      {Basic} */

    uint32_t down_expdr_release;
    /**< @h2xmle_description {Down expander release constant.}
         @h2xmle_range       {0..2147483648}
		 @h2xmle_dataFormat  {Q31}
		 @h2xmle_default     {1604514739}
         @h2xmle_policy      {Basic} */

    int32_t down_expdr_min_gain_db;
    /**< @h2xmle_description {Downward expander minimum gain in dB.}
         @h2xmle_range       {-2147483648..2147483647}
		 @h2xmle_dataFormat  {Q23}
		 @h2xmle_default     {-50331648}
         @h2xmle_policy      {Basic} */

    uint16_t down_expdr_hysteresis;
    /**< @h2xmle_description {Down expander hysteresis constant.}
         @h2xmle_range       {0..65535}
		 @h2xmle_dataFormat  {Q14}
		 @h2xmle_default     {16384}
         @h2xmle_policy      {Basic} */

    int16_t up_cmpsr_threshold;
    /**< @h2xmle_description {Upward compressor threshold.}
         @h2xmle_range       {-32768..32767}
		 @h2xmle_dataFormat  {Q7}
		 @h2xmle_default   	 {3239}
         @h2xmle_policy      {Basic} */

    uint32_t up_cmpsr_attack;
    /**< @h2xmle_description {Up compressor attack constant.}
         @h2xmle_range       {0..2147483648}
	     @h2xmle_dataFormat  {Q31}
	     @h2xmle_default     {5897467}
         @h2xmle_policy      {Basic} */

    uint32_t up_cmpsr_release;
    /**< @h2xmle_description {Up compressor release constant.}
         @h2xmle_range       {0..2147483648}
	     @h2xmle_dataFormat  {Q31}
	     @h2xmle_default     {5897467}
         @h2xmle_policy      {Basic} */

    uint16_t up_cmpsr_slope;
    /**< @h2xmle_description {Upward compression slope.}
         @h2xmle_range       {0..65535}
		 @h2xmle_dataFormat  {Q16}
		 @h2xmle_default     {0}
         @h2xmle_policy      {Basic} */

    uint16_t up_cmpsr_hysteresis;
    /**< @h2xmle_description {Up compressor hysteresis constant.}
         @h2xmle_range       {0..65535}
		 @h2xmle_dataFormat  {Q14}
		 @h2xmle_default     {18855}
         @h2xmle_policy      {Basic} */

    int16_t down_cmpsr_threshold;
    /**< @h2xmle_description {Downward compressor threshold.}
         @h2xmle_range       {-32768..32767}
		 @h2xmle_dataFormat  {Q7}
		 @h2xmle_default     {9063}
         @h2xmle_policy      {Basic} */

    uint16_t down_cmpsr_slope;
    /**< @h2xmle_description {Downward compression slope.}
         @h2xmle_range       {0..65535}
		 @h2xmle_dataFormat  {Q16}
		 @h2xmle_default     {64879}
         @h2xmle_policy      {Basic} */

    uint32_t down_cmpsr_attack;
    /**< @h2xmle_description {Down compressor attack constant.}
         @h2xmle_range       {0..2147483648}
		 @h2xmle_dataFormat  {Q31}
		 @h2xmle_default     {1604514739}
         @h2xmle_policy      {Basic} */

    uint32_t down_cmpsr_release;
    /**< @h2xmle_description {Down compressor release constant.}
         @h2xmle_range       {0..2147483648}
		 @h2xmle_dataFormat  {Q31}
		 @h2xmle_default     {1604514739}
         @h2xmle_policy      {Basic} */

    uint16_t down_cmpsr_hysteresis;
    /**< @h2xmle_description {Down compressor hysteresis constant.}
         @h2xmle_range       {0..65535}
		 @h2xmle_dataFormat  {Q14}
		 @h2xmle_default     {16384}
         @h2xmle_policy      {Basic} */

    uint16_t down_sample_level;
    /**< @h2xmle_description {DRC down sample level.}
         @h2xmle_range       {1..16}
         @h2xmle_default     {1}
         @h2xmle_displayType {slider}
         @h2xmle_policy      {Basic} */

    uint32_t delay_us;
    /**< @h2xmle_description {DRC delay in microseconds.}
         @h2xmle_range       {0..100000}
         @h2xmle_default     {5000}
         @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;

/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/

#define MODULE_ID_DRC  0x07001066
/**
    @h2xmlm_module       {"MODULE_ID_DRC", MODULE_ID_DRC}
    @h2xmlm_displayName  {"Dynamic Range Control"}
    @h2xmlm_modSearchKeys{drc, Audio, Voice}
	@h2xmlm_description  { - Dynamic Range Control Algorithm.
						   - This module supports the following parameter IDs:\n
						   - #PARAM_ID_MODULE_ENABLE\n
						   - #PARAM_ID_DRC_CONFIG\n
						   - Supported Input Media Format: \n
                           - Data Format          : FIXED \n
                           - fmt_id               : PCM \n
                           - Sample Rates         : Don't care \n
                           - Number of channels   : 1...32 \n
                           - Channel type         : Don't care \n
                           - Bits per sample      : 16, 32 \n
                           - Q format             : Q15, Q27 \n
                           - Interleaving         : De-interleaved unpacked \n
                           - Signed/unsigned      : Signed \n}
    @h2xmlm_dataMaxInputPorts   {1}
    @h2xmlm_dataMaxOutputPorts  {1}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_SC, APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable       {true}
    @h2xmlm_stackSize           {DRC_STACK_SIZE}
    @h2xmlm_ctrlDynamicPortIntent  { "DRC DOWN COMP threshold" = INTENT_ID_DRC_CONFIG, maxPorts= 1 }
    @h2xmlm_toolPolicy           { Calibration }

    @{                   <-- Start of the Module -->

    @h2xml_Select					{param_id_module_enable_t}
    @h2xmlm_InsertParameter

    @h2xml_Select					{param_id_drc_config_t}
    @h2xmlm_InsertParameter

    @h2xml_Select       {param_id_drc_config_t::rms_time_avg_const}

    @h2xmle_defaultDependency   {   Samplerate = "8000",  default = "598"}
    @h2xmle_defaultDependency   {   Samplerate = "16000",  default = "299"}
    @h2xmle_defaultDependency   {   Samplerate = "32000",  default = "1484"}
    @h2xmle_defaultDependency   {   Samplerate = "48000",  default = "993"}

    @h2xml_Select       {param_id_drc_config_t::down_expdr_attack}

    @h2xmle_defaultDependency   {   Samplerate = "8000",  default = "2010199605"}
    @h2xmle_defaultDependency   {   Samplerate = "16000",  default = "1604514739"}
    @h2xmle_defaultDependency   {   Samplerate = "32000",  default = "1067661045"}
    @h2xmle_defaultDependency   {   Samplerate = "48000",  default = "789550996"}

    @h2xml_Select       {param_id_drc_config_t::down_expdr_release}

    @h2xmle_defaultDependency   {   Samplerate = "8000",  default = "2010199605"}
    @h2xmle_defaultDependency   {   Samplerate = "16000",  default = "1604514739"}
    @h2xmle_defaultDependency   {   Samplerate = "32000",  default = "1067661045"}
    @h2xmle_defaultDependency   {   Samplerate = "48000",  default = "789550996"}

    @h2xml_Select       {param_id_drc_config_t::up_cmpsr_attack}

    @h2xmle_defaultDependency   {   Samplerate = "8000",  default = "5897467"}
    @h2xmle_defaultDependency   {   Samplerate = "16000",  default = "5897467"}
    @h2xmle_defaultDependency   {   Samplerate = "32000",  default = "2950761"}
    @h2xmle_defaultDependency   {   Samplerate = "48000",  default = "1967625"}

    @h2xml_Select       {param_id_drc_config_t::up_cmpsr_release}

    @h2xmle_defaultDependency   {   Samplerate = "8000",  default = "5897468"}
    @h2xmle_defaultDependency   {   Samplerate = "16000",  default = "5897467"}
    @h2xmle_defaultDependency   {   Samplerate = "32000",  default = "2950761"}
    @h2xmle_defaultDependency   {   Samplerate = "48000",  default = "1967625"}

    @h2xml_Select       {param_id_drc_config_t::down_cmpsr_attack}

    @h2xmle_defaultDependency   {   Samplerate = "8000",  default = "2010199605"}
    @h2xmle_defaultDependency   {   Samplerate = "16000",  default = "1604514739"}
    @h2xmle_defaultDependency   {   Samplerate = "32000",  default = "1067661045"}
    @h2xmle_defaultDependency   {   Samplerate = "48000",  default = "789550996"}

    @h2xml_Select       {param_id_drc_config_t::down_cmpsr_release}

    @h2xmle_defaultDependency   {   Samplerate = "8000",  default = "2010199605"}
    @h2xmle_defaultDependency   {   Samplerate = "16000",  default = "1604514739"}
    @h2xmle_defaultDependency   {   Samplerate = "32000",  default = "1067661045"}
    @h2xmle_defaultDependency   {   Samplerate = "48000",  default = "789550996"}

@}                   <-- End of the Module -->*/



#endif
