#ifndef _DETECTION_CMN_API_H_
#define _DETECTION_CMN_API_H_
/**
 * \file detection_cmn_api.h
 * \brief 
 *  	 This file contains common parameters for listen libraries
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "module_cmn_api.h"

/*------------------------------------------------------------------------------
   Parameters
------------------------------------------------------------------------------*/
/*==============================================================================
   Constants
==============================================================================*/

#define DETECTION_ENGINE_MAX_REGISTERED_MODELS 20

/*Detection Statuses*/ 

/** Status (Detected) sent to notify that the detection engine has
    detected a keyword.
 */
#define DETECTION_STATUS_SUCCESS 0

/** Status (Rejected) sent to notify the that a detection module has
    rejected a keyword.
.*/
#define DETECTION_STATUS_FAILURE 1

/* Maximum Number of Models that are
 * supported for this Module
 */
#define MAX_MODELS_SUPPORTED 8
/* Maximum Number of keyword per model that are
 * supported for this Module
 */
#define MAX_KEYWORDS_SUPPORTED 8

/* Unique Paramter id */
#define PARAM_ID_DETECTION_ENGINE_CONFIG 0x08001049

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure definition for Parameter */

/** @h2xmlp_parameter   {"PARAM_ID_DETECTION_ENGINE_CONFIG",
                         PARAM_ID_DETECTION_ENGINE_CONFIG}
    @h2xmlp_description {Configures the Detection Engine module.}
    @h2xmlp_toolPolicy  {Calibration} */
#include "spf_begin_pack.h"
struct param_id_detection_engine_config_t
{
   uint16_t mode;
   /**< @h2xmle_description {Flags that configure the Detection Engine module to
                             run in different modes.}

        @h2xmle_bitField    {0x00000001}
        @h2xmle_bitName     {detectionMode}
        @h2xmle_description {Keyword/Pattern Detection mode.}
        @h2xmle_rangeList   {Disabled=0;Enabled=1}
        @h2xmle_bitFieldEnd

        @h2xmle_bitField    {0x00000002}
        @h2xmle_bitName     {verificationMode}
        @h2xmle_description {User Verification mode.}
        @h2xmle_rangeList   {Disabled=0;Enabled=1}
        @h2xmle_bitFieldEnd

        @h2xmle_bitField    {0x00000004}
        @h2xmle_bitName     {eventMode}
        @h2xmle_description {Detection events are sent to the HLOS.}
        @h2xmle_rangeList   {Success=0;Always=1}
        @h2xmle_bitFieldEnd

        @h2xmle_default     {1} */

   uint16_t custom_payload_size;
   /**< @h2xmle_description {Size of of additional custom configuration sent to
                             the Detection Engine module. \n
                             \n
                             If custom_payload_size is not 4-byte aligned, the
                             caller must fill padding bytes to ensure that the
                             entire calibration is 4-byte aligned. \n
                             \n
                             Immediately following this variable is the custom
                             configuration payload of size custom_payload_size.}
        @h2xmle_range       {0..0xFFFF}
        @h2xmle_default     {0} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_detection_engine_config_t param_id_detection_engine_config_t;

/* Unique Paramter id */
#define PARAM_ID_DETECTION_GAIN 0x0800104B

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure definition for Parameter */

/** @h2xmlp_parameter   {"PARAM_ID_DETECTION_GAIN",
                         PARAM_ID_DETECTION_GAIN}
    @h2xmlp_description {Sets the gain that is applied to incoming audio data.}
    @h2xmlp_toolPolicy  {Calibration} */

#include "spf_begin_pack.h"
struct param_id_detection_gain_t
{
   int16_t gain;
   /**< @h2xmle_description {Gain that is applied to the data.}
        @h2xmle_default     {0x100}
        @h2xmle_dataFormat  {Q8} 
        @h2xmle_policy       {Basic}*/

   uint16_t reserved;
   /**< @h2xmle_description {Used for alignment; must be set to 0.}
        @h2xmle_readonly    {true}
        @h2xmle_default     {0} 
        @h2xmle_policy       {Basic}*/
}
#include "spf_end_pack.h"
;
typedef struct param_id_detection_gain_t param_id_detection_gain_t;

/* Unique Parameter id */
#define PARAM_ID_DETECTION_ENGINE_SOUND_MODEL 0x0800104C
/*==============================================================================
   Type definitions
==============================================================================*/

/** @h2xmlp_emptyParameter {"PARAM_ID_DETECTION_ENGINE_SOUND_MODEL",
                            PARAM_ID_DETECTION_ENGINE_SOUND_MODEL}
    @h2xmlp_description    {Registers a sound model with a module. \n
           \n
           -The sound model is a blob of data interpreted by the module that
           supports this parameter. The model contains unique sound
           characteristics and signatures that are used by the algorithm hosted
           in the module. \n
           \n
           -This paramter must be registered by using the
            APM_CMD_REGISTER_CFG with the sound model in a persistent
            payload only. \n
           \n
           -All calibration parameters start from a 4-byte aligned address.
           This parameter for this module has an additional 8-byte alignment
           requirement because it performs vector processing directly on the
           data.} 
    @h2xmlp_toolPolicy {Calibration} */
	
/* Unique Parameter id */
#define PARAM_ID_DETECTION_ENGINE_REGISTER_MULTI_SOUND_MODEL 0x0800124E
/*==============================================================================
   Type definitions
==============================================================================*/

/** @h2xmlp_Parameter      {"PARAM_ID_DETECTION_ENGINE_REGISTER_MULTI_SOUND_MODEL",
                             PARAM_ID_DETECTION_ENGINE_REGISTER_MULTI_SOUND_MODEL}
    @h2xmlp_description    {Registers a sound model of a specific model id with a module. \n
           \n
           -The sound model is a blob of data interpreted by the module that
           supports this parameter. The model contains unique sound
           characteristics and signatures that are used by the algorithm hosted
           in the module. \n
           \n
           -This paramter must be registered by using the
            APM_CMD_REGISTER_CFG with the sound model in a persistent
            payload and using APM_CMD_SET_CFG in a non-persistent payload \n
           \n
           -All calibration parameters start from a 4-byte aligned address.
           This parameter for this module has an additional 256-byte alignment 
		       requirement if MLA is enabled and 8 byte alignment if MLA is disabled.}
    @h2xmlp_toolPolicy {Calibration} */	

/* Structure definition for Parameter */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_detection_engine_register_multi_sound_model_t
{
  uint32_t model_id;
  /**< @h2xmle_description {This is an unique number to differentiate
                            between different sound models to be loaded on DSP}
		@h2xmle_range          {0..0xFFFFFFFE}					
        @h2xmle_default    {0x0}*/

  uint32_t model_size;
  /**< @h2xmle_description {Indicated the size of the sound model.
                            Sound model payload follows this variable}
		@h2xmle_range          {0..0xFFFFFFFF}					
        @h2xmle_default    {0x0}*/

  uint8_t model[0];
  /**< @h2xmle_description {Sound Model Data Blob which is interpreted by the Library
							              This Sound Model blob must be 256 bytes aligned if MLA is used
							              This Sound Model blob must be 8byte aligned if MLA is not used.}
     @h2xmle_default       {0}	*/	
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_detection_engine_register_multi_sound_model_t param_id_detection_engine_register_multi_sound_model_t;

/* Unique Parameter id */
#define PARAM_ID_DETECTION_ENGINE_DEREGISTER_MULTI_SOUND_MODEL 0x08001250
/*==============================================================================
   Type definitions
==============================================================================*/

/** @h2xmlp_Parameter      {"PARAM_ID_DETECTION_ENGINE_DEREGISTER_MULTI_SOUND_MODEL",
                             PARAM_ID_DETECTION_ENGINE_DEREGISTER_MULTI_SOUND_MODEL}
    @h2xmlp_description    {Deregisters a sound model of a specific model id with a module. \n
           \n
           -This paramter must be used to deregister a sound model \n}
    @h2xmlp_persistType  { Shared }      
    @h2xmlp_toolPolicy {NO_SUPPORT} */  

/* Structure definition for Parameter */

#include "spf_begin_pack.h"
struct param_id_detection_engine_deregister_multi_sound_model_t
{
  uint32_t model_id;
  /**< @h2xmle_description {This is an unique number to differentiate
                            between different sound models to be unloaded from DSP}
    @h2xmle_range          {0..0xFFFFFFFE}         
        @h2xmle_default    {0x0}*/
}
#include "spf_end_pack.h"
;
typedef struct param_id_detection_engine_deregister_multi_sound_model_t param_id_detection_engine_deregister_multi_sound_model_t;


/* Unique Parameter id */
#define PARAM_ID_BEST_CH_ENABLE 0x08001065

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure definition for Parameter */

/** @h2xmlp_parameter   {"PARAM_ID_BEST_CH_ENABLE", PARAM_ID_BEST_CH_ENABLE}
    @h2xmlp_description {Enables the reporting of best channel to DAM Module.}
    @h2xmlp_toolPolicy   {Calibration} */
#include "spf_begin_pack.h"
struct param_id_best_ch_enable_t
{
   uint32_t enable;
   /**< @h2xmle_description {Enables or disables best ch reporting.}
        @h2xmle_rangeList   {Disabled=0; Enabled=1}
        @h2xmle_default     {0}
        @h2xmle_policy       {Basic}*/
}
#include "spf_end_pack.h"
;
typedef struct param_id_best_ch_enable_t param_id_best_ch_enable_t;

/*==============================================================================
   Constants
==============================================================================*/

/* Unique Parameter id */
#define PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG 0x0800104E

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure definition for Parameter */
/** @h2xmlp_parameter   {"PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG",
                          PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG}
    @h2xmlp_description { Configures the payload of the detection events. }
  @h2xmlp_toolPolicy  {Calibration}*/
#include "spf_begin_pack.h"
struct param_id_detection_engine_generic_event_cfg_t
{
   uint32_t event_mode;
   /**< @h2xmle_description {Bit field used to indicate which type of information
                             is to be embedded in the event. Set this field only
                             if the event_type variable is set to 1.}

        @h2xmle_bitField    {0x00000001}
        @h2xmle_bitName     {confidence_level_information}
        @h2xmle_description {Includes confidence level information in detection
                             events.}
        @h2xmle_rangeList   {Disabled=0;Enabled=1}
        @h2xmle_bitFieldEnd

        @h2xmle_bitField    {0x00000002}
        @h2xmle_bitName     {keyword_indices_information}
        @h2xmle_description {Includes keyword index information in detection
                             events from the detection engine.}
        @h2xmle_rangeList   {Disabled=0;Enabled=1}
        @h2xmle_bitFieldEnd

        @h2xmle_bitField    {0x00000004}
        @h2xmle_bitName     {detection_timestamp}
        @h2xmle_description {Includes detection timestamp info in the detection event payload.}
        @h2xmle_rangeList   {Disabled=0;Enabled=1}
        @h2xmle_bitFieldEnd

        @h2xmle_bitField    {0x00000008}
        @h2xmle_bitName     {ftrt_data_information}
        @h2xmle_description {Includes ftrt data length information in the detection
                             event payload.}
        @h2xmle_rangeList   {Disabled=0;Enabled=1}
        @h2xmle_bitFieldEnd
        
        @h2xmle_bitField    {0x00000010}
        @h2xmle_bitName     {best_channel_index_information}
        @h2xmle_description {Includes best channel index information in detection events
                             from the Engine}
        @h2xmle_rangeList   {Disabled=0;Enabled=1}
        @h2xmle_bitFieldEnd
      
        @h2xmle_bitField    {0x00000020}
        @h2xmle_bitName     {voice_ui_multi_model_result_info_t}
        @h2xmle_description {Includes result information in detection events
                             from the SVA5 Engine}
        @h2xmle_rangeList   {Disabled=0;Enabled=1}
        @h2xmle_bitFieldEnd
		
      @h2xmle_default      {1}*/
}
#include "spf_end_pack.h"
;
typedef struct param_id_detection_engine_generic_event_cfg_t param_id_detection_engine_generic_event_cfg_t;

/*==============================================================================
   Constants
==============================================================================*/

/* Unique Parameter id */
#define EVENT_ID_DETECTION_ENGINE_GENERIC_INFO 0x0800104F

/*==============================================================================
   Type definitions
==============================================================================*/

/** @h2xmlp_parameter   {"EVENT_ID_DETECTION_ENGINE_GENERIC_INFO",
                          EVENT_ID_DETECTION_ENGINE_GENERIC_INFO}
    @h2xmlp_description { Detection event raised by the Detection Engine module.}
    @h2xmlp_toolPolicy  {NO_SUPPORT}*/

#include "spf_begin_pack.h"
struct event_id_detection_engine_generic_info_t
{
   uint16_t status;
   /**< Status sent by the Detection Engine module to the client.

        @values
        - #DETECTION_STATUS_SUCCESS
        - #DETECTION_STATUS_FAILURE @tablebulletend */

   uint16_t payload_size;
   /**< Actual size of the parameter data payload that follows this structure.

        @values > 0 bytes */
}
#include "spf_end_pack.h"
;
typedef struct event_id_detection_engine_generic_info_t event_id_detection_engine_generic_info_t;

/** IDs for keyword index information and confidence value information in
    the detection engine generic event payload.
    structure which contains key_id and actual payload size of each key.
 */
typedef enum detection_engine_event_key_id_info_t
{
   KEY_ID_CONFIDENCE_LEVELS_INFO = 0x2000, /**< Confidence levels detected per sound model. */
   KEY_ID_KWD_POSITION_INFO,               /**< Keyword start and end timestamps. */
   KEY_ID_TIMESTAMP_INFO,                  /**< Detetion timestamp info */
   KEY_ID_FTRT_DATA_INFO,                  /**< FTRT Data length in the circular buffer.  */
   KEY_ID_BEST_CH_IDX_INFO,                /**< Best Ch Idx detected by the algorithm  */
   KEY_ID_VOICE_UI_MULTI_MODEL_RESULT_INFO,     /**< Multi Model Detection Engine result info */   
   KEY_ID_MAX_VALUE =  0x7FFFFFFF          /**< Maxium supported value for KEY_ID.  */
}detection_engine_event_key_id_info_t;

/* subStruct */
#include "spf_begin_pack.h"
/** @weakgroup weak_detection_event_info_header_t
@{ */
/* detection event header info structure */
struct detection_event_info_header_t
{
   detection_engine_event_key_id_info_t key_id;
   /**< Different key_ids
        @values
        - #KEY_ID_CONFIDENCE_LEVELS_INFO
        - #KEY_ID_KWD_POSITION_INFO
        - #KEY_ID_TIMESTAMP_INFO
        - #KEY_ID_FTRT_DATA_INFO 
        - #KEY_ID_BEST_CH_IDX_INFO 
	- #KEY_ID_VOICE_UI_MULTI_MODEL_RESULT_INFO @tablebulletend */

   uint32_t payload_size;
   /**< Size of a specific key payload.

        @values 0..0xFFFFFFFF (Default is 0) */
}
#include "spf_end_pack.h"
;
typedef struct detection_event_info_header_t detection_event_info_header_t;

/* Payload associated with the KEY_ID_CONFIDENCE_LEVELS_INFO type*/
/** @h2xmlp_subStruct */

/** @h2xmlp_description {Information about confidence levels. Used when
                         confidence level information is expected in a generic
                         detection engine event. To get the information, the
                         mode field in PARAM_ID_DETECTION_ENGINE_EVENT_MODE
                         must be set accordingly.} */
struct confidence_level_info_t
{
   uint32_t number_of_confidence_values;
   /**< @h2xmle_description {Sum of all keyword models plus the active user
                             models.}
        @h2xmle_range        {1..20}
        @h2xmle_default      {1} */

   uint8_t confidence_levels[DETECTION_ENGINE_MAX_REGISTERED_MODELS];
   /**< @h2xmle_description {Array of confidence levels for each keyword and
                             active user pair. \n
                             \n
                             This array is of size DETECTION_ENGINE_MAX_REGISTERED_MODELS.
                             Each element is 1 byte long. \n
                             \n
                             Assuming N keywords and M active user models are
                             included in the model, the first N elements of
                             this parameter correspond to the confidence of N
                             keywords. The N+1 to N+M elements control the
                             confidence level of the M active user model. Each
                             element maps confidence values for entries (keyword
                             or user specific keyword) whose order is specified
                             in the sound model.} */
};
typedef struct confidence_level_info_t confidence_level_info_t;

/* Payload associated with the KEY_ID_POSITION_INFO type */

/** @h2xmlp_subStruct
    @h2xmlp_toolPolicy   {NO_SUPPORT} */
/**<@h2xmlp_description {Statistics for keyword positions. Used when keyword
                         index information is expected in a generic detection
                         engine event. To get the information, the mode field in
                         PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG must be set
                         accordingly} */
struct keyword_position_info_t
{
   uint32_t kw_start_timestamp_lsw;
   /**< @h2xmle_description {Lower 32 bits of the 64-bit Keyword start time stamp. (in us).}
        @h2xmle_range       {0..4294967295}
        @h2xmle_default     {0} */

   uint32_t kw_start_timestamp_msw;
   /**< @h2xmle_description {Upper 32 bits of the 64-bit Keyword start time stamp. (in us).}
        @h2xmle_range       {0..4294967295}
        @h2xmle_default     {0} */

   uint32_t kw_end_timestamp_lsw;
   /**< @h2xmle_description {Lower 32 bits of the 64-bit Keyword end time stamp. (in us).}
        @h2xmle_range       {0..4294967295}
        @h2xmle_default     {0} */

   uint32_t kw_end_timestamp_msw;
   /**< @h2xmle_description {Upper 32 bits of the 64-bit Keyword end time stamp. (in us).}
        @h2xmle_range       {0..4294967295}
        @h2xmle_default     {0} */
};
typedef struct keyword_position_info_t keyword_position_info_t;
/* Payload associated with the KEY_ID_TIMESTAMP_INFO type*/

/** @h2xmlp_subStruct
    @h2xmlp_toolPolicy   {NO_SUPPORT}*/
/**<@h2xmlp_description { Payload contains timestamp of the detection event. Used when keyword
                         index information is expected in a generic detection
                         engine event. To get the information, the mode field in
                         PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG must be set
                         accordingly} */
struct detection_timestamp_info_t
{
   uint32_t detection_timestamp_lsw;
   /**< @h2xmle_description {Lower 32 bits of the 64-bit detection timestamp. (in milliseconds).}
        @h2xmle_range         {0..4294967295}
        @h2xmle_default       {0} */

   uint32_t detection_timestamp_msw;
   /**< @h2xmle_description {Upper 32 bits of the 64-bit detection timestamp (in milliseconds).}
        @h2xmle_range         {0..4294967295}
        @h2xmle_default       {0} */
};
typedef struct detection_timestamp_info_t detection_timestamp_info_t;

/* Payload associated with the KEY_ID_FTRT_DATA_INFO type*/

/** @h2xmlp_subStruct
    @h2xmlp_toolPolicy   {NO_SUPPORT} */

/**<@h2xmlp_description {Information about the key ID ftrt data length. Used
                         when ftrt data information is expected in a generic
                         detection engine event. To get the information, the
                         mode field in PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG
                         must be set accordingly.} */
struct ftrt_data_info_t
{
   uint32_t ftrt_data_length_in_us;
   /**< @h2xmle_description {Amount of data (in micro seconds) buffered in Dam module, to read data
                             in FTRT mode.}
        @h2xmle_range         {0..4294967295}
        @h2xmle_default       {0} */
};
typedef struct ftrt_data_info_t ftrt_data_info_t;

/* Payload associated with the KEY_ID_BEST_CH_IDX_INFO type*/

/** @h2xmlp_subStruct
    @h2xmlp_toolPolicy   {NO_SUPPORT} */

/**<@h2xmlp_description {Information about the key ID best channel index. Used
                         when this best ch information is expected in a generic
                         detection engine event. To get the information, the
                         mode field in PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG
                         must be set accordingly.} */
struct best_ch_idx_info_t
{
   uint32_t best_ch_idx;
   /**< @h2xmle_description {Best Channel Index}
        @h2xmle_range         {0..15}
        @h2xmle_default       {0} */
};
typedef struct best_ch_idx_info_t best_ch_idx_info_t;

/* Payload associated with the KEY_ID_VOICE_UI_MULTI_MODEL_RESULT_INFO type*/

/** @h2xmlp_subStruct
    @h2xmlp_toolPolicy   {NO_SUPPORT} */

/**<@h2xmlp_description {Information about the key ID Stage1 SVA5 Result info. Used
                         when this SVA5 result information is expected in a generic
                         detection engine event. To get the information, the
                         mode field in PARAM_ID_DETECTION_ENGINE_GENERIC_EVENT_CFG
                         must be set accordingly.} */
struct voice_ui_multi_model_result_info_t
{
    uint32_t num_detected_models;
    /**< @h2xmle_description {Indicates number of models that got detected.
                              If 3 models are registered with STAGE1_SVA5 and
                              detected happened for 2 models,
                              then this value will be set to 2
                              Following this will be num_detected_models
                              number of model_stats_t type payloads
                              each containing the result statistics
                              for each detected model}
         @h2xmle_range       {1..MAX_MODELS_SUPPORTED}
         @h2xmle_default     {0}*/
};
typedef struct voice_ui_multi_model_result_info_t voice_ui_multi_model_result_info_t;

struct model_stats_t
{
    uint32_t detected_model_id;
      /**< @h2xmle_description {Model ID of the model which got detected}
           @h2xmle_range       {0..0xFFFFFFFE}
           @h2xmle_default     {0} */

    uint32_t detected_keyword_id;
    /**< @h2xmle_description {Keyword ID of the keyword which got detected}
         @h2xmle_range       {0..MAX_KEYWORDS_SUPPORTED}
         @h2xmle_default     {0} */

    uint32_t best_channel_idx;
      /**< @h2xmle_description {Best channel index}
           @h2xmle_range       {0..15}
           @h2xmle_default     {0} */

    uint32_t best_confidence_level;
      /**< @h2xmle_description {Best Confidence Level for the detected model}
           @h2xmle_range       {0..100}
           @h2xmle_default     {0} */

   uint32_t kw_start_timestamp_lsw;
   /**< @h2xmle_description {Lower 32 bits of the 64-bit Keyword start time stamp. (in us).}
        @h2xmle_range       {0..4294967295}
        @h2xmle_default     {0} */

   uint32_t kw_start_timestamp_msw;
   /**< @h2xmle_description {Upper 32 bits of the 64-bit Keyword start time stamp. (in us).}
        @h2xmle_range       {0..4294967295}
        @h2xmle_default     {0} */

   uint32_t kw_end_timestamp_lsw;
   /**< @h2xmle_description {Lower 32 bits of the 64-bit Keyword end time stamp. (in us).}
        @h2xmle_range       {0..4294967295}
        @h2xmle_default     {0} */

   uint32_t kw_end_timestamp_msw;
   /**< @h2xmle_description {Upper 32 bits of the 64-bit Keyword end time stamp. (in us).}
        @h2xmle_range       {0..4294967295}
        @h2xmle_default     {0} */

   uint32_t detection_timestamp_lsw;
   /**< @h2xmle_description {Lower 32 bits of the 64-bit detection timestamp. (in milliseconds).}
        @h2xmle_range       {0..4294967295}
        @h2xmle_default     {0} */

   uint32_t detection_timestamp_msw;
   /**< @h2xmle_description {Upper 32 bits of the 64-bit detection timestamp (in milliseconds).}
        @h2xmle_range       {0..4294967295}
        @h2xmle_default     {0} */
};
typedef struct model_stats_t model_stats_t;
/** @} */ /* end_weakgroup weak_detection_event_info_header_t */
/*==============================================================================
   Constants
==============================================================================*/

/* Unique Parameter id */
#define PARAM_ID_DETECTION_ENGINE_RESET 0x08001051

/*==============================================================================
   Type definitions
==============================================================================*/

/**@h2xmlp_emptyParameter {"PARAM_ID_DETECTION_ENGINE_RESET",
                           PARAM_ID_DETECTION_ENGINE_RESET}
    @h2xmlp_description   {Resets the Detection Engine module to its initial
                           algorithm stage and begins a new detection.}
    @h2xmlp_toolPolicy    {NO_SUPPORT} */

/*==============================================================================
   Constants
==============================================================================*/
/* Unique Parameter id */
#define PARAM_ID_DETECTION_ENGINE_BUFFERING_CONFIG     0x08001044

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure definition for Parameter */

/** @h2xmlp_parameter   {"PARAM_ID_DETECTION_ENGINE_BUFFERING_CONFIG",
                         PARAM_ID_DETECTION_ENGINE_BUFFERING_CONFIG}
    @h2xmlp_description { Set the buffering requirements for the detection module.
                          Based on this requirements, detection module resizes the Audio
                          Dam buffers via IMCL.}
    @h2xmlp_toolPolicy  {Calibration} */
#include "spf_begin_pack.h"
struct param_id_detection_engine_buffering_config_t
{
   uint32_t hist_buffer_duration_msec;
   /**< @h2xmle_description {Duration (in milliseconds) of the history buffer
                             that holds the keyword. The duration is limited by the
                             memory available in the system.}
        @h2xmle_policy       {Basic}
        @h2xmle_default     {0} */

   uint32_t pre_roll_duration_msec;
   /**< @h2xmle_description {Amount of data before a keyword start (in
                             milliseconds).}
        @h2xmle_policy       {Basic}
        @h2xmle_default       {0} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_detection_engine_buffering_config_t param_id_detection_engine_buffering_config_t;


#define PARAM_ID_DETECTION_ENGINE_MULTI_MODEL_BUFFERING_CONFIG 0x0800124F

/*==============================================================================
   Type definitions
==============================================================================*/

/* Structure definition for Parameter */

/** @h2xmlp_parameter   {"PARAM_ID_DETECTION_ENGINE_MULTI_MODEL_BUFFERING_CONFIG",
                         PARAM_ID_DETECTION_ENGINE_MULTI_MODEL_BUFFERING_CONFIG}
    @h2xmlp_description { Set the buffering requirements for the PDK module.
                          Based on this requirements, PDK resizes the Audio
                          Dam buffers via IMCL.}
    @h2xmlp_toolPolicy  {Calibration} */
#include "spf_begin_pack.h"
struct param_id_detection_engine_multi_model_buffering_config_t
{
  uint32_t model_id;
  /**< @h2xmle_description {Model id of the model for which buffering config is being done
                             The Model has to be registered before sending this set param}
        @h2xmle_policy       {Basic}
        @h2xmle_range       {0..0xFFFFFFFE}
        @h2xmle_default     {0} */
  uint32_t hist_buffer_duration_msec;
  /**< @h2xmle_description {Duration (in milliseconds) of the history buffer
                             that holds both the data before keyword detection
                             and the keyword. The duration is limited by the
                             memory available in the system.}
        @h2xmle_policy       {Basic}
        @h2xmle_default     {0} */

  uint32_t pre_roll_duration_msec;
  /**< @h2xmle_description {Amount of data before a keyword start (in
                             milliseconds).}
        @h2xmle_policy       {Basic}
        @h2xmle_default       {0} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_detection_engine_multi_model_buffering_config_t param_id_detection_engine_multi_model_buffering_config_t;



#define PARAM_ID_DETECTION_ENGINE_PER_MODEL_RESET 0x080014ED

/** @h2xmlp_parameter    {"PARAM_ID_DETECTION_ENGINE_PER_MODEL_RESET", PARAM_ID_DETECTION_ENGINE_PER_MODEL_RESET}
    @h2xmlp_description  { Resets the Detection Engine model to its initial algorithm stage and begins a new detection\n}
    @h2xmlp_toolPolicy   {Calibration}*/

#include "spf_begin_pack.h"
struct param_id_detection_engine_per_model_reset_t
{
    uint32_t model_id;
    /**< @h2xmle_description {This is an unique number to differentiate
                            between different sound models to perform reset}
         @h2xmle_default     {0x0}
         @h2xmle_range       {0..0xFFFFFFFE} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_detection_engine_per_model_reset_t param_id_detection_engine_per_model_reset_t;

#endif /* _DETECTION_CMN_API_H_ */
