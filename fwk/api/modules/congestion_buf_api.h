#ifndef __CONGESTION_BUF_H
#define __CONGESTION_BUF_H
/**
 * \file congestion_buf_api.h
 * \brief
 *     This file contains module published by Congestion Buf CAPI intialization.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "module_cmn_api.h"
#include "imcl_fwk_intent_api.h"

/**
  @h2xml_title1          {Congestion Audio Buffer Module}
  @h2xml_title_agile_rev {Congestion Audio Buffer Module}
  @h2xml_title_date      {Jun 4, 2022}
*/

/** @ingroup ar_spf_mod_cab_macros
    Maximum number of input ports supported by the Congestion Audio Buffer module. */
#define CAPI_CONGESTION_BUF_MAX_INPUT_PORTS         (1)

/** @ingroup ar_spf_mod_cab_macros
    Maximum number of output ports supported by the Congestion Audio Buffer module. */
#define CAPI_CONGESTION_BUF_MAX_OUTPUT_PORTS        (1)

/*==============================================================================
   Constants
==============================================================================*/

/** @ingroup ar_spf_mod_cab_macros
    Configures the Congestion Audio Buffer module. */
#define PARAM_ID_CONGESTION_BUF_CONFIG         0x080014E2

/** @ingroup ar_spf_mod_cab_macros
    Configures the Congestion Audio Buffer size. */
#define PARAM_ID_CONGESTION_BUF_SIZE_CONFIG    0x080014E5

/*==============================================================================
   Type Definitions
==============================================================================*/

typedef struct param_id_congestion_buf_config_t param_id_congestion_buf_config_t;

/** @ingroup ar_spf_mod_cab_macros
    This parameter ID is used to configure the Congestion Buffer module. */

/** @h2xmlp_parameter   {"PARAM_ID_CONGESTION_BUF_CONFIG",
                          PARAM_ID_CONGESTION_BUF_CONFIG}
    @h2xmlp_description { This param ID is used to configure Congestion Buffer module. }
    @h2xmlp_toolPolicy  {NO_SUPPORT} */
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_congestion_buf_config_t
{
      uint32_t bit_rate_mode;
      /**< Defines the bit rate mode of the module. */

      /**< @h2xmle_description { Defines the bit rate mode of module }
           @h2xmle_rangeList   {"UNSET" = 0; "AVG" = 1; "MAX" = 2} */

      uint32_t bit_rate;
      /**< Defines the bit rate of the decoder. A value of 0 indicates unknown or unset bitrate. */

      /**< @h2xmle_description { Defines the bit rate of decoder. A value of '0' indicates unknown\unset bitrate}
           @h2xmle_default     { 0 } */

      uint32_t congestion_buffer_duration_ms;
      /**< Defines the congestion buffer size (not latency) in milliseconds. */

      /**< @h2xmle_description { Defines the congestion buffer size (not latency) in milli seconds}
           @h2xmle_default     { 0 }
           @h2xmle_range       {0..300} */

      uint32_t delay_buffer_duration_ms;
      /**< Reserved. Set to 0. */

      /**< @h2xmle_description { Reserved, set to 0 }
           @h2xmle_default     { 0 } */

      uint32_t frame_size_mode;
      /**< Defines the mode of the frame size, which can be duration or samples. */

      /**< @h2xmle_description { Defines the mode of frame size which can be duration or samples }
           @h2xmle_rangeList   {"UNSET" = 0; "DURATION IN US" = 1; "SAMPLES" = 2} */

      uint32_t frame_size_value;
      /**< Decoder frame size in us or samples, where 0 is  unset or variable. */

      /**< @h2xmle_description { Decoder frame size in us or samples - 0 : unset\variable}
           @h2xmle_default     { 0 } */

      uint32_t sampling_rate;
      /**< Decoder sampling rate, which is used to derive frame duration when frame size mode is samples. */

      /**< @h2xmle_description { Decoder sampling rate, used to derive frame duration when frame size mode is samples } */

      uint32_t mtu_size;
      /**< Maximum Transfer Unit in bytes. */

      /**< @h2xmle_description { Maximum Transfer Unit in bytes }
           @h2xmle_default     { 1024 }
           @h2xmle_range       {0..1200} */

}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

typedef struct param_id_congestion_buf_size_config_t param_id_congestion_buf_size_config_t;

/** @ingroup ar_spf_mod_cab_macros
    This parameter ID is used to configure the Congestion Buffer size from QACT for debug purposes. */

/** @h2xmlp_parameter   {"PARAM_ID_CONGESTION_BUF_SIZE_CONFIG",
                          PARAM_ID_CONGESTION_BUF_SIZE_CONFIG}
    @h2xmlp_description { This param ID is used to configure Congestion Buffer size from QACT for debug purpose. }
    @h2xmlp_toolPolicy  {CALIBRATION} */

#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_congestion_buf_size_config_t
{
      uint32_t congestion_buffer_duration_ms;
      /**< Defines the congestion buffer size (not latency) in milliseconds. */

      /**< @h2xmle_description { Defines the congestion buffer size (not latency) in milli seconds}
           @h2xmle_default     { 0 }
           @h2xmle_range       {0..300} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;

/*------------------------------------------------------------------------------
   Module
------------------------------------------------------------------------------*/

/** @ingroup ar_spf_mod_cab_macros
    This module is used to buffer compress audio data for use cases such as BT Sink.

    @subhead4{Supported parameter IDs}
    - #PARAM_ID_CONGESTION_BUF_CONFIG @lstsp1
    - #PARAM_ID_CONGESTION_BUF_SIZE_CONFIG

    @subhead4{Supported input media format ID}
    - Data Format          : RAW_COMPRESSED @lstsp1
    - fmt_id               : Don't care. */
#define MODULE_ID_CONGESTION_AUDIO_BUFFER                     0x07010004

/**
    @h2xmlm_module         {"MODULE_ID_CONGESTION_AUDIO_BUFFER", MODULE_ID_CONGESTION_AUDIO_BUFFER}
    @h2xmlm_displayName    {"Congestion Audio Buffer"}

    @h2xmlm_description    { This module is used to buffer compressed audio data for usecases such as BT Sink.\n

    \n This module supports the following parameter IDs:\n
     - PARAM_ID_CONGESTION_BUF_CONFIG \n
     - PARAM_ID_CONGESTION_BUF_SIZE_CONFIG\n

   \n Supported Input Media Format:\n
*  - Data Format          : RAW_COMPRESSED\n
*  - fmt_id               : Don't care\n }

    @h2xmlm_dataMaxInputPorts    { 1 }
    @h2xmlm_dataMaxOutputPorts   { 1 }
    @h2xmlm_supportedContTypes  { APM_CONTAINER_TYPE_GC }
    @h2xmlm_isOffloadable        {false}
    @h2xmlm_stackSize            { 1024 }
    @{                     <-- Start of the Module -->

    @h2xml_Select      {param_id_congestion_buf_config_t}
    @h2xmlm_InsertParameter
    @h2xml_Select      {param_id_congestion_buf_size_config_t}
    @h2xmlm_InsertParameter
 @}                     <-- End of the Module --> */

#endif /*__CONGESTION_BUF_H*/
