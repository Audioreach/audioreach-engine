#ifndef SAL_API_H
#define SAL_API_H
/*========================================================================*/
/**
@file sal_api.h

@brief sal_api.h: This file contains the Module Id, Param IDs and configuration
    structures exposed by the SAL Module.
*/
/* =========================================================================
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear
   ========================================================================== */
 
/*------------------------------------------------------------------------
  Include files
  -----------------------------------------------------------------------*/
#include "module_cmn_api.h"
#include "api_limiter.h"

/*# @h2xml_title1          {Simple Accumulator and Limiter (SAL) API}
    @h2xml_title_agile_rev {Simple Accumulator and Limiter (SAL) API}
    @h2xml_title_date      {June 20, 2018} */

/*==============================================================================
   Constants
==============================================================================*/

/** @ingroup ar_spf_mod_sal_mod
    Enumerates the maximum number of output ports for the SAL module. */
#define SAL_MAX_OUTPUT_PORTS        1

/** @ingroup ar_spf_mod_sal_mod
    Enumerates the stack size of the module. @newpage */
#define CAPI_SAL_STACK_SIZE      4096

/** @ingroup ar_spf_mod_sal_mod
    Identifier for the Simple Accumulator and Limiter (SAL) module, which
    accumulates samples in the data streams on its inputs.

    @subhead4{Supported parameter IDs}
    - #PARAM_ID_SAL_LIMITER_ENABLE @lstsp1
    - #PARAM_ID_SAL_OUTPUT_CFG @lstsp1
    - #PARAM_ID_LIMITER_CFG

    @subhead4{Supported input media format ID}
    - Data format       : #DATA_FORMAT_FIXED_POINT @lstsp1
    - fmt_id            : Don't care @lstsp1
    - Sample rates      : Any (>0) @lstsp1
    - Number of channels: 1..128 (for certain products this module supports only 32 channels) @lstsp1
    - Channel type      : 1..128 @lstsp1
    - Bits per sample   : 16, 32 @lstsp1
    - Q format          : 15, 27, 31 @lstsp1
    - Interleaving      : De-interleaved unpacked @lstsp1
    - Signed/unsigned   : Any

    @par Prerequisite
    The SYNC module is required to synchronize input streams when
    the SAL module is placed in APM_CONTAINER_TYPE_SC. For information on the
    container, see AudioReach Signal Processing Framework (SPF) API Reference.
*/
#define MODULE_ID_SAL               0x07001010

/*# @h2xmlm_module             {"MODULE_ID_SAL", MODULE_ID_SAL}
    @h2xmlm_displayName        {"Accumulator and Limiter"}
    @h2xmlm_modSearchKeys	   {Mixer, Limiter, Audio, Voice}
    @h2xmlm_description        {ID for the SAL module, which accumulates
                                samples in the data streams on its inputs. \n
                                Prerequisite: The SYNC module is required to
                                synchronize input streams when the SAL module
                                is placed in APM_CONTAINER_TYPE_SC. For
                                information on the container, see AudioReach Signal Processing Framework (SPF) API Reference.}
    @h2xmlm_dataMaxInputPorts  {INFINITE}
    @h2xmlm_dataMaxOutputPorts {SAL_MAX_OUTPUT_PORTS}
    @h2xmlm_dataOutputPorts    {OUT=1}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_GC, APM_CONTAINER_TYPE_SC}
    @h2xmlm_isOffloadable      {true}
    @h2xmlm_stackSize          {CAPI_SAL_STACK_SIZE}
    @h2xmlm_ToolPolicy         {Calibration}

    @{                      <-- Start of the Module --> */

/** @ingroup ar_spf_mod_sal_mod
    Identifier for the parameter the SAL module uses to enable its Limiter
    functionality per the default tuning from the module.

    @msgpayload
    param_id_sal_limiter_enable_t @newpage
 */
#define PARAM_ID_SAL_LIMITER_ENABLE 0x0800101E

/*# @h2xmlp_parameter   {"PARAM_ID_SAL_LIMITER_ENABLE",
                          PARAM_ID_SAL_LIMITER_ENABLE}
    @h2xmlp_description {ID for the parameter the SAL module uses to enable
                         its Limiter functionality per the default tuning from
                         the module.} */

/** @ingroup ar_spf_mod_sal_mod
    Payload for #PARAM_ID_SAL_LIMITER_ENABLE.
 */
#include "spf_begin_pack.h"
struct param_id_sal_limiter_enable_t
{
   uint32_t enable_lim;
   /**< Specifies whether the Limiting functionality of the SAL module is
        enabled.

        @valuesbul
        - 0 -- Disable_always
        - 1 -- Enable_if_req (Default) Will be enabled for multiple streams but not for single stream.
		- 2 -- Enable_always  Will be enabled even for single stream @tablebulletend */

   /*#< @h2xmle_description {Specifies whether the Limiting functionality of
                             the SAL module is enabled.}
        @h2xmle_rangeList   {"Disable_always"=0;
                             "Enable_if_req"=1;
							 "Enable_always"=2}
        @h2xmle_default     {1} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_sal_limiter_enable_t param_id_sal_limiter_enable_t;


/** @ingroup ar_spf_mod_sal_mod
    Identifier for the parameter the SAL uses to configure the output bit
    width, number of channels, and sampling rate.

    Since the Accumulator is output triggered, the processing is dictated by
    these parameters of the stream that has to be delivered on the output port.

    @msgpayload
    param_id_sal_output_cfg_t
 */
#define PARAM_ID_SAL_OUTPUT_CFG 0x08001016

/*# @h2xmlp_parameter   {"PARAM_ID_SAL_OUTPUT_CFG", PARAM_ID_SAL_OUTPUT_CFG}
    @h2xmlp_description {ID for the parameter the SAL uses to configure the
                         output bit width, number of channels, and sampling
                         rate.} */

/** @ingroup ar_spf_mod_sal_mod
    Payload for #PARAM_ID_SAL_OUTPUT_CFG.
*/
#include "spf_begin_pack.h"
struct param_id_sal_output_cfg_t
{
   int32_t bits_per_sample;
   /**< Bits per sample at the output port.

        @valuesbul
        - #PARAM_VAL_NATIVE (Default)
        - #PARAM_VAL_UNSET
        - 16 bits per sample
        - 24 bits per sample
        - 32 bits @tablebulletend */

   /*#< @h2xmle_description {Bits per sample at the output port.}
        @h2xmle_rangeList   {"PARAM_VAL_NATIVE"=-1;
                             "PARAM_VAL_UNSET"=-2;
                             "16-bit"=16;
                             "24-bit"=24;
                             "32-bit"=32}
        @h2xmle_default     {-1} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_sal_output_cfg_t param_id_sal_output_cfg_t;


/*# @h2xml_Select           {param_id_limiter_cfg_t}
    @h2xmlm_InsertParameter
    @h2xmlp_toolPolicy      {Calibration; RTC} */


/** @ingroup ar_spf_mod_sal_mod
    Identifier for the parameter that configures the static parameters of the
    Limiter Library.

    This configuration is NOT immediately applied if it is configured during
    runtime because it will cause reallocation of the Limiter.

    @msgpayload
    param_id_sal_limiter_static_cfg_t @newpage
 */
#define PARAM_ID_SAL_LIMITER_STATIC_CFG                            0x08001047

/*# @h2xmlp_parameter   {"PARAM_ID_SAL_LIMITER_STATIC_CFG",
                          PARAM_ID_SAL_LIMITER_STATIC_CFG}
    @h2xmlp_description {ID for the parameter that configures the static
                         parameters of the Limiter Library. \n
                         This configuration is NOT immediately applied if it
                         is configured during runtime because it will cause
                         reallocation of the Limiter.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_sal_mod
    Payload for #PARAM_ID_SAL_LIMITER_STATIC_CFG.
 */
#include "spf_begin_pack.h"
struct param_id_sal_limiter_static_cfg_t
{
   uint16_t max_block_size_ms;
   /**< Specifies the maximum block size (in milliseconds) to be allocated to
        the Limiter by the SAL module.

        @values 1 through 40 ms (Default = 1)

        Recommended values are:
        - 1 ms in GEN_CNTR
        - 5 ms or higher based on the use case in SPL_CNTR */

   /*#< @h2xmle_description {Specifies the maximum Limiter block size (in
                             milliseconds) to be allocated by the SAL module.
                             \n
                             Recommended values are 1 ms in GEN_CNTR, and 5 ms
                             or higher based on the use case in SPL_CNTR.}
        @h2xmle_range       {1..40}
        @h2xmle_default     {1} */

   uint16_t delay_in_sec_q15;
   /**< Specifies the limiter delay to be set to the Limiter.

        @values 0 through 10 milliseconds in Q15 format (Default = 1) */

   /*#< @h2xmle_description {Specifies the limiter delay (in Q15 format) to be
                             set to the Limiter.}
        @h2xmle_range       {0..327}
        @h2xmle_default     {33}
        @h2xmle_dataFormat  {Q15} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_sal_limiter_static_cfg_t param_id_sal_limiter_static_cfg_t;

/*# @}                      <-- End of the Module --> */


#endif /* SAL_API_H */
