#ifndef _GAPLESS_API_H_
#define _GAPLESS_API_H_
/**
 * \file gapless_api.h
 * \brief
 *    This file contains the Gapless module APIs.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "module_cmn_api.h"

/*# @h2xml_title1          {Gapless Module}
    @h2xml_title_agile_rev {Gapless Module}
    @h2xml_title_date      {August 12, 2019} */

/*==============================================================================
   Constants
==============================================================================*/

/** @ingroup ar_spf_mod_gapless_macros
    Maximum number of input ports supported by the Gapless module. */
#define GAPLESS_MAX_INPUT_PORTS   0x2

/** @ingroup ar_spf_mod_gapless_macros
    Maximum number of output ports supported by the Gapless module. */
#define GAPLESS_MAX_OUTPUT_PORTS  0x1

/** @ingroup ar_spf_mod_gapless_macros
    Size of the stack for the Gapless module. */
#define GAPLESS_STACK_SIZE        4096

/*==============================================================================
   Event ID
==============================================================================*/

/** @ingroup ar_spf_mod_gapless_mod
    GUID for an early end-of-stream event.

    This event is raised immediately when an EOS enters the Gapless module.

    This event can trigger the client to prepare to send the next stream. While
    the client is setting up the next stream, the Gapless module drains its
    buffered data. The amount of buffered data is equal to the maximum time the
    client takes to switch streams. The time is configured as part of
    #PARAM_ID_EARLY_EOS_DELAY.

    For the delay buffer (and therefore for gapless switching) to be used for
    the stream, this event, along with PARAM_ID_EARLY_EOS_DELAY, must be sent
    before the stream is started.

    @msgpayload
    event_id_gapless_early_eos_event_t
*/
#define EVENT_ID_EARLY_EOS         0x08001126

/*==============================================================================
   Event Type definitions
==============================================================================*/

/*# @h2xmlp_parameter   {"EVENT_ID_EARLY_EOS", EVENT_ID_EARLY_EOS}
    @h2xmlp_description {GUID for an early EOS event. This event is raised
                         immediately when an EOS enters the Gapless module.
                         For more details, see AudioReach Signal Processing
                         Framework (SPF) API Reference.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_gapless_mod
    Payload for #EVENT_ID_EARLY_EOS.
 */
#include "spf_begin_pack.h"
struct event_id_gapless_early_eos_event_t
{
   uint32_t src_module_iid;
   /**< Identifier for the #MODULE_ID_WR_SHARED_MEM_EP instance of the stream
        that is about to end. */

   /*#< @h2xmle_description {Instance ID of the MODULE_ID_WR_SHARED_MEM_EP
                             of the stream that is about to end.}
        @h2xmle_default     {0}
        @h2xmle_range       {0..0xFFFFFFFF}
        @h2xmle_policy      {Basic} */

}
#include "spf_end_pack.h"
;
typedef struct event_id_gapless_early_eos_event_t event_id_gapless_early_eos_event_t;


/*==============================================================================
   Param ID
==============================================================================*/

/** @ingroup ar_spf_mod_gapless_mod
    Identifier for the parameter sent to the Gapless module to configure the
    maximum time the client will take to switch streams.

    This parameter determines the size of the Gapless module's delay buffers.
    The size does not include delay that is downstream of the early EOS.

    For the delay buffer (and therefore gapless switching) to be used for a
    stream, this parameter, along with #EVENT_ID_EARLY_EOS, must be sent before
    the stream is started.

    @msgpayload
    param_id_gapless_early_eos_delay_t @newpage
*/
#define PARAM_ID_EARLY_EOS_DELAY                 0x0800114C

/*# @h2xmlp_parameter   {"PARAM_ID_EARLY_EOS_DELAY", PARAM_ID_EARLY_EOS_DELAY}
    @h2xmlp_description {ID for the parameter sent to the Gapless module to
                         configure the maximum time the client will take to
                         switch streams. \n
                         This parameter determines the size of the Gapless
                         module's delay buffers. The size does not
                         include delay that is downstream of the early EOS.
                         For the delay buffer (and therefore gapless switching)
                         to be used for a stream, this parameter, along with
                         EVENT_ID_EARLY_EOS, must be sent before the stream is
                         started.}
    @h2xmlp_toolPolicy  {Calibration} */


/*==============================================================================
   Param structure definitions
==============================================================================*/

/** @ingroup ar_spf_mod_gapless_mod
    Payload for #PARAM_ID_EARLY_EOS_DELAY.
*/
#include "spf_begin_pack.h"
struct param_id_gapless_early_eos_delay_t
{
   uint32_t early_eos_delay_ms;
   /**< Duration in milliseconds between the early EOS notification and the EOS
        notification rendered from the hardware endpoint.

        Delay that is downstream of the early EOS is not accounted for in this
        value. This delay determines the size of the Gapless module's internal
        delay buffers. */

   /*#< @h2xmle_description {Duration in milliseconds between the early EOS
                             notification and the EOS notification rendered
                             from the hardware endpoint. \n
                             Delay that is downstream of the early EOS is not
                             accounted for in this value. This delay determines
                             the size of the Gapless module's internal delay
                             buffers.}
        @h2xmle_default     {0}
        @h2xmle_range       {0..0xFFFFFFFF}
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pack.h"
;
typedef struct param_id_gapless_early_eos_delay_t param_id_gapless_early_eos_delay_t;

/*==============================================================================
   Module
==============================================================================*/

/** @ingroup ar_spf_mod_gapless_mod
    Identifier for the Gapless module, which allows for gapless switching
    between two input streams. The output remains continuous through the stream
    switch.

    @subhead4{Supported input media format ID}
    - Data format       : #DATA_FORMAT_FIXED_POINT @lstsp1
    - fmt_id            : #MEDIA_FMT_ID_PCM @lstsp1
    - Sample rates      : Any @lstsp1
    - Number of channels: Any @lstsp1
    - Channel type      : Don't care @lstsp1
    - Bit width         : Any @lstsp1
    - Interleaving      : De-interleaved unpacked @lstsp1
    - Endianness        : Any

    Input ports can have different input media formats.
*/
#define MODULE_ID_GAPLESS                  0x0700104D

/*# @h2xmlm_module             {"MODULE_ID_GAPLESS", MODULE_ID_GAPLESS}
    @h2xmlm_displayName        {"Gapless"}
    @h2xmlm_modSearchKeys	   {Voice}
    @h2xmlm_description        {ID for the module that allows for gapless
                                switching between two input streams. \n
                                The output remains continuous through the
                                stream switch. Input ports are allowed to have
                                different input media formats. For more
                                details, see AudioReach Signal Processing
                                Framework (SPF) API Reference.}
    @h2xmlm_dataMaxInputPorts  {GAPLESS_MAX_INPUT_PORTS}
    @h2xmlm_dataMaxOutputPorts {GAPLESS_MAX_OUTPUT_PORTS}
    @h2xmlm_dataInputPorts     {IN0 = 2;
                                IN1 = 4}
    @h2xmlm_dataOutputPorts    {OUT = 3}
    @h2xmlm_toolPolicy         {Calibration}
    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable      {false}
    @h2xmlm_stackSize          {GAPLESS_STACK_SIZE}

    @{                     <-- Start of the Module -->
    @h2xml_Select           {param_id_gapless_early_eos_delay_t}
    @h2xmlm_InsertParameter
    @}                     <-- End of the Module --> */


#endif /*_GAPLESS_API_H_*/
