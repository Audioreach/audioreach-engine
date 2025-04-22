#ifndef _PRIORITY_SYNC_API_H_
#define _PRIORITY_SYNC_API_H_
/**
 * \file priority_sync_api.h
 * \brief
 *    This file contains Priority Sync module APIs
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "apm_graph_properties.h"

/*# @h2xml_title1          {Priority Synchronization Module API}
    @h2xml_title_agile_rev {Priority Synchronization Module API}
    @h2xml_title_date      {July 12, 2018} */

/*==============================================================================
   Defines
==============================================================================*/

/** @ingroup ar_spf_mod_priority_sync
    Enumerates the maximum number of input ports for the Priority
    Synchronization module. */
#define PRIORITY_SYNC_MAX_IN_PORTS 2

/** @ingroup ar_spf_mod_priority_sync
    Enumerates the maximum number of output ports for the Priority
    Synchronization module. */
#define PRIORITY_SYNC_MAX_OUT_PORTS 2


/** @ingroup ar_spf_mod_priority_sync
    Identifier of the primary input port for the Priority Synchronization
    module. */
#define PRIORITY_SYNC_PRIMARY_IN_PORT_ID 0x2

/** @ingroup ar_spf_mod_priority_sync
    Identifier of the primary output port for the Priority Synchronization
    module. */
#define PRIORITY_SYNC_PRIMARY_OUT_PORT_ID 0x1


/** @ingroup ar_spf_mod_priority_sync
    Identifier of the secondary input port for the Priority Synchronization
    module. */
#define PRIORITY_SYNC_SECONDARY_IN_PORT_ID 0x4

/** @ingroup ar_spf_mod_priority_sync
    Identifier of the secondary output port for the Priority Synchronization
    module. */
#define PRIORITY_SYNC_SECONDARY_OUT_PORT_ID 0x3

/** @ingroup ar_spf_mod_priority_sync
    Enumerates the stack size of the Priority Synchronization module.
    @newpage */
#define PRIORITY_SYNC_STACK_SIZE 4096

/*==============================================================================
   Param ID
==============================================================================*/

/** @ingroup ar_spf_mod_priority_sync
 *
    Parameter to set the buffering mode of secondary data.

    If set to the MAINTAIN_LATEST_DATA (default mode) then internal buffer for secondary port is allowed to overflow
while waiting for primary path data. This is to ensure that the secondary signal (which is real time) remains in time
sync with primary path data (which is also real time).

    If set to MAINTAIN_OLDEST_DATA then internal buffer for secondary port is not allowed to overflow while waiting for
primary path data, this avoids data loss on secondary path. This mode is helpful when there is no correlation between
primary and secondary signal and secondary path signal is not required to be in time sync with primary signal.

    This parameter is not relevant if timestamp-based synchronization is enabled because in that case secondary signal
may be allowed to overflow to match timestamp with primary path data.

    @msgpayload
    param_id_priority_sync_secondary_buffering_mode_t
*/
#define PARAM_ID_PRIORITY_SYNC_SECONDARY_BUFFERING_MODE 0x08001A60

#define PRIORITY_SYNC_SECONDARY_MAINTAIN_LATEST_DATA 0
#define PRIORITY_SYNC_SECONDARY_MAINTAIN_OLDEST_DATA 1

/*==============================================================================
   Param structure definitions
==============================================================================*/

/*# @h2xmlp_parameter   {"PARAM_ID_PRIORITY_SYNC_SECONDARY_BUFFERING_MODE",PARAM_ID_PRIORITY_SYNC_SECONDARY_BUFFERING_MODE}
    @h2xmlp_description {Parameter to set the buffering mode of secondary data. }
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_priority_sync
    Payload for #PARAM_ID_PRIORITY_SYNC_SET_SECONDARY_SIGNAL_TYPE.
*/
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_priority_sync_secondary_buffering_mode_t
{
   uint32_t sec_buffering_mode;
   /**< @h2xmle_description {Parameter to set the buffering mode of secondary data.}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"MAINTAIN_LATEST_DATA"   = PRIORITY_SYNC_SECONDARY_MAINTAIN_LATEST_DATA;
                             "MAINTAIN_OLDEST_DATA" = PRIORITY_SYNC_SECONDARY_MAINTAIN_OLDEST_DATA}
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_priority_sync_secondary_buffering_mode_t param_id_priority_sync_secondary_buffering_mode_t;

/*==============================================================================
   Param ID
==============================================================================*/

/** @ingroup ar_spf_mod_priority_sync
    Parameter to enable timestamp based synchronization of primary and secondary
    path.

    @msgpayload
    param_id_priority_sync_timestamp_sync_t
*/
#define PARAM_ID_PRIORITY_SYNC_TIMESTAMP_SYNC     0x080014EA

/*==============================================================================
   Param structure definitions
==============================================================================*/

/*# @h2xmlp_parameter   {"PARAM_ID_PRIORITY_SYNC_TIMESTAMP_SYNC",PARAM_ID_PRIORITY_SYNC_TIMESTAMP_SYNC}
    @h2xmlp_description {Parameter to enable timestamp based synchronization of primary and secondary
    path.}
    @h2xmlp_toolPolicy  {Calibration} */

/** @ingroup ar_spf_mod_priority_sync
    Payload for #PARAM_ID_PRIORITY_SYNC_TIMESTAMP_SYNC.
*/
#include "spf_begin_pack.h"
#include "spf_begin_pragma.h"
struct param_id_priority_sync_timestamp_sync_t
{
   uint32_t enable;
   /**< @h2xmle_description {enable timestamp based synchronization. only for voice-UI}
        @h2xmle_default     {0}
        @h2xmle_rangeList   {"enable"   = 1;"disable" = 0}
        @h2xmle_policy      {Basic} */
}
#include "spf_end_pragma.h"
#include "spf_end_pack.h"
;
typedef struct param_id_priority_sync_timestamp_sync_t param_id_priority_sync_timestamp_sync_t;


/*==============================================================================
   API definitions
==============================================================================*/

/** @ingroup ar_spf_mod_priority_sync
    Identifier for the module used to synchronize the secondary input signal
    with respect to the primary(priority) input signal.

    @subhead4{Use cases}
    - For ECNS and similar use cases -- To synchronize an echo reference signal
      with mic data. @lstsp1
    - For the ICMD use case -- To synchronize music Rx signal with the voice Tx
      signal.

    @subhead4{Supported input media format ID}
    - Data format       : #DATA_FORMAT_FIXED_POINT @lstsp1
    - fmt_id            : #MEDIA_FMT_ID_PCM @lstsp1
    - Sample rates      : 8, 11.025, 12, 16, 22.05, 24, 32, 44.1, 48, 88.2, 96,
                          176.4, 192, 352.8, 384 kHz @lstsp1
    - Number of channels: 1 to 128 (for certain products this module supports only 32 channels) @lstsp1
    - Channel type      : Don't care @lstsp1
    - Bit width         : 16, 24, 32 @lstsp1
    - Q format          : Don't care @lstsp1
    - Interleaving      : De-interleaved unpacked @lstsp1
    - Endianness        : little, big

    Primary and secondary ports can have different media formats. The only
    restriction is that if one port runs at a fractional rate, the other port
    must also run at a fractional rate. The module does not support use cases
    where ports have variable input frame sizes.
*/
#define MODULE_ID_EC_SYNC 0x07001028

/** @ingroup ar_spf_mod_priority_sync
    Forward declaration for the Priority Synchronization module. */
#define MODULE_ID_PRIORITY_SYNC MODULE_ID_EC_SYNC

/*# @h2xmlm_module             {"MODULE_ID_PRIORITY_SYNC",
                                 MODULE_ID_PRIORITY_SYNC}
    @h2xmlm_displayName        {"Priority Sync"}
    @h2xmlm_modSearchKeys      {Audio, Voice, Echo Canceller }
    @h2xmlm_description        {ID for the module used to synchronize the
                                secondary input signal with respect to the
                                primary(priority) input signal. For more
                                details, see AudioReach Signal Processing Framework
                                (SPF) API Reference.}
    @h2xmlm_dataMaxInputPorts  {PRIORITY_SYNC_MAX_IN_PORTS}
    @h2xmlm_dataInputPorts     {PRIMARY_IN=PRIORITY_SYNC_PRIMARY_IN_PORT_ID;
                                SECONDARY_IN=PRIORITY_SYNC_SECONDARY_IN_PORT_ID}
    @h2xmlm_dataMaxOutputPorts {PRIORITY_SYNC_MAX_OUT_PORTS}
    @h2xmlm_dataOutputPorts    {PRIMARY_OUT=PRIORITY_SYNC_PRIMARY_OUT_PORT_ID;
                                SECONDARY_OUT=PRIORITY_SYNC_SECONDARY_OUT_PORT_ID}

    @h2xmlm_supportedContTypes {APM_CONTAINER_TYPE_SC, APM_CONTAINER_TYPE_GC}
    @h2xmlm_isOffloadable      {true}
    @h2xmlm_stackSize          {PRIORITY_SYNC_STACK_SIZE}

    @{                      <-- Start of the Module -->
    @h2xml_Select           {param_id_priority_sync_timestamp_sync_t}
    @h2xmlm_InsertParameter

    @h2xml_Select           {param_id_priority_sync_secondary_buffering_mode_t}
    @h2xmlm_InsertParameter
    @}                      <-- End of the Module --> */

#endif //_PRIORITY_SYNC_API_H_
