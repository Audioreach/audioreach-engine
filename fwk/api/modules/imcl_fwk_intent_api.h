#ifndef _IMCL_FWK_INTENT_API_H_
#define _IMCL_FWK_INTENT_API_H_
/**
 *  \file imcl_fwk_intent_api.h
 * \brief
 *    This file contains all the public intent names.
 *    The intent related structs are defined in internal header file
 *
 * \copyright
 *   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *   SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*------------------------------------------------------------------------------
 *  Header Includes
 *----------------------------------------------------------------------------*/

/** @addtogroup ar_spf_mod_ctrl_port_int_ids
    Intents used by the I2S, PCM-TDM, Sample Slip, Sample Slip EC, and SPR
    modules.
*/

#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/* -----------------------------------------------------------------------
 * Type Declarations
 * ----------------------------------------------------------------------- */

/* clang-format off */

/* clang-format on */

/*==============================================================================
  Intent IDs
==============================================================================*/

/** @ingroup ar_spf_mod_ctrl_port_int_ids
    Identifier for the IMCL intent used to share a real-time drift source's
    accumulated drift relative to the local timer with rate matching modules.
*/
#define INTENT_ID_TIMER_DRIFT_INFO           0x080010C2

/** @ingroup ar_spf_mod_ctrl_port_int_ids
    Identifier for the IMCL intent used to send a module instance ID. */
#define INTENT_ID_MODULE_INSTANCE_INFO       0x08001089


#ifndef DOXYGEN_SHOULD_SKIP_THIS // to end of file

/** Identifier for the IMCL intent used to send a deadline time. */
#define INTENT_ID_BT_DEADLINE_TIME           0x080010D3

/** Identifier for the IMCL intent used to define the payload structure of an
    IMCL message.

    The SVA and Dam modules support the following functionalities:
    - Channel resizing -- An input intent ID exposed by the Dam module. This
      intent allows resizing of the channel buffers based on the SVA module's
      buffering requirement.
    - Output port data flow control -- An input intent ID exposed by the Dam
      module. This intent is used to open or close the Dam output port gates by
      SVA module.
    - Best channel output -- This intent allows detection engines to send the
      best channel indices to the Dam module. Upon receiving the best channel
      indices, the Dam module outputs only the best channels from the given
      output port.
    - FTRT data availably information -- An output intent ID exposed by the Dam
      module to send the unread data length [FTRT data] length present in
      channel buffers to the SVA module.
*/
#define INTENT_ID_AUDIO_DAM_DETECTION_ENGINE_CTRL    0x08001064

/** Identifier for the IMCL intent used for the control link between the
    detection engine and stage 2 detection engine. */
#define INTENT_ID_DETECTION_ENGINE_STAGE2_TO_STAGE1   0x0800124B

/** Identifier for the IMCL intent used to communicate between depack v1,v2/conn proxy src and encoder.
    @subhead4{Supported parameter IDs}
    - #IMCL_PARAM_ID_BT_SIDEBAND_ENCODER_FEEDBACK
    - #IMCL_PARAM_ID_BT_SIDEBAND_ENCODER_FEEDBACK_V2
    - #IMCL_PARAM_ID_BIT_RATE_LEVEL_ENCODER_FEEDBACK
    - #IMCL_PARAM_ID_BT_SIDEBAND_TRIGGER_ENABLE
    - #IMCL_PARAM_ID_ENC_TO_CONN_PROXY_SINK
    - #IMCL_PARAM_ID_CONN_PROXY_SINK_TO_ENC
*/
#define INTENT_ID_BT_ENCODER_FEEDBACK           0x080010D7

/**< This intent ID allows AAD to be able to communicate with
disables AAD MODULE*/
#define INTENT_ID_AAD_STATE_CONTROL             0x08001386

/** Identifier for the IMCL intent used to communicate between Conn proxy modules
	@subhead4{Supported parameter IDs}
   	   - #IMCL_PARAM_ID_PRIMER_FEEDBACK
*/
#define INTENT_ID_BT_CONN_PROXY_FEEDBACK        0x080013C5

/** Identifier for the IMCL intent used to communicate between AAD and Codec DMA module.
   	   - #IMCL_PARAM_ID_HW_AAD_ENABLE_REQ
       - #IMCL_PARAM_ID_HW_AAD_ENABLE_ACK
       - #IMCL_PARAM_ID_SWITCH_PRBDMA_MODE
*/
#define INTENT_ID_PRBDMA_CONTROL               0x08001A74

#endif /* DOXYGEN_SHOULD_SKIP_THIS */


#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* _IMCL_FWK_INTENT_API_H_ */
