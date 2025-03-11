/**========================================================================
 @file imcl_spm_intent_api.h

 @brief This file contains all the public intent names
 the intent related structs are defined in internal header files

 Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause

 ====================================================================== */

#ifndef _IMCL_SPM_INTENT_API_H_
#define _IMCL_SPM_INTENT_API_H_

/*------------------------------------------------------------------------------
 *  Header Includes
 *----------------------------------------------------------------------------*/

#include "ar_defs.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/** -----------------------------------------------------------------------
 ** Type Declarations
 ** ----------------------------------------------------------------------- */

/* clang-format off */

/*==============================================================================
  Intent ID
==============================================================================*/
// Intent ID for the control link between FFECNS and detection engine.
#define INTENT_ID_FFECNS_SVA_FREEZE                  0x080010BA


/*==============================================================================
  Intent ID
==============================================================================*/
/**<
Intent ID for the control link between AVC Tx and Rx module.

The AVC module automatically adjusts the loudness of the Rx voice in
response to the near-end noise level on the Tx path. This adjustment
improves the intelligibility of the receive path voice signal.
*/
#define INTENT_ID_AVC                   	     0x080010DB

/*==============================================================================
  Intent ID
==============================================================================*/
// Intent ID for SP
#define INTENT_ID_SP                                0x08001204

/*==============================================================================
  Intent ID
==============================================================================*/
// Intent ID for HAPTICS
#define INTENT_ID_HAPTICS                            0x0800136E

/*==============================================================================
  Intent ID
==============================================================================*/
// Intent ID for CPS
#define INTENT_ID_CPS                                0x08001537

/*==============================================================================
  Intent ID
==============================================================================*/
/**<
Intent ID to send DRC parameters over control link.
e.g.
Control link between Rx DRC and Tx AVC modules in voice path.
*/
#define INTENT_ID_DRC_CONFIG 			     0x080010F3

/*==============================================================================
  Intent ID
==============================================================================*/
/**<
Intent ID for the control link to send gain information.
This intent can be used by a module to send its gain to another module.

e.g.
Soft Volume Control sends the target gain to AVC-TX module in voice path.
*/
#define INTENT_ID_GAIN_INFO         	 	     0x080010F5

/*==============================================================================
  Intent ID
==============================================================================*/
// Intent ID for the control link between Soft Vol and Popless Equalizer.
#define INTENT_ID_P_EQ_VOL_HEADROOM                  0x08001118

/*==============================================================================
  Intent ID
==============================================================================*/
/**<
Intent ID for the control link between RVE Tx and Rx module.

The RVE module automatically adjusts the loudness of the Rx voice in
response to the near-end noise level on the Tx path. This adjustment
improves the intelligibility of the receive path voice signal.
*/
#define INTENT_ID_RVE                   	    0x080010DD

/*==============================================================================
  Intent ID
==============================================================================*/
/** IMCL intent ID for sending the notification to the packetizer v1
 * to send sideband ack to BT that the primer sideband is received */
#define INTENT_ID_PACKETIZER_V1_FEEDBACK        0x08001148

/*==============================================================================
  Intent ID
==============================================================================*/
/** IMCL intent ID for sending the notification to the cop v2 packetizer
 * to send sideband ack to BT that the primer sideband is received and for packetizer
 * to send rx output media format related information to depacketizer */
#define INTENT_ID_V2_PACKETIZER_FEEDBACK        0x08001306

/*==============================================================================
  Intent ID
==============================================================================*/
/**<
Intent ID for the control link between CTM Tx and Rx module.
*/
#define INTENT_ID_CTM_TTY                   	0x08001191

/*==============================================================================
  Intent ID
==============================================================================*/
/**<
Intent ID for the control link between TTY module(1x/CTM/LTE_TTY in Rx path) and soft volume module(in Tx path).
TTY module communicates with the soft volume module to indicate whether to
mute or unmute the stream in the Tx path. The parameter - PARAM_ID_IMCL_MUTE
is used for this communication.

*/
#define INTENT_ID_MUTE                          0x08001195

/*==============================================================================
  Intent ID
==============================================================================*/
/**<
Intent ID for the control link between Mailbox RX and DTMF module. Mailbox RX uses this
control link to send tone configuration to DTMF generator module.
*/
#define INTENT_ID_DTMF_GENERATOR_CTRL           0x08001207

/*==============================================================================
  Intent ID
==============================================================================*/
/**<
Intent ID for the control link between audioss aov dma driver and the voice wakeup module.
When VW_v2 gets status_detected or status_rejected, this control link is used to send commnad to stop the aov dma.
This is bkz in audioss vad, there is no -ve edge interrupt to stop the aov dma
*/
#define INTENT_ID_AUDIOSS_AOV_DMA               0x0800130A

/* clang-format on */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* _IMCL_SPM_INTENT_API_H_ */
