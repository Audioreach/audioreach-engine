/**========================================================================
 @file imcl_bt_encoder_feedback_api.h
 @brief This file contains API's to receive/send encoder feedback

 Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 SPDX-License-Identifier: BSD-3-Clause-Clear

 ====================================================================== */

#ifndef _IMCL_BT_ENCODER_FEEDBACK_H_
#define _IMCL_BT_ENCODER_FEEDBACK_H_

/*------------------------------------------------------------------------------
 *  Header Includes
 *----------------------------------------------------------------------------*/
#include "imcl_fwk_intent_api.h"
#include "imcl_bt_bitrate_level_encoder_feedback_api.h"
#include "imcl_bt_sideband_encoder_feedback_api.h"

#ifdef INTENT_ID_BT_ENCODER_FEEDBACK

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/** -----------------------------------------------------------------------
 ** Type Declarations
 ** ----------------------------------------------------------------------- */

/* clang-format off */

/* Param id to send any info from the encoder to Conn Proxy Sink module through imcl*/
#define IMCL_PARAM_ID_ENC_TO_CONN_PROXY_SINK   0x080013C7

/* Payload of IMCL_PARAM_ID_ENC_TO_CONN_PROXY_SINK*/
typedef struct imcl_param_id_enc_to_conn_proxy_sink_t
{
    uint32_t opcode;
    /**< ID of the data payload. */

    uint32_t payload_length;
    /**< Length of the data payload. */
#ifdef __H2XML__
    uint8_t payload[0];
    /**< Data payload */
#endif
}imcl_param_id_enc_to_conn_proxy_sink_t;



/* Param id to send any info to the encoder from Conn Proxy Sink module through imcl*/
#define IMCL_PARAM_ID_CONN_PROXY_SINK_TO_ENC   0x080013C8

/* Payload of IMCL_PARAM_ID_CONN_PROXY_SINK_TO_ENC*/
typedef struct imcl_param_id_conn_proxy_sink_to_enc_t
{
	uint32_t opcode;
    /**< ID of the data payload. */

	uint32_t payload_length;
    /**< Length of the data payload. */

#ifdef __H2XML__
    uint8_t payload[0];
    /**< Data payload */
#endif
}imcl_param_id_conn_proxy_sink_to_enc_t;


/* clang-format on */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif //INTENT_ID_BT_ENCODER_FEEDBACK

#endif /* _IMCL_BT_ENCODER_FEEDBACK_H_ */
