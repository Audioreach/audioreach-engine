/**
 *  \file imcl_deadline_time_api.h
 *  
 *  \brief
 *        This file contains API's to send the deadline time after which Gate module can start processing data
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _IMCL_DEADLINE_TIME_H_
#define _IMCL_DEADLINE_TIME_H_

/*------------------------------------------------------------------------------
 *  Header Includes
 *----------------------------------------------------------------------------*/
#include "imcl_fwk_intent_api.h"

#ifdef INTENT_ID_BT_DEADLINE_TIME

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/** -----------------------------------------------------------------------
 ** Type Declarations
 ** ----------------------------------------------------------------------- */

/* clang-format off */

/** This param is used by cop depack, conn proxy sink to send the deadline time to the
    the Gate module */
#define IMCL_PARAM_ID_BT_DEADLINE_TIME            0x080010D4


typedef struct param_id_imcl_bt_deadline_time param_id_imcl_bt_deadline_time;

struct param_id_imcl_bt_deadline_time
{
	 bool_t     open_gate_immediately;
	 /**< Indication to gate module to open the gate directly
	  *  Connectivity proxy sink to set this in a2dp use cases */

	 bool_t     is_ep_transmit_delay_valid;
	 /**< Flag to indicate if Endpoint transmit delay field below is valid*/

	 uint32_t   ep_transmit_delay_us;
	 /**< Endpoint transmit delay
	  *   If is_ep_transmit_delay_valid is set, this field should be considered in gate module
	  *   deadline time calculation */

	 int64_t    deadline_time_us;
	 /**< Deadline time */

	 uint16_t   frame_interval_us;
	 /**< Frame duration in microseconds. This indicates the encoder and decoder frame durations.*/

	 uint16_t   frame_size_bytes;
	 /**< Frame size of the encoded data. It's the max payload size that encoder is expected to provide for each to-air interval. */
};

/* clang-format on */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif // INTENT_ID_BT_DEADLINE_TIME

#endif /* _IMCL_DEADLINE_TIME_H_ */
