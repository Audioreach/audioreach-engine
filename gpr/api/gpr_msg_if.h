#ifndef __GPR_MSG_IF_H__
#define __GPR_MSG_IF_H__

/**
 * @file  gpr_msg_if.h
 * @brief This file contains GPR list APIs
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


/******************************************************************************
 * Includes                                                                    *
 *****************************************************************************/
#include "gpr_comdef.h"

/*****************************************************************************
 *  Core Messages                                                            *
 ****************************************************************************/

/** @addtogroup gpr_resp_opcodes
@{ */

/**
  Response message opcode that indicates a command has completed. All services
  and clients must handle this response opcode.

  @par Payload (gpr_ibasic_rsp_result_t)
  @table{weak__gpr__ibasic__rsp__result__t}
*/
#define GPR_IBASIC_RSP_RESULT ( 0x02001005 )

#include "gpr_pack_begin.h"

/* Payload of the GPR_IBASIC_RSP_RESULT command response message. */
typedef struct gpr_ibasic_rsp_result_t gpr_ibasic_rsp_result_t;

/** @weakgroup weak_gpr_ibasic_rsp_result_t
@{ */
struct gpr_ibasic_rsp_result_t
{
  uint32_t opcode;    /**< Command operation code that completed. */
  uint32_t status;    /**< Completion status (see Section
                           @xref{hdr:StatusCodes}). */
}
#include "gpr_pack_end.h"
;
/** @} */ /* end_weakgroup weak_gpr_ibasic_rsp_result_t */


/**
  Standard message opcode that indicates a command was accepted.

  The generation and processing of this event is optional. Clients that do not
  understand this event must drop it by freeing the received packet.

  @par Payload (gpr_ibasic_evt_accepted_t)
  @table{weak__gpr__ibasic__evt__accepted__t}
*/
#define GPR_IBASIC_EVT_ACCEPTED ( 0x02001006 )

#include "gpr_pack_begin.h"

/* Payload of the GPR_IBASIC_EVT_ACCEPTED message. */
typedef struct gpr_ibasic_evt_accepted_t gpr_ibasic_evt_accepted_t;

/** @weakgroup weak_gpr_ibasic_evt_accepted_t
@{ */
struct gpr_ibasic_evt_accepted_t
{
  uint32_t opcode;    /**< Operation code of the command that was accepted. */
}
#include "gpr_pack_end.h"
;
/** @} */ /* end_weakgroup weak_gpr_ibasic_evt_accepted_t */
/** @} */ /* end_addtogroup gpr_resp_opcodes */

#endif /* __GPR_MSG_IF_H__ */

