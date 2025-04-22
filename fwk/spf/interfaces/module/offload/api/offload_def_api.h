/**
 *   \file offload_def_api.h
 *   \brief
 *        This file contains Shared mem module APIs extension for Read shared memory Endpoint module
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef OFFLOAD_DEF_API_H_
#define OFFLOAD_DEF_API_H_

/*------------------------------------------------------------------------
 * Include files
 * -----------------------------------------------------------------------*/
#include "ar_defs.h"
#include "ar_guids.h"

#define CLIENT_ID_HLOS 0
/* definition for HLOS as client*/

#define CLIENT_ID_OLC 1
/* definition for OLC as client*/

/* Offload EP module type */
typedef enum {
   WR_SHARED_MEM_EP     = 0,
   RD_SHARED_MEM_EP     = 1,
   WR_SHARED_MEM_CLIENT = 2,
   RD_SHARED_MEM_CLIENT = 3
} offload_ep_module_type_t;

/** Satellite GPR command */
#define SPF_MSG_CMD_SATELLITE_GPR 0x01001045

#endif // OFFLOAD_DEF_API_H_
