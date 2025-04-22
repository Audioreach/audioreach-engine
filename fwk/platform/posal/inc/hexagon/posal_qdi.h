#ifndef __POSAL_QDI_H_
#define __POSAL_QDI_H_

/*==============================================================================
  @brief POSAL qdi header

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

/*==============================================================================
   Includes
==============================================================================*/
#include "ar_error_codes.h"
#include "posal_types.h"

#define POSAL_QDI_NAME "/platform/posal"
#define POSAL_QDI_BASE 258
#define POSAL_QDI_CMD_HANDLER (POSAL_QDI_BASE + 0)

/* POSAL QDI cmds */
typedef enum
{
   POSAL_QDI_CMD_MEM_POOL_ADD_PAGES,
   POSAL_QDI_CMD_MEM_POOL_REMOVE_PAGES
} posal_qdi_cmd_t;

/* Initialize POSAL Driver QDI Interface */
ar_result_t posal_qdi_init();

/* De-Initialize POSAL Driver QDI Interface */
void posal_qdi_deinit();

#endif // __POSAL_QDI_H_
