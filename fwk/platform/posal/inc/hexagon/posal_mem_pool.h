#ifndef __POSAL_MEM_POOL_QDI_H_
#define __POSAL_MEM_POOL_QDI_H_

/*==============================================================================
  @brief POSAL_MEM_POOL header

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/

/*==============================================================================
   Includes
==============================================================================*/
#include "posal_qdi.h"
#include "qurt_qdi_driver.h"
#include "qurt_types.h"
#include "qurt_qdi.h"

#define POSAL_MEM_POOL_PAGE_TABLE_SHIFT_VALUE 12

/* Add pages to pool dynamically */
ar_result_t posal_mem_pool_add_pages(qurt_mem_pool_t pool, uint32_t first_pageno, uint32_t size_in_pages);

/* Remove pages from pool dynamically */
ar_result_t posal_mem_pool_remove_pages(qurt_mem_pool_t pool, uint32_t first_pageno, uint32_t size_in_pages);

/* qdi command handler for posal mem pool apis */
ar_result_t posal_mem_pool_qdi_cmd_handler(int32_t        client_handle,
                                           qurt_qdi_arg_t a1,
                                           qurt_qdi_arg_t a2,
                                           qurt_qdi_arg_t a3,
                                           qurt_qdi_arg_t a4,
                                           qurt_qdi_arg_t a5,
                                           qurt_qdi_arg_t a6,
                                           qurt_qdi_arg_t a7,
                                           qurt_qdi_arg_t a8,
                                           qurt_qdi_arg_t a9);

/* create AUDIO_DYNAMIC_POOL */
ar_result_t posal_mem_pool_init();

#endif // __POSAL_MEM_POOL_QDI_H_
