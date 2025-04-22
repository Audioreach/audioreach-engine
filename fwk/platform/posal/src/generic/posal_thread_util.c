/**
 * \file posal_thread_util.c
 * \brief
 *  	This file contains utilities for threads
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "posal.h"


/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/

ar_result_t posal_thread_allocate_stack(uint32_t nStackSize,char **pStackpointer,POSAL_HEAP_ID heap_id)
{
   *pStackpointer = NULL;
   /* Allocate the stack pointer */
   if (NULL == (*pStackpointer = (char *)posal_memory_malloc(nStackSize, heap_id)))
   {
      AR_MSG(DBG_ERROR_PRIO, "THRD CREATE: Stack pointer allocation failed");
      return AR_ENOMEMORY;
   }
  
   return AR_EOK;
}
