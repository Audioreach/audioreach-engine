#ifndef __GPR_HEAP_I_H__
#define __GPR_HEAP_I_H__

/**
 * \file gpr_heap_i.h
 * \brief
 *  	This file contains GPR heap utils
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_osal_heap.h"
#include "gpr_api.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

// Populates output argument ar_heap_info based on the input GPR heap index and alignment.
static inline uint32_t gpr_populate_ar_heap_info(gpr_heap_index_t    gpr_heap_index,
                                                 ar_heap_align_bytes alignment,
                                                 ar_heap_info       *heap_info_ptr)
{
   heap_info_ptr->align_bytes = alignment;
   heap_info_ptr->tag         = AR_HEAP_TAG_DEFAULT;
   heap_info_ptr->pool_type   = AR_HEAP_POOL_DEFAULT;

   if (GPR_HEAP_INDEX_1 == gpr_heap_index)
   {
      heap_info_ptr->heap_id = AR_HEAP_ID_1;
   }
   else
   {
      heap_info_ptr->heap_id = AR_HEAP_ID_DEFAULT;
   }

   return AR_EOK;
}

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* __GPR_HEAP_I_H__ */
