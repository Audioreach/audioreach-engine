/**
 * \file ar_osal_heap.c
 *
 * \brief
 *      Defines public APIs for heap memory allocation.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */
#include "ar_osal_heap.h"
#include "ar_osal_error.h"
#include <stdlib.h>

/**
 * \brief ar_heap_init
 *        initialize heap memory interface.
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_heap_init(void) {
    return AR_EOK;
}

/**
 * \brief ar_heap_deinit.
 *       De-initialize heap memory interface.
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_heap_deinit(void) {
    return AR_EOK;
}

/**
 * \brief Allocates heap memory.
 *
 * \param[in] bytes: number of bytes to allocate heap memory.
 * \param[in] heap_info: pointer of type: ar_heap_info.
 *
 * \return
 *  Nonzero -- Success: pointer to the allocated heap memory
 *  NULL -- Failure
 *
 */
void* ar_heap_malloc(_In_ size_t bytes, _In_ par_heap_info heap_info)
{
    void* pBuff = NULL;
    size_t alignment = AR_HEAP_ALIGN_DEFAULT;
    if (NULL == heap_info) {
        goto end;
    }
    switch (heap_info->align_bytes) {
        case AR_HEAP_ALIGN_4_BYTES: /** 4-byte boundary */
            alignment = 4;
            break;
        case AR_HEAP_ALIGN_8_BYTES: /** 8-byte boundary */
            alignment = 8;
            break;
        case AR_HEAP_ALIGN_16_BYTES: /** 16-byte boundary */
            alignment = 16;
            break;
        case AR_HEAP_ALIGN_DEFAULT: /** Default */
            break;
    }
    if (AR_HEAP_ALIGN_DEFAULT == heap_info->align_bytes) {
        pBuff = malloc(bytes);
    } else {
        if (alignment < sizeof(void *)) {
            alignment = sizeof(void *);
        }
        if (posix_memalign(&pBuff, alignment, bytes) != 0) {
            pBuff = NULL;
        }
    }

end:
    return pBuff;
}


/**
 * \brief Allocates heap memory and initialize with 0.
 *
 * \param[in] bytes: number of bytes to allocate heap memory.
 * \param[in] heap_info: pointer of type: ar_heap_info.
 *
 * \return
 *  Nonzero -- Success: pointer to the allocated heap memory
 *  NULL -- Failure
 *
 */
void* ar_heap_calloc(_In_ size_t bytes, _In_ par_heap_info heap_info)
{
    void* pBuf = NULL;
    pBuf = ar_heap_malloc(bytes, heap_info);
    if (NULL != pBuf) {
        memset(pBuf, 0, bytes);
    }
    return pBuf;
}

/**
 * \brief Frees heap memory.
 *
 * \param[in] heap_ptr: pointer to heap memory obtained from ar_heap_alloc().
 *
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
_IRQL_requires_max_(DISPATCH_LEVEL)
void ar_heap_free(_In_ void* heap_ptr, _In_ par_heap_info heap_info)
{
    if (NULL != heap_ptr && NULL != heap_info) {
        free(heap_ptr);
    }
    return;
}

