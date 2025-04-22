/**
 * \file ar_osal_mem_op.c
 *
 * \brief
 *      Defines public APIs for memory operations.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_osal_mem_op.h"
#include "ar_osal_error.h"

/**
 * \brief ar_mem_cpy
 *        copies bytes between buffers.
 * \param[in_out] dest: destinatiopn buffer to copy data.
 * \param[in] dest_size: destination buffer size.
 * \param[in] src: source buffer pointer to copy data from.
 * \param[in] size: bytes to copy from source buffer.
 * \return
 *  0 -- Success
 *  Nonzero -- Failure 
 */
_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_mem_cpy(_Inout_ void* dest, _In_ size_t dest_size, _In_ const void* src, _In_ size_t size)
{
    int32_t cStatus = AR_EOK;
    if (NULL == dest || NULL == src || 0 == dest_size || 0 == size ||
        dest_size < size) {
        cStatus = AR_EBADPARAM;
        goto end;
    }
    memcpy(dest, src, size);
end:
    return cStatus;
}

/**
 * \brief ar_mem_move
 *        copies bytes from source area to destination area.
 * \param[in_out] dest: destinatiopn buffer to copy data.
 * \param[in] dest_size: destination buffer size.
 * \param[in] src: source buffer pointer to copy data from.
 * \param[in] size: bytes to copy from source buffer.
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 * note: If some regions of the source area and the destination overlap, ar_mem_move 
 * ensures that the original source bytes in the overlapping region are copied before 
 * being overwritten.
 */
_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_mem_move(_Inout_ void* dest, _In_ size_t dest_size, _In_ const void* src, _In_ size_t size)
{
    int32_t cStatus = AR_EOK;
    if (NULL == dest || NULL == src || 0 == dest_size || 0 == size ||
        dest_size < size) {
        cStatus = AR_EBADPARAM;
        goto end;
    }
    memmove(dest, src, size);
end:
    return cStatus;
}

/**
 * \brief ar_mem_set
 *        set buffer to specified value.
 * \param[in_out] dest: destinatiopn buffer to set the specified value.
 * \param[in] c: specified value to set, of integer type.
 * \param[in] size: buffer bytes to set the value, must be multiple of specified value "c" data size.
 * \return
 *  0 -- Success
 *  Nonzero -- Failure 
 */
_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_mem_set(_Inout_ void* dest, _In_ int32_t c, _In_ size_t size)
{
    int32_t cStatus = AR_EOK;
    if (NULL == dest || 0 == size ) {
        cStatus = AR_EBADPARAM;
        goto end;
    }
    memset(dest, c, size);
end:
    return cStatus;
}

/**
 * \brief ar_mem_cmp
 *        compare two buffers.
 * \param[in] buff1: first buffer.
 * \param[in] buff2: second buffer.
 * \param[in] size: bytes to compare.
 * \return
 *  < 0   buff1 < buff2
 *    0   buff1 == buff2
 *  > 0   buff1 > buff2
 */
_IRQL_requires_max_(DISPATCH_LEVEL)
int32_t ar_mem_cmp(_In_ const void* buff1, _In_ const void* buff2, _In_ size_t size)
{
    /* no check for null input pointer or size as the system itself doesn`t provide it*/
    return memcmp(buff1, buff2, size);
}

