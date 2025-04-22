#ifndef AR_OSAL_MEM_OP_H
#define AR_OSAL_MEM_OP_H

 /**
  * \file ar_osal_mem_op.h
  * \brief
  *      Defines public APIs for memory operations.
  * \copyright
  *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
  *  SPDX-License-Identifier: BSD-3-Clause-Clear
  */

#include "ar_osal_types.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

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
int32_t ar_mem_cpy(void* dest, size_t dest_size, const void* src, size_t size);

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
int32_t ar_mem_move(void* dest, size_t dest_size, const void* src, size_t size);

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
int32_t ar_mem_set(void* dest, int32_t c, size_t size);

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
int32_t ar_mem_cmp(const void* buff1, const void* buff2, size_t size);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef AR_OSAL_MEM_OP_H */

