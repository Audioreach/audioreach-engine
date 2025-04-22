#ifndef AR_OSAL_HEAP_H
#define AR_OSAL_HEAP_H

/**
 * \file ar_osal_heap.h
 * \brief
 *      Defines public APIs for heap memory allocation.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_osal_types.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/** enum for heap memory byte alignments*/
typedef enum _ar_heap_align_bytes
{
	AR_HEAP_ALIGN_DEFAULT = 0, /** default alignment */
	AR_HEAP_ALIGN_4_BYTES = 1, /** 4-byte boundary */
	AR_HEAP_ALIGN_8_BYTES = 2, /** 8-byte boundary */
	AR_HEAP_ALIGN_16_BYTES = 3, /** 16-byte boundary */
} ar_heap_align_bytes;

/** enum for heap memory ids */
typedef enum _ar_heap_id
{
	AR_HEAP_ID_DEFAULT = 0,   /** default heap id */
	AR_HEAP_ID_1 = 1,   /** custom heap id 1 */
	AR_HEAP_ID_2 = 2,   /** custom heap id 2 */
	AR_HEAP_ID_3 = 3,   /** custom heap id 3 */
	AR_HEAP_ID_4 = 4,   /** custom heap id 4 */
	AR_HEAP_ID_5 = 5,   /** custom heap id 5 */
	AR_HEAP_ID_6 = 6,   /** custom heap id 6 */
	AR_HEAP_ID_7 = 7,   /** custom heap id 7 */
	AR_HEAP_ID_8 = 8,   /** custom heap id 8 */
	AR_HEAP_ID_9 = 9,   /** custom heap id 9 */
	AR_HEAP_ID_10 = 10,  /** custom heap id 10 */
	AR_HEAP_ID_11 = 11,  /** custom heap id 11 */
} ar_heap_id;

/** enum for heap memory types */
typedef enum _ar_heap_pool_type
{
	/** default pool type, as supported by each platform. */
	AR_HEAP_POOL_DEFAULT = 0,
	/** allocated memory is nonpaged and executable—that is, instruction execution is enabled in this memory. */
	AR_HEAP_POOL_NON_PAGED_EXECUTE = 1,
	/** allocated memory is nonpaged and instruction execution is disabled. */
	AR_HEAP_POOL_NON_PAGED_NX = 2,
	/** allocated memory is pageable.*/
	AR_HEAP_POOL_PAGED = 4,
} ar_heap_pool_type;

/** default heap memory tag ASCII characters: 'LASO'->'OSAL' */
#define AR_HEAP_TAG_DEFAULT     (0x4c41534f)

/**
* Heap memory info structure
*/
typedef struct ar_heap_info_t
{
	ar_heap_align_bytes    align_bytes; /** heap memory byte alignment required.*/
	ar_heap_pool_type      pool_type;   /** pool type to allocate heap memory. */
	ar_heap_id             heap_id;     /** head id to allocate heap memory. */
	uint32_t                 tag;         /** used for tracing heap memory allocations.default:AR_HEAP_TAG_DEFAULT */
                                          /** Specify the pool tag as a non - zero character literal of one to to four */
                                          /** characters delimited by single quotation marks(for example, 'Tag1'). */
                                          /** The string is usually specified in reverse order(for example, '1gaT'). */
                                          /** Each ASCII character in the tag must be a value in the range 0x20 (space)to 0x7E (tilde).*/
} ar_heap_info, * par_heap_info;

/**
 * \brief ar_heap_init
 *        initialize heap memory interface.
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_heap_init(void);

/**
 * \brief ar_heap_deinit.
 *       De-initialize heap memory interface.
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_heap_deinit(void);

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
void* ar_heap_malloc(size_t bytes, par_heap_info heap_info);

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
void* ar_heap_calloc(size_t bytes, par_heap_info heap_info);

/**
 * \brief Frees heap memory.
 *
 * \param[in] heap_ptr: pointer to heap memory obtained from ar_heap_alloc().
 *
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
void ar_heap_free(void* heap_ptr, par_heap_info heap_info);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef AR_OSAL_HEAP_H */

