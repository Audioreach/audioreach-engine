#ifndef AR_OSAL_SHMEM_H
#define AR_OSAL_SHMEM_H

/**
 * \file ar_osal_shmem.h
 * \brief
 *     Defines public APIs for shared memory allocation for DSP.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_osal_types.h"
#include "ar_osal_sys_id.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/


 /** enum for shmem memory type*/
typedef enum ar_shmem_memory_type
{
	/** 0  Shared physical memory allocation.*/
	AR_SHMEM_PHYSICAL_MEMORY = 0,
	/** 1  Shared virtual memory allocation */
	AR_SHMEM_VIRTUAL_MEMORY = 1
}ar_shmem_memory_type_t;

/** enum for shmem cache type*/
typedef enum ar_shmem_cache_type
{
	/** 0  cached.*/
	AR_SHMEM_CACHED = 0,
	/** 1  uncached.*/
	AR_SHMEM_UNCACHED = 1
}ar_shmem_cache_type_t;

/** enum for shmem offset/address buffer index type*/
typedef enum ar_shmem_buffer_index_type
{
	/** 0  use physical or virtual addresses.*/
	AR_SHMEM_BUFFER_ADDRESS = 0,
	/** 1  use offsets, the offset is from the base address.*/
	AR_SHMEM_BUFFER_OFFSET = 1
}ar_shmem_buffer_index_type_t;

/**< Bits to indicate if hardware accelerator is enabled/disabled */
#define AR_SHMEM_HW_ACCELERATOR_ENABLED 0x1
#define AR_SHMEM_HW_ACCELERATOR_DISABLED 0x0

/**< Bit mask for hardware accelerator flag */
#define AR_SHMEM_BIT_MASK_HW_ACCELERATOR_FLAG 0x1
/**< Shift amount for hardware accelerator setup flag */
#define AR_SHMEM_SHIFT_HW_ACCELERATOR_FLAG 0x0

 /**
 * Shared memory info structure
 */
typedef struct ar_shmem_info_t
{
	ar_shmem_cache_type_t            cache_type;     /**< in, cache type, cached or uncached memory */
	size_t                           buf_size;       /**< in, shared buffer size, should be a minimum of 4K and multiple of 4K only*/
	ar_shmem_memory_type_t           mem_type;       /**< out, shmem memory type, virtual or physical for ar_shmem_alloc()*/
													 /**< in, shmem memory type, virtual or physical for ar_shmem_map()*/
	ar_shmem_buffer_index_type_t     index_type;     /**< out, DSP to operate on buffer offsets or on address pointers.*/
	uint32_t                         ipa_lsw;        /**< out, smmu mapped ipa lsw, output for ar_shmem_alloc()/ar_shmem_map() */
	uint32_t                         ipa_msw;        /**< out, smmu mapped ipa msw, output for ar_shmem_alloc()/ar_shmem_map() */
	uint32_t                         pa_lsw;         /**< out, physical address lsw, alignment requirements apply, like 4k,
															   start address multiple of 64, output of ar_shmem_alloc()*/
															   /**< in, physical address lsw, alignment requirements apply, like 4k,
															   start address multiple of 64, input for ar_shmem_map()*/
	uint32_t                         pa_msw;         /**< out, physical address msw, alignment requirements apply, like 4k,
															   start address multiple of 64, output of ar_shmem_alloc()*/
															   /**< in, physical address lsw, alignment requirements apply, like 4k
															    start address multiple of 64, input for ar_shmem_map()*/
	void                            *vaddr;          /**< out, virtual address 64bit/32bit, alignment requirements apply, like 4k,
															   start address multiple of 64, input for ar_shmem_alloc()*/
	uint64_t                         metadata;       /**< out opt, pointer address to metadata structure defined by each platform
																   for ar_shmem_alloc()*/
	uint8_t                          num_sys_id;     /**< in, number of subsystem IDs provided with ar_shmem_alloc()/ar_shmem_map() call.*/
	uint8_t                         *sys_id;         /**< in, pointer to array of size num_sys_id for sub-system Ids provided
														  in ar_osal_sys_id.h, used to allocate shared memory between the given
														  list of sys_id provided with ar_shmem_alloc()/ar_shmem_map() call.*/
	uint32_t                         platform_info;  /**< in opt, optional field for passing platform specific data to OSAL, this can be
														  used for example to communicate some heap properties provided only for ar_shmem_alloc()*/
	uint32_t                         flags;          /**< in, Bit field for flags.
                                                            @values{for bit 0}
                                                            - 1 -- hardware accelerator enabled, use AR_SHMEM_HW_ACCELERATOR_ENABLED
                                                            - 0 -- hardware accelerator disabled, use AR_SHMEM_HW_ACCELERATOR_DISABLED
                                                            - To set this bit, use #AR_SHMEM_BIT_MASK_HW_ACCELERATOR_FLAG and
                                                              AR_SHMEM_SHIFT_HW_ACCELERATOR_FLAG
                                                            All other bits are reserved; must be set to 0. */
} ar_shmem_info;

/**
* Shared memory physical address and size details for hyp assign.
*/
typedef struct ar_shmem_hyp_assign_phys_addr_t
{
	uint64_t			phys_addr;		/**< 64-bit physical address. */
	size_t				size;			/**< size in bytes for the buffer pointed by phys_addr*/
} ar_shmem_hyp_assign_phys_addr;

/**
* Destination sub system permission bit mask types supported.
* Combination of permissions can be used as per requirements.
*/
typedef enum  ar_shmem_hyp_assign_dest_sys_perm_t
{
    DEST_SYS_PERM_INVALID      = 0x0, /**< Invalid permission bit mask. */
    DEST_SYS_PERM_EXEC         = 0x1, /**< Execute permission bit mask. */
    DEST_SYS_PERM_WRITE_ONLY = 0x2,    /**< Write only permission bit mask. */
    DEST_SYS_PERM_EXEC_WRITE   = (DEST_SYS_PERM_EXEC | DEST_SYS_PERM_WRITE_ONLY), /**< Execute and Write permission.*/
    DEST_SYS_PERM_READ_ONLY    = 0x4, /**< Read only permission bit mask. */
    DEST_SYS_PERM_EXEC_READ    = (DEST_SYS_PERM_EXEC | DEST_SYS_PERM_READ_ONLY), /**< Execute and Read permission. */
    DEST_SYS_PERM_WRITE_READ   = (DEST_SYS_PERM_WRITE_ONLY | DEST_SYS_PERM_READ_ONLY), /**< Write and Read permission. */
    DEST_SYS_PERM_EXEC_WRITE_READ = (DEST_SYS_PERM_EXEC | DEST_SYS_PERM_WRITE_ONLY | DEST_SYS_PERM_READ_ONLY), /**< Execute, Write and Read permission. */
} ar_shmem_hyp_assign_dest_sys_perm;

/**
* Destination sub system id details and permission required for hyp assign.
*/
typedef struct ar_shmem_hyp_assign_dest_sys_info_t
{
	uint64_t                            dest_sys_id;   /**< destination sub system id, refer to ar_osal_sys_id.h */
	ar_shmem_hyp_assign_dest_sys_perm    dest_perm;     /**< destination permissions */
} ar_shmem_hyp_assign_dest_sys_info;

/**
* Hyp assign physical memory info structure.
*/
typedef struct ar_shmem_hyp_assign_phys_info_t
{
	ar_shmem_hyp_assign_phys_addr    *phys_addr_list;       /**< list of hyp_assign_phys_addr structures.*/
	uint32_t                          phys_addr_list_size;  /**< number of phys_addr_list entries.*/
	uint64_t                          *src_sys_list;        /**< source sub system Id list, refer to ar_osal_sys_id.h for sys ids.*/
	uint32_t                          src_sys_list_size;    /**< number of src_sys_list entries.*/
	ar_shmem_hyp_assign_dest_sys_info *dest_sys_list;       /**< destination sys list hyp_assign_dest_sys_info.*/
	uint32_t                          dest_sys_list_size;   /**< number of destination sys list/dest_sys_list entires.*/
	uint64_t                            metadata;           /**< in opt, pointer address to metadata structure defined by platform
                                                                   during ar_shmem_alloc() call.*/
} ar_shmem_hyp_assign_phys_info;

/**
 * \brief ar_shmem_init
 *        initialize shared memory interface.
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_shmem_init(void);

/**
 * \brief Allocates shared memory.
 *  Only non cached memory allocation supported.
 *  Size if multiple of 4KB and the returned is aligned to 4KB boundary.
 *  Buffer start address should be atleast 64bit multiple aligned.
 *
 * \param[in_out] info: pointer to ar_shmem_info.
 *
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 *
 */
int32_t ar_shmem_alloc(ar_shmem_info *info);

/**
 * Frees shared memory.
 *
 * \param[in] info: pointer to ar_shmem_info.
 *
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_shmem_free(ar_shmem_info *info);

/**
 * \brief Helps map memory with SMMU an already allocated shared memory
 *  for a give sub system.
 *  Size should be multiple of 4KB boundary.
 *  Buffer start address should be 64bit aligned.
 *
 * \param[in_out] info: pointer to ar_shmem_info.
 *                      required input parameters in ar_shmem_info
 *                        ar_shmem_info_t.cache_type
 *                        ar_shmem_info_t.buf_size
 *                        ar_shmem_info_t.mem_type
 *                        ar_shmem_info_t.pa_lsw
 *                        ar_shmem_info_t.pa_msw
 *                        ar_shmem_info_t.num_sys_id
 *                        ar_shmem_info_t.sys_id
 *                      required output parameters in ar_shmem_info
 *                        ar_shmem_info_t.ipa_lsw
 *                        ar_shmem_info_t.ipa_msw
 *
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 *
 */
int32_t ar_shmem_map(ar_shmem_info *info);

/**
 * \brief Helps unmap the shared memory allocated externally with SMMU.
 *
 * \param[in]   info: pointer to ar_shmem_info.
 *                      required input parameters in ar_shmem_info
 *                        ar_shmem_info_t.cache_type
 *                        ar_shmem_info_t.buf_size
 *                        ar_shmem_info_t.mem_type
 *                        ar_shmem_info_t.pa_lsw
 *                        ar_shmem_info_t.pa_msw
 *                        ar_shmem_info_t.num_sys_id
 *                        ar_shmem_info_t.sys_id
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 *
 */
 int32_t ar_shmem_unmap(ar_shmem_info *info);

 /**
 * \brief Helps to hyp assign physical memory between source and destination sub systems.
 *
 * \param[in]   info: pointer to ar_shmem_hyp_assign_phys_info.

 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 *
 */
int32_t ar_shmem_hyp_assign_phys(ar_shmem_hyp_assign_phys_info *info);

/**
* \brief ar_shmem_get_uid.
* Get associated unique identifier(UID) for the shared memory
* pointed by alloc_handle, platform which doesn`t support UID
* should return alloc_handle as UID with expectation of
* alloc_handle being unique.
*
* \param[in]  alloc_handle: handle for the shared memory.
* \param[out] uid: unique identifier to the shmem.
* \return
*  0 -- Success
*  Nonzero -- Failure
*/
int32_t ar_shmem_get_uid(uint64_t alloc_handle, uint64_t *uid);

/**
 * \brief ar_shmem_deinit.
 *
 * \return
 *  0 -- Success
 *  Nonzero -- Failure
 */
int32_t ar_shmem_deinit(void);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef AR_OSAL_SHMEM_H */

