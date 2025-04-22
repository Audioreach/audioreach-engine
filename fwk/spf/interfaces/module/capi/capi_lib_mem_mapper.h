#ifndef ELITE_LIB_MEM_MAPPER_H
#define ELITE_LIB_MEM_MAPPER_H

/**
 *   \file capi_lib_mem_mapper.h
 *   \brief
 *        A CAPI supporting library for a memory mapper
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*------------------------------------------------------------------------------
 * Include Files
 *----------------------------------------------------------------------------*/
#include "capi.h"
#include "apm_memmap_api.h"

/** @addtogroup capi_lib_mem_map
The Memory Mapper library (ELITE_LIB_MEM_MAPPER) is used to map and unmap
memory buffers that are shared by another subsystem, such as the HLOS.
*/

/** @addtogroup capi_lib_mem_map
@{ */

/** Unique identifier of the Memory Mapper library.

  @subhead{To use this library}
  -# Get an object of type #elite_lib_mem_mapper_t using
     #ELITE_LIB_MEM_MAPPER. \n @vertspace{1.5}
     The object is an instance of the library, together with the virtual
     function table (#elite_lib_mem_mapper_vtable) for the library interface.
  -# Call map_memory() to map any physical memory regions to virtual memory.
  -# Call get_virt_addr() to get the virtual address of specified physical
     memory.
  -# Call unmap_memory() to remove the mapping from physical memory to virtual
     memory.
*/
#define ELITE_LIB_MEM_MAPPER 0x0001321B

/*------------------------------------------------------------------------------
 * Interface definition
 *----------------------------------------------------------------------------*/
typedef struct elite_lib_mem_mapper_t elite_lib_mem_mapper_t;

typedef struct elite_lib_mem_mapper_vtable elite_lib_mem_mapper_vtable;

/** Virtual function table for the library interface.
*/
struct elite_lib_mem_mapper_vtable
{
   /** Maps to the base interface for the library. @newpage */
   capi_library_base_t b;

   /**
     Maps the shared memory provided by the HLOS.

     @datatypes
     #elite_lib_mem_mapper_t \n
     apm_shared_map_region_payload_t (see apm_memmap_api.h)

     @param[in] obj_ptr         Pointer to the instance of this object.
     @param[in] mem_pool_id     Memory pool ID passed by the HLOS.
     @param[in] property_flag   Property flag of the memory passed by the HLOS.
     @param[in] is_cached       Specifies whether to map the memory as cached
                                (TRUE) or not cached (FALSE).
     @param[in] num_regions     Number of elements in the regions_array.
     @param[in] regions_array   Pointer to the array of regions to be mapped
                                into a virtual space.
     @param[in] mem_map_handle  Pointer to the memory mapping handle.

     @return
     Memory map handle.

     @dependencies
     None. @newpage
   */
   capi_err_t (*map_memory)(elite_lib_mem_mapper_t *         obj_ptr,
                            uint16_t                         mem_pool_id,
                            uint16_t                         property_flag,
                            bool_t                           is_cached,
                            uint16_t                         num_regions,
                            apm_shared_map_region_payload_t *regions_array,
                            uint32_t *                       mem_map_handle);

   /**
     Unmaps the memory.

     @datatypes
     #elite_lib_mem_mapper_t

     @param[in] obj_ptr         Pointer to the instance of this object.
     @param[in] mem_map_handle  Handle of the memory mapping returned by
                                map_memory().

     @return
     #CAPI_EOK -- Success
     @par
     Error code -- Failure (see Section @xref{hdr:errorCodes})

     @dependencies
     The memory region must have been mapped previously using map_memory().
     @newpage
   */
   capi_err_t (*unmap_memory)(elite_lib_mem_mapper_t *obj_ptr, uint32_t mem_map_handle);

   /**
     Gets the virtual address of the mapped memory region.

     @datatypes
     elite_lib_mem_mapper_t

     @param[in] obj_ptr         Pointer to the instance of this object.
     @param[in] mem_map_handle  Handle of the memory mapping returned by
                                map_memory().
     @param[in] phy_addr_lsw    LSW of the physical address to be unmapped.
     @param[in] phy_addr_lsw    MSW of the physical address to be unmapped.
     @param[out] virt_addr      Pointer to the virtual address that
                                corresponds to the physical address.

     @return
     Virtual address.

     @dependencies
     The memory region must have been mapped previously using map_memory().
   */
   capi_err_t (*get_virt_addr)(elite_lib_mem_mapper_t *obj_ptr,
                               uint32_t                mem_map_handle,
                               uint32_t                phy_addr_lsw,
                               uint32_t                phy_addr_msw,
                               uint32_t                reg_size,
                               uint32_t *              virt_addr);
};

typedef struct elite_lib_mem_mapper_t elite_lib_mem_mapper_t;

/** Contains a pointer to the memory mapper virtual function table that is
    defined in #elite_lib_mem_mapper_vtable.
 */
struct elite_lib_mem_mapper_t
{
   const elite_lib_mem_mapper_vtable *vtable;
   /**< Pointer to the virtual function table. */
};

/** @} */ /* end_addtogroup capi_lib_mem_map */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* #ifndef ELITE_LIB_MEM_MAPPER_H */
