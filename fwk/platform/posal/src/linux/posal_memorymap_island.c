/**
 * \file posal_memorymap.c
 * \brief
 *  This file contains a utility for memory mapping and unmapping shared memory, LPM etc.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "posal.h"

/* ----------------------------------------------------------------------------
 * Global Declarations/Definitions
 * ------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
 * Function Definitions
 * ------------------------------------------------------------------------- */
/**
  Given a virtual address, retrieves the corresponding mapping entry from
  the page table, adjusts the offset based on the page size, and returns the
  full 64-bit physical address back to the user.

  @param[in] virt_addr    Starting virtual address.

  @return
  Physical address if the address is mapped; otherwise, 0.

  @dependencies
  The client object must have been registered and the corresponding memory
  mapped before calling this function.
 */
uint64_t posal_memorymap_get_physical_addr_v2(void *virt_addr_ptr)
{
   return 0;
}