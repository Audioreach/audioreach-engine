/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/*============================================================================
  FILE:          simple_mm.c

  OVERVIEW:      Simple memory management system for allocating and
                 initializing data within a contiguous block.

  DEPENDENCIES:  None
============================================================================*/

/*----------------------------------------------------------------------------
 * Include Files
 * -------------------------------------------------------------------------*/

#include <string.h>
#include "simple_mm.h"
#include "AEEStdDef.h"

/*----------------------------------------------------------------------------
  Constants
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Externalized Function Definitions
 * -------------------------------------------------------------------------*/

/*======================================================================
  FUNCTION      smm_init

  DESCRIPTION   Initialize simple memory manager by connecting the mem
                pointer in smm to the given starting address.

  DEPENDENCIES  memstart must be 64-bit aligned

  PARAMETERS    smm: [in and out] handle to mem manager
                memstart: [in] starting address of memory

  RETURN VALUE  None

  SIDE EFFECTS  update smm->mem
======================================================================*/
void smm_init(SimpleMemMgr *smm, void *memstart)
{
   if (smm != NULL)
   {
      smm->mem = memstart;
   }
}

/*======================================================================
  FUNCTION      smm_malloc_size

  DESCRIPTION   Determine effective size that will be allocated
                by subsequent call to smm_malloc.
                Aligns by 64 bits.

  DEPENDENCIES  None

  PARAMETERS    size: [in] input size in bytes

  RETURN VALUE  Number of bytes that will be allocated by smm_malloc

  SIDE EFFECTS  None
======================================================================*/
size_t smm_malloc_size(size_t size)
{
   return ( ALIGN8(size) );
}

/*======================================================================
  FUNCTION      smm_calloc_size

  DESCRIPTION   Determine effective size that will be allocated
                by subsequent call to smm_calloc.
                Aligns by 64 bits.

  DEPENDENCIES  None

  PARAMETERS    nmemb: [in] number of elements to allocate
                size: [in] size in bytes of each element

  RETURN VALUE  Number of bytes that will be allocated by smm_calloc

  SIDE EFFECTS  None
======================================================================*/
size_t smm_calloc_size(size_t nmemb, size_t size)
{
   return ( ALIGN8(nmemb * size) );
}

size_t smm_calloc_size_4kaligned(size_t nmemb, size_t size)
{
   return ( 2 * ALIGN4096(nmemb * size) );
}

/*======================================================================
  FUNCTION      smm_malloc

  DESCRIPTION   Allocate a block of memory of the given size,
                aligned on 64-bit boundary.  Move smm->mem
                to the next available 64-bit aligned address.

  DEPENDENCIES  smm must be initialized by call to smm_init

  PARAMETERS    smm: [in and out] handle to memory manager
                size: [in] number of bytes to allocate

  RETURN VALUE  pointer to the allocated data

  SIDE EFFECTS  update smm->mem
======================================================================*/
void *smm_malloc(SimpleMemMgr *smm, size_t size)
{
   int8 *p = NULL;
   int8 *p1 = NULL;
   if (smm != NULL)
   {
      p = (int8 *) smm->mem;
      p1 = p + ALIGN8(size);
      smm->mem = p1;
   }
   return p;
}

void *smm_malloc_4kaligned(SimpleMemMgr *smm, size_t size)
{
   int8 *p = NULL;
   int8 *p1 = NULL;

   uint32 *ptr;
   if (smm != NULL)
   {
      ptr = (uint32 *) smm->mem;
      p = (int8 *)ALIGN4096(ptr);
      p1 = p + ALIGN8(size);
      smm->mem = p1;
   }
   return p;
}

/*======================================================================
  FUNCTION      smm_calloc

  DESCRIPTION   Allocate a block of memory of the given size,
                aligned on 64-bit boundary.  Clear the allocated
                region to 0.  Move smm->mem to the next available
                64-bit aligned address.

  DEPENDENCIES  smm must be initialized by call to smm_init

  PARAMETERS    smm: [in and out] handle to memory manager
                nmemb: [in] number of elements
                size: [in] size of each element in bytes

  RETURN VALUE  pointer to the allocated data

  SIDE EFFECTS  update smm->mem
======================================================================*/
void *smm_calloc(SimpleMemMgr *smm, size_t nmemb, size_t size)
{
   void *p = NULL;
   size_t total = nmemb * size;

   if (smm != NULL)
   {
      p = smm_malloc(smm, total);
      if (p != NULL)
      {
         memset(p, 0, total);
      }
   }

   return p;
}


void *smm_calloc_4kaligned(SimpleMemMgr *smm, size_t nmemb, size_t size)
{
   void *p = NULL;
   size_t total = nmemb * size;

   if (smm != NULL)
   {
      p = smm_malloc_4kaligned(smm, total);
      if (p != NULL)
      {
         memset(p, 0, total);
      }
   }

   return p;
}

/*======================================================================
  FUNCTION      smm_free

  DESCRIPTION   Free allocated memory.  Does not do anything in this
                implementation.

  DEPENDENCIES  None

  PARAMETERS    smm: [in and out] handle to memory manager
                ptr: [in] pointer to previously allocated memory

  RETURN VALUE  None

  SIDE EFFECTS  None
======================================================================*/
void smm_free(SimpleMemMgr *smm, void *ptr)
{
   /* No-op in this implementation. */
}
