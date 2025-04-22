/**
 * \file posal_island.h
 * \brief
 *  	 This file contains island utilities' declarations.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_ISLAND_H
#define POSAL_ISLAND_H

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* -----------------------------------------------------------------------
** Global definitions/forward declarations
** ----------------------------------------------------------------------- */
typedef enum
{
   POSAL_ISLAND_HEAP_Q6_TCM        = 1, // Default Island Heap Type - Q6 TCM
   POSAL_ISLAND_HEAP_LPASS_TCM     = 2, // LPASS_TCM
   POSAL_ISLAND_HEAP_LLC           = 3, // LLC
   POSAL_ISLAND_HEAP_NUM_SUPPORTED = 3
} posal_island_heap_t;

/** Default island heap = Q6 TCM by default. */
extern POSAL_HEAP_ID spf_mem_island_heap_id;

/** Private Api for getting island heap id */
extern POSAL_HEAP_ID posal_private_get_island_heap_id_v2(uint32_t island_heap_type);

/** Private Api for getting posal mem type */
extern posal_mem_t posal_private_get_mem_type_from_heap_type(uint32_t island_heap_type);

/**
  This function process island exit.

  @return
  Indication of success (0) or failure (nonzero).

  @dependencies
   None.
*/
ar_result_t posal_island_trigger_island_exit(void);

/**
  Inline function to exit island when USES_AUDIO_IN_ISLAND is not defined

  @return
  Indication of success (0) or failure (nonzero).

  @dependencies
   None.
*/
#ifndef USES_AUDIO_IN_ISLAND
static inline ar_result_t posal_island_trigger_island_exit_inline(void)
{
   return AR_EOK;
}
#endif // USES_AUDIO_IN_ISLAND

/**
  This function Get island mode status.

  Returns a value indicating whether the underlying system is executing in island mode.

  @return
  0 - Normal mode.
  1 - Island mode.

  @dependencies
  None.
*/
bool_t posal_island_get_island_status(void);

/* Returns POSAL_HEAP_DEFAULT if island heap id type is not supported */
static inline POSAL_HEAP_ID posal_get_island_heap_id(void)
{
   return spf_mem_island_heap_id;
}

/* Returns POSAL_HEAP_DEFAULT if island heap id type is not supported */
static inline POSAL_HEAP_ID posal_get_island_heap_id_v2(posal_island_heap_t heap_type)
{
   return posal_private_get_island_heap_id_v2(heap_type);
}

/* Returns POSAL_HEAP_ID given a memory type */
static inline POSAL_HEAP_ID posal_get_heap_id(posal_mem_t mem_type)
{
   switch (mem_type)
   {
      case POSAL_MEM_TYPE_LOW_POWER:
      {
         return posal_get_island_heap_id();
      }
      case POSAL_MEM_TYPE_LOW_POWER_2:
      {
         return posal_get_island_heap_id_v2(POSAL_ISLAND_HEAP_LLC);
      }
      default: // POSAL_MEM_TYPE_DEFAULT
      {
         return POSAL_HEAP_DEFAULT;
      }
   }
}

/* Returns posal_mem_t for a given heap type */
static inline posal_mem_t posal_get_mem_type_from_heap_type(posal_island_heap_t heap_type)
{
   return posal_private_get_mem_type_from_heap_type(heap_type);
}

#if defined(AVS_USES_ISLAND_MEM_PROF)
/**
  This function returns current island memory usage
*/
uint32_t posal_island_get_current_mem_usage();
uint32_t posal_island_get_current_mem_usage_v2(posal_island_heap_t heap_type);

/**
  This function returns max island memory usage
*/
uint32_t posal_island_get_max_allowed_mem_usage();
uint32_t posal_island_get_max_allowed_mem_usage_v2(posal_island_heap_t heap_type);
#endif // AVS_USES_ISLAND_MEM_PROF

#ifdef __cplusplus
}
#endif //__cplusplus
#endif // #ifndef POSAL_ISLAND_H