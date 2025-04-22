/**
 * \file irm_pvt_api.h
 *  
 * \brief
 *        API header for Integrated Resource Monitor (IRM) APIs to test framework.
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _IRM_PVT_API_H_
#define _IRM_PVT_API_H_

/*----------------------------------------------------------------------------------------------------------------------
 Include files
 ---------------------------------------------------------------------------------------------------------------------*/
#include "ar_defs.h"

/**
  @ingroup irm_cmd_get_memory_stats
  This API is used by client to request information on spf memory consumption

  @gpr_hdr_fields
  Opcode -- IRM_CMD_GET_MEMORY_STATS

  @msg_payload
  None

  @return
  Opcode -- IRM_CMD_RSP_GET_MEMORY_STATS

  @dependencies
  None
 */
#define IRM_CMD_GET_MEMORY_STATS 0x01001011

/**
  @ingroup irm_cmd_rsp_get_memory_stats
  This API is sent in response to IRM_CMD_GET_MEMORY_STATS command
  The response payload returns statistics related to spf memory
  allocation use

  @gpr_hdr_fields
  Opcode -- IRM_CMD_RSP_GET_MEMORY_STATS

  @msg_payload
  irm_cmd_rsp_get_memory_stats_t

  @return
  None

  @dependencies
  None
 */
#define IRM_CMD_RSP_GET_MEMORY_STATS 0x02001004

/**
  This structure is the payload structure used by IRM_CMD_RSP_GET_MEMORY_STATS command.
  Contains the statistics related to spf memory allocation use. This can be used to
  detect how fully the spf heap is being used and to track memory allocation/free
  counts for memory leaks
 */
#include "spf_begin_pack.h"

struct irm_cmd_rsp_get_memory_stats_t
{
   uint32_t num_mallocs;
   /**< Number of memory allocations that have occurred since bootup or
        since statistics were reset. */

   uint32_t num_frees;
   /**< Number of times memory was freed since bootup or since
        statistics were reset. */

   uint32_t current_heap_use;
   /**< Current number of bytes allocated from the heap since statistics were
        reset for . */

   uint32_t peak_heap_use;
   /**< Peak number of bytes allocated from the heap since bootup or since
        statistics were reset for .*/

   uint32_t num_non_mallocs;
   /**< Number of non- memory allocations that have occurred since bootup
        or since statistics were reset. */

   uint32_t num_non_frees;
   /**< Number of times non- memory was freed since bootup or since
        statistics were reset. */

   uint32_t current_non_heap_use;
   /**< Current number of bytes allocated from the heap since statistics were
        reset for non-. */

   uint32_t peak_non_heap_use;
   /**< Peak number of bytes allocated from the heap since bootup or since
        statistics were reset for non-. */
   uint32_t num_nondefault_mallocs;
   /**< Number of  lpa and lpm heap memory allocations that have occurred since bootup or
            since statistics were reset. */

   uint32_t num_nondefault_frees;
   /**< Number of times  lpa  and lpm heap memory was freed since bootup or since
            statistics were reset. */

   uint32_t current_nondefault_heap_use;
   /**< Current number of bytes allocated from the lpa and lpm heap since statistics were
            reset for . */

   uint32_t peak_nondefault_heap_use;
   /**< Peak number of bytes allocated from the lpa and lpm heap since bootup or since
            statistics were reset for .*/
}

#include "spf_end_pack.h"
;
typedef struct irm_cmd_rsp_get_memory_stats_t irm_cmd_rsp_get_memory_stats_t;

/**
  @ingroup irm_cmd_reset_peak_heap_use
  Requests that the spf reset the peak heap usage to zero, as reported in
  profiling events and acknowledgments.

  @gpr_hdr_fields
  Opcode -- IRM_CMD_RESET_PEAK_HEAP_USE

  @msg_payload
  None

  @return
  None

  @dependencies
  None
 */
#define IRM_CMD_RESET_PEAK_HEAP_USE 0x01001012

#endif //_IRM_PVT_API_H_
