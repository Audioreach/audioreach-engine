/**
@file irm_sysmon_utils.cpp

@brief Wrapper for sysmon function calls.

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#include "irm_sysmon_util.h"

#include "amssheap.h"
#include "spf_svc_calib.h"

#if !defined(SIM) && defined(AVS_USES_SYSMON)

int32_t irm_wrapper_profile_enable(uint32_t block_id, int32_t enable)
{

   int32_t result = 0;
   switch (block_id)
   {
      case IRM_BLOCK_ID_PROCESSOR:
      {
         result = sysmon_audio_register(enable);
         break;
      }
      case IRM_BLOCK_ID_CONTAINER:
      case IRM_BLOCK_ID_MODULE:
      {
#if defined(AVS_USES_QURT_PROF)
         qurt_profile_enable(enable);
#endif
         break;
      }
      default:
      {
         break;
      }
   }
   return result;
}

int32_t irm_query_processor_metrics(sysmon_audio_query_t *query_ptr, uint64_t *idle_pcycles_ptr)
{
   return sysmon_audio_query(query_ptr);
}

#endif

#if !defined(SIM) && !defined(AVS_USES_SYSMON)
int32_t irm_wrapper_profile_enable(uint32_t block_id, int32_t enable)
{
   return 0;
}

int32_t irm_query_processor_metrics(sysmon_audio_query_t *query_ptr, uint64_t *idle_pcycles_ptr)
{
   return 0;
}
#endif
