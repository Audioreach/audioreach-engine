/**
@file irm_cntr_prof_util.h

@brief IRM container profile util.

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#ifndef _IRM_CNTR_PROF_UTIL_H_
#define _IRM_CNTR_PROF_UTIL_H_

#include "ar_error_codes.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/
#define IRM_MAX_NUM_HW_THREADS 6

#if defined(AVS_USES_QURT_PROF)
#define IRM_PROFILE_MODULE_PROCESS_BEGIN(prof_info_ptr)                                                                \
   uint64_t pcycles_before[IRM_MAX_NUM_HW_THREADS];                                                                    \
   uint64_t pcycles_after[IRM_MAX_NUM_HW_THREADS];                                                                     \
   uint64_t pcycles_total;                                                                                             \
   uint64_t pktcnt_before;                                                                                             \
   uint64_t pktcnt_after;                                                                                              \
   uint32_t thread_id;                                                                                                 \
   if (prof_info_ptr)                                                                                                  \
   {                                                                                                                   \
      memset(pcycles_before, 0, sizeof(pcycles_before));                                                               \
      memset(pcycles_after, 0, sizeof(pcycles_after));                                                                 \
      pcycles_total = 0;                                                                                               \
      pktcnt_before = 0;                                                                                               \
      pktcnt_after  = 0;                                                                                               \
      thread_id     = posal_thread_get_curr_tid();                                                                     \
      qurt_profile_get_threadid_pcycles(thread_id, &pcycles_before[0]);                                                \
      pktcnt_before = qurt_thread_pktcount_get(thread_id);                                                             \
   }

#define IRM_PROFILE_MODULE_PROCESS_END(prof_info_ptr, prof_mutex)                                                      \
   if (prof_info_ptr)                                                                                                  \
   {                                                                                                                   \
      pktcnt_after = qurt_thread_pktcount_get(thread_id);                                                              \
      qurt_profile_get_threadid_pcycles(thread_id, &pcycles_after[0]);                                                 \
      for (uint32_t hw_idx = 0; hw_idx < IRM_MAX_NUM_HW_THREADS; hw_idx++)                                             \
      {                                                                                                                \
         pcycles_total += pcycles_after[hw_idx] - pcycles_before[hw_idx];                                              \
      }                                                                                                                \
                                                                                                                       \
      if (prof_mutex)                                                                                                  \
      {                                                                                                                \
         posal_mutex_lock(prof_mutex);                                                                                 \
         prof_info_ptr->accum_pcylces += pcycles_total;                                                                \
         prof_info_ptr->accum_pktcnt += (pktcnt_after - pktcnt_before);                                                \
         posal_mutex_unlock(prof_mutex);                                                                               \
      }                                                                                                                \
   }
#else

#define IRM_PROFILE_MODULE_PROCESS_BEGIN(prof_info_ptr)
#define IRM_PROFILE_MODULE_PROCESS_END(prof_info_ptr, prof_mutex)

#endif

#if defined(AVS_USES_QURT_PROF)
#define IRM_PROFILE_MOD_PROCESS_SECTION(prof_info_ptr, prof_mutex, XX_CODE_SECTION_XX)                                 \
   do                                                                                                                  \
   {                                                                                                                   \
      if (prof_info_ptr)                                                                                               \
      {                                                                                                                \
         uint64_t pcycles_before[IRM_MAX_NUM_HW_THREADS] = { 0 };                                                      \
         uint64_t pcycles_after[IRM_MAX_NUM_HW_THREADS]  = { 0 };                                                      \
         uint64_t pcycles_total                          = 0;                                                          \
         uint64_t pktcnt_before                          = 0;                                                          \
         uint64_t pktcnt_after                           = 0;                                                          \
         uint32_t thread_id                              = posal_thread_get_curr_tid();                                \
         qurt_profile_get_threadid_pcycles(thread_id, &pcycles_before[0]);                                             \
         pktcnt_before = qurt_thread_pktcount_get(thread_id);                                                          \
                                                                                                                       \
         XX_CODE_SECTION_XX                                                                                            \
                                                                                                                       \
         pktcnt_after = qurt_thread_pktcount_get(thread_id);                                                           \
         qurt_profile_get_threadid_pcycles(thread_id, &pcycles_after[0]);                                              \
         for (uint32_t hw_idx = 0; hw_idx < IRM_MAX_NUM_HW_THREADS; hw_idx++)                                          \
         {                                                                                                             \
            pcycles_total += pcycles_after[hw_idx] - pcycles_before[hw_idx];                                           \
         }                                                                                                             \
                                                                                                                       \
         if (prof_mutex)                                                                                               \
         {                                                                                                             \
            posal_mutex_lock(prof_mutex);                                                                              \
            prof_info_ptr->accum_pcylces += pcycles_total;                                                             \
            prof_info_ptr->accum_pktcnt += (pktcnt_after - pktcnt_before);                                             \
            posal_mutex_unlock(prof_mutex);                                                                            \
         }                                                                                                             \
      }                                                                                                                \
      else                                                                                                             \
      {                                                                                                                \
         XX_CODE_SECTION_XX                                                                                            \
      }                                                                                                                \
   } while (0)
#else

#define IRM_PROFILE_MOD_PROCESS_SECTION(prof_info_ptr, prof_mutex, XX_CODE_SECTION_XX)                                 \
   do                                                                                                                  \
   {                                                                                                                   \
      XX_CODE_SECTION_XX                                                                                               \
   } while (0)

#endif

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* _IRM_CNTR_PROF_UTIL_H_ */
