/**
@file irm_sysmon_util.h

@brief Wrapper interface between IRM-Sysmon or IRM-SIM.

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#ifndef _IRM_SYSMON_UTIL_H_
#define _IRM_SYSMON_UTIL_H_

#include "irm_i.h"
#include "irm_cntr_prof_util.h"

#if defined(SIM) || !defined(AVS_USES_SYSMON)
#define MAX_QUERY_SIZE 10
/**
 * @struct sysmon_audio_query_t
 * @brief Input parameter type used to query for sysmon data from AUDIOPD
 */
typedef struct
{

   unsigned long long int sysclocktick;
   /**< Sys clock tick count at the time of sample
    *   collection */

   unsigned long long int pcycles;
   /**< Pcycle counter read from QuRT at the time of sample
    *   collection */

   unsigned long long int busVoteinAb;
   /**< ADSPPM static vote feedback for bus clock in Ab bps at the
    *   time of sample collection*/

   unsigned long long int busVoteinIb;
   /**< ADSPPM static vote feedback for bus clock in Ib bps at the
    *   time of sample collection*/

   unsigned int coreClkVoteinkHz;
   /**< ADSPPM static vote feedback for core clock in KHz at the
    *   time of sample collection*/

   unsigned int avsheap_total_bytes;
   /**< Total heap memory allocated in Audio PD at the time
    *   of sample collection */

   unsigned int avsheap_available_total;
   /**< Total available heap in Audio PD at the time of
    *   sample collection */

   unsigned int avsheap_available_max;
   /**< Maximum continuous chunk available in Audio PD
    *   heap at the time of sample collection */

   unsigned int pktcnt;
   /**< Free Running packet count at the time of sample
    *   collection */

   unsigned long long axi_rd_cnt;
   /**< Free Running AXI read byte count at the time of sample
    *   collection */

   unsigned long long axi_wr_cnt;
   /**< Free Running AXI write byte count at the time of sample
    *   collection */

   unsigned long long reserved[MAX_QUERY_SIZE];
   /**< Reserved field */
} sysmon_audio_query_t;
#else
#include "sysmon_audio_query.h"
#endif

int32_t irm_query_processor_metrics(sysmon_audio_query_t *query_ptr, uint64_t *idle_pcycles_ptr);
void irm_calculate_non_idle_cycles(irm_metric_id_q6_hw_info_t *q6_hw_info_ptr,
                                   uint32_t *                  pcyles_ptr,
                                   uint64_t *                  idle_cycles_ptr);
void irm_get_q6_hw_info(irm_metric_id_q6_hw_info_t *q6_hw_info_ptr);
int32_t irm_wrapper_profile_enable(uint32_t block_id, int32_t enable);
ar_result_t irm_process_register_module_event(irm_t *me_ptr, gpr_packet_t *gpr_pkt_ptr);
ar_result_t irm_tst_fwk_event_send(irm_t *irm_ptr, uint64_t wakeup_ts, uint64_t threshold_us);
ar_result_t irm_tst_fwk_override_event_send(irm_t *irm_ptr);
#endif /* _IRM_SYSMON_UTIL_H_ */
