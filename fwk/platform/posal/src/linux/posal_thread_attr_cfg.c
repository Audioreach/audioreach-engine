/**
 * @file posal_thread_attr_cfg.c
 *
 * @brief
 *       This file contains functional implementations that are exposed to the framework
 *       to be invoked in order to retrieve the thread priority.
 *       Also defines the appropriate static and dynamic LUTs.
 *
 *  @copyright
 *       Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *       SPDX-License-Identifier: BSD-3-Clause-Clear
 */


/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "posal_thread_attr_cfg_i.h"
#include <pthread.h>

#define SCHED_DEFAULT       0xFFFFFFFF //0xFFFFFFFF = IGNORE. SCHED_FIFO
#define AFFINITY_DEFAULT    0xFFFFFFFF

typedef struct dyn_attr_lookup_t
{
   posal_thread_prio_t  thread_prio;        //platform specific priority value.
   uint32_t             affinity_mask;      //bit mask
   uint32_t             sched_policy;       //SCHED_FIFO, SCHED_RR, SCHED_OTHER https://man7.org/linux/man-pages/man3/pthread_attr_setschedpolicy.3.html
   uint32_t             frame_duration_us;
} dyn_attr_lookup_t;

typedef struct static_attr_lookup_t
{
   posal_thread_prio_t     thread_prio;
   uint32_t                affinity_mask;
   uint32_t                sched_policy;
   spf_thread_prio_id_t    id;
} static_attr_lookup_t;

/* ----------------------------------------------------------------------------
 * Global Declarations/Definitions
 * ------------------------------------------------------------------------- */
// 2 columns in the lookup - RT(RealTime) and IRT(Interrupt Triggered RT).
// RT Prio for a given duration/static_req_id is IRT prio -1.
/**
 * Controlling priorities of dynamic threads:
 *
 * attr_duration_map_table holds the list of dyn_attr_lookup_t entries,
 * mapping durations to its corresponding thread priorities.
 *
 * Any duration > 30ms will be assigned a uniform
 * priority = SPF_THREAD_PRIO_LOWEST.
 *
 */

dyn_attr_lookup_t attr_duration_map_table[][2] = {
   /*-----------------RT Lookup-------------------------------------------------||-----------IRT Lookup------------------*/
   {{ SPF_THREAD_CRITICAL_PRIO - 1,  AFFINITY_DEFAULT, SCHED_DEFAULT, 100 },   {SPF_THREAD_CRITICAL_PRIO,       AFFINITY_DEFAULT, SCHED_DEFAULT, 100}},
   {{ SPF_THREAD_HIGH_PRIO     - 1,  AFFINITY_DEFAULT, SCHED_DEFAULT, 500 },   {SPF_THREAD_HIGH_PRIO      - 0,  AFFINITY_DEFAULT, SCHED_DEFAULT, 500}},
   {{ SPF_THREAD_HIGH_PRIO     - 3,  AFFINITY_DEFAULT, SCHED_DEFAULT, 1000},   {SPF_THREAD_HIGH_PRIO      - 2,  AFFINITY_DEFAULT, SCHED_DEFAULT, 1000}},
   {{ SPF_THREAD_MEDIUM_PRIO   - 3,  AFFINITY_DEFAULT, SCHED_DEFAULT, 2000},   {SPF_THREAD_MEDIUM_PRIO    - 2,  AFFINITY_DEFAULT, SCHED_DEFAULT, 2000}},
   {{ SPF_THREAD_MEDIUM_PRIO   - 7,  AFFINITY_DEFAULT, SCHED_DEFAULT, 3000},   {SPF_THREAD_MEDIUM_PRIO    - 6,  AFFINITY_DEFAULT, SCHED_DEFAULT, 3000}},
   {{ SPF_THREAD_MEDIUM_PRIO   - 9,  AFFINITY_DEFAULT, SCHED_DEFAULT, 4000},   {SPF_THREAD_MEDIUM_PRIO    - 8,  AFFINITY_DEFAULT, SCHED_DEFAULT, 4000}},
   {{ SPF_THREAD_MEDIUM_PRIO   - 13, AFFINITY_DEFAULT, SCHED_DEFAULT, 5000},   {SPF_THREAD_MEDIUM_PRIO    - 12, AFFINITY_DEFAULT, SCHED_DEFAULT, 5000}},
   {{ SPF_THREAD_LOW_PRIO      - 3,  AFFINITY_DEFAULT, SCHED_DEFAULT, 6000},   {SPF_THREAD_LOW_PRIO       - 2,  AFFINITY_DEFAULT, SCHED_DEFAULT, 6000}},
   {{ SPF_THREAD_LOW_PRIO      - 8,  AFFINITY_DEFAULT, SCHED_DEFAULT, 7000},   {SPF_THREAD_LOW_PRIO       - 7,  AFFINITY_DEFAULT, SCHED_DEFAULT, 7000}},
   {{ SPF_THREAD_LOW_PRIO      - 12, AFFINITY_DEFAULT, SCHED_DEFAULT, 8000},   {SPF_THREAD_LOW_PRIO       - 11, AFFINITY_DEFAULT, SCHED_DEFAULT, 8000}},
   {{ SPF_THREAD_LOW_PRIO      - 15, AFFINITY_DEFAULT, SCHED_DEFAULT, 9000},   {SPF_THREAD_LOW_PRIO       - 14, AFFINITY_DEFAULT, SCHED_DEFAULT, 9000}},
   {{ SPF_THREAD_LOW_PRIO      - 19, AFFINITY_DEFAULT, SCHED_DEFAULT, 10000},  {SPF_THREAD_LOW_PRIO       - 18, AFFINITY_DEFAULT, SCHED_DEFAULT, 10000}},
   {{ SPF_THREAD_VERY_LOW_PRIO - 1,  AFFINITY_DEFAULT, SCHED_DEFAULT, 12000},  {SPF_THREAD_VERY_LOW_PRIO,       AFFINITY_DEFAULT, SCHED_DEFAULT, 12000}},
   {{ SPF_THREAD_VERY_LOW_PRIO - 3,  AFFINITY_DEFAULT, SCHED_DEFAULT, 14000},  {SPF_THREAD_VERY_LOW_PRIO  - 2,  AFFINITY_DEFAULT, SCHED_DEFAULT, 14000}},
   {{ SPF_THREAD_VERY_LOW_PRIO - 5,  AFFINITY_DEFAULT, SCHED_DEFAULT, 16000},  {SPF_THREAD_VERY_LOW_PRIO  - 4,  AFFINITY_DEFAULT, SCHED_DEFAULT, 16000}},
   {{ SPF_THREAD_VERY_LOW_PRIO - 7,  AFFINITY_DEFAULT, SCHED_DEFAULT, 18000},  {SPF_THREAD_VERY_LOW_PRIO  - 6,  AFFINITY_DEFAULT, SCHED_DEFAULT, 18000}},
   {{ SPF_THREAD_VERY_LOW_PRIO - 9,  AFFINITY_DEFAULT, SCHED_DEFAULT, 20000},  {SPF_THREAD_VERY_LOW_PRIO  - 8,  AFFINITY_DEFAULT, SCHED_DEFAULT, 20000}},
   {{ SPF_THREAD_VERY_LOW_PRIO - 11, AFFINITY_DEFAULT, SCHED_DEFAULT, 22000},  {SPF_THREAD_VERY_LOW_PRIO  - 10, AFFINITY_DEFAULT, SCHED_DEFAULT, 22000}},
   {{ SPF_THREAD_VERY_LOW_PRIO - 13, AFFINITY_DEFAULT, SCHED_DEFAULT, 24000},  {SPF_THREAD_VERY_LOW_PRIO  - 12, AFFINITY_DEFAULT, SCHED_DEFAULT, 24000}},
   {{ SPF_THREAD_VERY_LOW_PRIO - 15, AFFINITY_DEFAULT, SCHED_DEFAULT, 26000},  {SPF_THREAD_VERY_LOW_PRIO  - 14, AFFINITY_DEFAULT, SCHED_DEFAULT, 26000}},
   {{ SPF_THREAD_VERY_LOW_PRIO - 17, AFFINITY_DEFAULT, SCHED_DEFAULT, 28000},  {SPF_THREAD_VERY_LOW_PRIO  - 16, AFFINITY_DEFAULT, SCHED_DEFAULT, 28000}},
   {{ SPF_THREAD_VERY_LOW_PRIO - 19, AFFINITY_DEFAULT, SCHED_DEFAULT, 30000},  {SPF_THREAD_VERY_LOW_PRIO  - 18, AFFINITY_DEFAULT, SCHED_DEFAULT, 30000}},
   {{ SPF_THREAD_PRIO_LOWEST,  AFFINITY_DEFAULT, SCHED_DEFAULT, 0xFFFFFFFF },  {SPF_THREAD_PRIO_LOWEST    + 1, AFFINITY_DEFAULT, SCHED_DEFAULT,  0xFFFFFFFF}}
};

uint32_t     attr_duration_map_table_array_size = SIZE_OF_ARRAY(attr_duration_map_table);

/**
 * Controlling priorities of static threads:
 *
 * attr_id_map_table holds the list of static_attr_lookup_t entries,
 * mapping static req IDs to its corresponding thread priorities.
 */

static_attr_lookup_t attr_id_map_table[][2] = {
   {{ SPF_IST_PRIO             ,  AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_IST_ID},         {SPF_IST_PRIO,                   AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_IST_ID}},
   {{ SPF_THREAD_CRITICAL_PRIO ,  AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_VOICE_TIMER_ID}, {SPF_THREAD_CRITICAL_PRIO - 1,   AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_VOICE_TIMER_ID}},
   {{ SPF_THREAD_MEDIUM_PRIO   ,  AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_PM_SERVER_ID},   {SPF_THREAD_MEDIUM_PRIO   - 4  , AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_PM_SERVER_ID}},
   {{ SPF_THREAD_MEDIUM_PRIO   ,  AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_ASPS_ID},        {SPF_THREAD_MEDIUM_PRIO   - 12 , AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_ASPS_ID}},
   {{ SPF_THREAD_MEDIUM_PRIO   ,  AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_DLS_ID},         {SPF_THREAD_MEDIUM_PRIO   - 13 , AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_DLS_ID}},
   {{ SPF_THREAD_MEDIUM_PRIO   ,  AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_APM_ID},         {SPF_THREAD_MEDIUM_PRIO   - 14 , AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_APM_ID}},
   {{ SPF_THREAD_MEDIUM_PRIO   ,  AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_PRM_ID},         {SPF_THREAD_MEDIUM_PRIO   - 16 , AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_PRM_ID}},
   {{ SPF_THREAD_MEDIUM_PRIO   ,  AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_VCPM_ID},        {SPF_THREAD_MEDIUM_PRIO   - 18 , AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_VCPM_ID}},
   {{ SPF_THREAD_PRIO_LOWEST   ,  AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_AMDB_ID},        {SPF_THREAD_PRIO_LOWEST   + 1  , AFFINITY_DEFAULT, SCHED_DEFAULT, SPF_THREAD_STAT_AMDB_ID}}
   //add static lookups as and when a new use case is implemented/needed
};

uint32_t       attr_id_map_table_array_size = SIZE_OF_ARRAY(attr_id_map_table);

/*
 * Calculates the thread priority of a thread given a
 * duration in microseconds.
 *
 * @param[in]  duration_us frame duration in microseconds
 *
 * @return
 * Thread priority value
 */
void spf_thread_determine_dyn_attr(uint32_t duration_us, bool_t is_interrupt_trig, posal_thread_prio_t *thread_prio_ptr, uint32_t *sched_policy_ptr, uint32_t *affinity_mask_ptr)
{
   POSAL_ASSERT(0 != duration_us);
   uint32_t            i          = 0;
   uint32_t            j          = (is_interrupt_trig == TRUE) ? 1 : 0;
   dyn_attr_lookup_t   attr       = {SPF_THREAD_PRIO_LOWEST, AFFINITY_DEFAULT, SCHED_DEFAULT};
   dyn_attr_lookup_t  *attr_ptr   = &attr;
   uint32_t            array_size = attr_duration_map_table_array_size;

   for (i = 0; i < array_size; i++)
   {
      if (duration_us <= attr_duration_map_table[i][j].frame_duration_us)
      {
         attr_ptr = &attr_duration_map_table[i][j];
         break;
      }
   }
   *thread_prio_ptr = attr_ptr->thread_prio;
   *sched_policy_ptr = attr_ptr->sched_policy;
   *affinity_mask_ptr = attr_ptr->affinity_mask;
}

/*
 * Calculates the thread priority of a thread given the static thread req ID
 *
 * @param[in]  static thread req ID
 *
 * @return
 * Thread priority value
 */
void spf_thread_determine_static_attr(spf_thread_prio_id_t req_id, bool_t is_interrupt_trig, posal_thread_prio_t *thread_prio_ptr, uint32_t *sched_policy_ptr, uint32_t *affinity_mask_ptr)
{
   uint32_t            i          = 0;
   uint32_t            j          = (is_interrupt_trig == TRUE) ? 1 : 0;
   static_attr_lookup_t   attr    = {SPF_THREAD_PRIO_LOWEST, AFFINITY_DEFAULT, SCHED_DEFAULT};
   static_attr_lookup_t  *attr_ptr= &attr;
   uint32_t            array_size = attr_id_map_table_array_size;

   for (i = 0; i < array_size; i++)
   {
      if (req_id == attr_id_map_table[i][j].id)
      {
         attr_ptr = &attr_id_map_table[i][j];
         break;
      }
   }

   *thread_prio_ptr = attr_ptr->thread_prio;
   *sched_policy_ptr = attr_ptr->sched_policy;
   *affinity_mask_ptr = attr_ptr->affinity_mask;
   return ;
}
