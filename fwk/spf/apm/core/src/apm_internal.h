#ifndef _APM_INTERNAL_H_
#define _APM_INTERNAL_H_

/**
 * \file apm_internal.h
 *
 * \brief
 *     This file contains private declarations of the APM Module
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "apm_i.h"
#include "apm_msg_utils.h"
#include "apm_ext_cmn.h"
#include "posal_power_mgr.h"

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/****************************************************************************
 * Macro Declarations
 ****************************************************************************/

/* clang-format off */

#define APM_MAX_CMD_Q_ELEMENTS   16

/** Choose higher number of rspQ elements for SIM to avoid
 *  memleaks */
#ifdef SIM
#define APM_MAX_RSP_Q_ELEMENTS   128
#else
#define APM_MAX_RSP_Q_ELEMENTS   32
#endif

#define APM_DONT_CARE_MASK      (0)
#define APM_KILL_SIG_MASK      (1 << 0)
#define APM_CMD_Q_MASK         (1 << 1)
#define APM_RSP_Q_MASK         (1 << 2)

#define IS_ALIGN_8_BYTE(a) (!((a) & (uint32_t)0x7))

#define IS_ALIGN_4_BYTE(a) (!((a) & (uint32_t)0x3))

#define ALIGN_8_BYTES(a)   ((a + 7) & (0xFFFFFFF8))

#define APM_PM_DELAY (40) /* Delay in milliseconds */

#define APM_LOG_ID         (0xAB400000)

enum apm_msg_type
{
   APM_MSG_TYPE_GPR = 0,
   APM_MSG_TYPE_GK = 1
};

enum apm_proc_q
{
   APM_CMD_Q_IDX = 0,
   APM_RSP_Q_IDX = 1,
   APM_MAX_NUM_PROC_Q
};

typedef struct apm_q_info apm_q_info_t;

struct apm_q_info
{
   char     q_name[POSAL_DEFAULT_NAME_LEN];
  /**< Array to hold queue name */

   uint32_t num_q_elem;
  /**< Number of queue elemetns */

   uint32_t q_sig_mask;
  /**< Queue signal mask */
};

/****************************************************************************
 * Structure Declarations
 ****************************************************************************/

/** APM PM Information Struct */
typedef struct apm_pm_info_t
{
  posal_pm_handle_t       pm_handle_ptr;
  /**< Pointer to PM handle */

  posal_pm_register_t     register_info;
  /**< PM register information */

   uint32_t               pm_vote_count;
   /**< Reference count for pm votes */
} apm_pm_info_t;

/** APM Module Struct */
struct apm_t
{
   spf_handle_t             handle;
   /**< APM thread handle */

   spf_cmd_handle_t         cmd_handle;
   /**< APM thread command handle */

   posal_channel_t        channel_ptr;
   /**< Mask for Q's owned by this obj */

   uint32_t                curr_wait_mask;
   /**< Channel mask or signals to act */

   uint32_t                channel_status;
   /**< Current signal received */

   posal_signal_t          kill_signal_ptr;
   /**< Signal to destroy APM module thread */

   posal_signal_t          gp_signal_ptr;
   /**< General purpose signal ptr */

   posal_queue_t           *q_list_ptr[APM_MAX_NUM_PROC_Q];
   /**< List of thread queue pointer */

   uint32_t                memory_map_client;
   /**< Memory map client */

   apm_graph_info_t        graph_info;
   /**< Graph info structure */

   apm_cmd_ctrl_t          *curr_cmd_ctrl_ptr;
   /**< Slot index of current command
        being processed */

   uint32_t                active_cmd_mask;
   /**< Bit mask for active commands under process */

   apm_cmd_ctrl_t          cmd_ctrl_list[APM_NUM_MAX_PARALLEL_CMD];
   /**< List of commands under process */

   apm_deferred_cmd_list_t def_cmd_list;
   /**< Commands for which the processing
        is deferred until all other commands
        have finished processing */

   apm_pm_info_t           pm_info;
   /**< Power manager information for apm */

   apm_ext_utils_t         ext_utils;
   /**< Vtable for extension utilities */

   uint32_t                gp_counter;
   /**< Free running general purpose counter.
        Used for assigning unique ID's to various
        objects managed by APM, e.g. data path ID */
};

/* clang-format off */

/****************************************************************************
 * Static Inline Function Definitions
 ****************************************************************************/


static inline uint32_t apm_get_next_uid(apm_t *apm_info_ptr)
{
   /** Counter wrap around is acceptable. Start counter from 1 */
   return (++apm_info_ptr->gp_counter);
}

static inline uint32_t apm_get_bit_index_from_channel_status(uint32_t channel_status)
{
   /** 1 bit is reserved for kill signal, hence subtracting from
    *  30 to get the postion of first "1" from right */
   return (30 - s32_cl0_s32(channel_status));

}

static inline void apm_clear_bit_index_in_channel_status(uint32_t *channel_status_ptr, uint32_t bit_pos)
{
   /** 1 bit is reserved for kill signal, hence adding 1 to the
    *  bit position for clearing the bit in the channel mask */
   *channel_status_ptr &= ~(1 << (bit_pos + 1));

}

#ifdef __cplusplus
}
#endif /*__cplusplus*/
#endif /* _APM_INTERAL_H_ */
