/**
 * \file posal_power_mgr.h
 * \brief
 *  	 Lite Wrapper for PM. Mainly to serve profiling. goal is not to hide MMPM/PM details
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_POWER_MGR_H
#define POSAL_POWER_MGR_H

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */

#include "posal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#define PM_SERVER_CLIENT_TOKEN_PREFIX "AUD_"

#define PM_SERVER_CLIENT_TOKEN_LENGTH (4)

#define PM_SERVER_CLIENT_NAME_LENGTH (16)

#define PM_SERVER_CLIENT_NAME_MAX_LENGTH (PM_SERVER_CLIENT_TOKEN_LENGTH + PM_SERVER_CLIENT_NAME_LENGTH)

/* =======================================================================
  Type Declarations
========================================================================== */
typedef void *posal_pm_handle_t;

/* PM modes */
typedef enum posal_pm_mode_t
{
   PM_MODE_DEFAULT = 0,       /**< Non-island, non-suppressible */
   PM_MODE_ISLAND,            /**< Island, suppressible */
   PM_MODE_ISLAND_DUTY_CYCLE, /**< Not an Island container but only BW votes are suppressible (Mainly used in BT A2DP
                                 use case) */
} posal_pm_mode_t;

/**< PM island types */
typedef enum posal_pm_island_type_t
{
   PM_ISLAND_TYPE_DEFAULT = 0, /** < default island type */
   PM_ISLAND_TYPE_LOW_POWER = PM_ISLAND_TYPE_DEFAULT, /**< island type to enter STD island */
   PM_ISLAND_TYPE_LOW_POWER_2  /**< island type to enter LLC island */
} posal_pm_island_type_t;

/**< PM island Vote */
typedef enum posal_pm_island_vote_type_t
{
   PM_ISLAND_VOTE_ENTRY = 0, /**<  Island vote to be casted for island entry state*/
   PM_ISLAND_VOTE_EXIT,      /**<  Island vote to be casted for island exit sate*/
   PM_ISLAND_VOTE_DONT_CARE, /**<  Island vote to be casted for island dont care state*/
} posal_pm_island_vote_type_t;

/*PM CPU LPR ID */
typedef enum posal_pm_cpu_lpr_id_t
{
   PM_LPR_CPU_SS_SLEEP = 0,
   PM_LPR_CPU_MAX
} posal_pm_cpu_lpr_id_t;

/**< PM CPU LPR Vote type */
typedef enum posal_pm_cpu_lpr_vote_type_t
{
   /*Vote type for PM_LPR_CPU_SS_SLEEP*/
   PM_VOTE_FOR_CPU_LPR_SUB_SYSTEM_SLEEP = 0, /* Vote for CPU sub sys sleep to be enabled */
   PM_VOTE_AGAINST_CPU_LPR_SUB_SYSTEM_SLEEP, /* Vote against CPU sub sys sleep to be disabled */
   PM_VOTE_NUM_CPU_LPR                       /* Max Num of CPU LPR vote type allowed*/
} posal_pm_cpu_lpr_vote_type_t;

/**< Register information */
typedef struct posal_pm_register_t
{
   posal_pm_mode_t        mode;        /**< PM mode */
   posal_pm_island_type_t island_type; /**< Island type */
} posal_pm_register_t;

/* MPPS resource struct */
typedef struct posal_pm_mpps_t
{
   bool_t   is_valid;  /* If mpps is voted on */
   uint32_t value;     /* mpps value (millions of packets per second) */
   uint64_t floor_clk; /* floor clock value in Hz */
} posal_pm_mpps_t;

/* Bandwidth resource struct */
typedef struct posal_pm_bw_t
{
   bool_t   is_valid; /* If bw is voted on */
   uint32_t value;    /* Bandwidth value (bytes/sec) */
} posal_pm_bw_t;

/* Sleep latency resource struct */
typedef struct posal_pm_sleep_latency_t
{
   bool_t   is_valid; /* If sleep latency is voted on */
   uint32_t value;    /* Sleep latency value in microseconds */
} posal_pm_sleep_latency_t;

/* Island vote struct */
typedef struct posal_pm_island_vote_t
{
   bool_t                      is_valid;         /* True  : in case of casting island vote */
   posal_pm_island_vote_type_t island_vote_type; /* Island voting type to be casted*/
   posal_pm_island_type_t      island_type;      /* Island type to be voted for */
} posal_pm_island_vote_t;

/* Sub Sys sleep vote struct */
typedef struct posal_pm_cpu_lpr_vote_t
{
   bool_t                       is_valid;          /* True  : in case of casting Sub Sys sleep vote */
   posal_pm_cpu_lpr_id_t        lpr_id;            /* LPR ID for which vote is casted */
   posal_pm_cpu_lpr_vote_type_t cpu_lpr_vote_type; /* CPU LPR voting type to be casted*/
} posal_pm_cpu_lpr_vote_t;

/* Resource information struct */
typedef struct posal_pm_resources_t
{
   posal_pm_mpps_t          mpps;                         /* MPPS information */
   posal_pm_bw_t            bw;                           /* Bandwidth information */
   posal_pm_sleep_latency_t sleep_latency;                /* Sleep latency information */
   posal_pm_island_vote_t   island_vote;                  /* Island voting information */
   posal_pm_cpu_lpr_vote_t  cpu_lpr_vote[PM_LPR_CPU_MAX]; /** CPU LPR voting information */
} posal_pm_resources_t;

/* Clock voting request information */
typedef struct posal_pm_request_info_t
{
   posal_pm_handle_t    pm_handle_ptr;   /* Pointer to PM handles info */
   uint32_t             client_log_id;   /* Container log ID */
   posal_signal_t       wait_signal_ptr; /* Signal to wait on for response */
   posal_pm_resources_t resources;       /* Struct of resources to vote on */
} posal_pm_request_info_t;

/* Clock voting release information */
typedef struct posal_pm_release_info_t
{
   posal_pm_handle_t    pm_handle_ptr;   /* Pointer to PM handles info */
   uint32_t             client_log_id;   /* Container log ID */
   posal_signal_t       wait_signal_ptr; /* Signal to wait on for response */
   uint32_t             delay_ms;        /* Release delay in milliseconds */
   posal_pm_resources_t resources;       /* List of resources to vote on */
} posal_pm_release_info_t;
/* -----------------------------------------------------------------------
 ** Global definitions/forward declarations
 ** ----------------------------------------------------------------------- */

/** @ingroup posal_power_mgr
  Sends request to ADSPPM

  @return
  returns error code.

  @dependencies
  None.
 */
ar_result_t posal_power_mgr_request(posal_pm_request_info_t *request_info_ptr);

/** @ingroup posal_power_mgr
  Sends release to ADSPPM

  @return
  returns error code.

  @dependencies
  None.
 */
ar_result_t posal_power_mgr_release(posal_pm_release_info_t *release_info_ptr);

/** @ingroup posal_power_mgr
  Registers for kpps and bw

  @return
  returns error code.

  @dependencies
  None.
 */
ar_result_t posal_power_mgr_register(posal_pm_register_t register_info,
                                     posal_pm_handle_t * pm_handle_pptr,
                                     posal_signal_t      wait_signal,
                                     uint32_t            log_id);

/** @ingroup posal_power_mgr
  Deregisters with ADSPPM

  @return
  returns error code.

  @dependencies
  None.
 */
ar_result_t posal_power_mgr_deregister(posal_pm_handle_t *pm_handle_pptr, uint32_t log_id);

/**
 * returns true if the client is registered.
 */
bool_t posal_power_mgr_is_registered(posal_pm_handle_t pm_handle_ptr);

/**
 * bumps up the bus and Q6 clocks.
 */
ar_result_t posal_power_mgr_request_max_out(posal_pm_handle_t pm_handle_ptr,
                                            posal_signal_t    wait_signal,
                                            uint32_t          log_id);

/**
 * releases the bus and Q6 clocks.
 */
ar_result_t posal_power_mgr_release_max_out(posal_pm_handle_t pm_handle_ptr, uint32_t log_id, uint32_t delay_ms);

/**
 * initalises structures and mutex (if any)
 */
void posal_power_mgr_init();

/**
 * de-initalises structures and mutex (if any)
 */
void posal_power_mgr_deinit();

/**
* To Send message commands to PM SERVER
*/
ar_result_t posal_power_mgr_send_command(uint32_t msg_opcode, void *payload_ptr , uint32_t payload_size);



#ifdef __cplusplus
}
#endif //__cplusplus

#endif // #ifndef POSAL_POWER_MGR_H
