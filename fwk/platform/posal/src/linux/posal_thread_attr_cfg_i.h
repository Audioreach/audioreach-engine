/**
 * @file posal_thread_attr_cfg_i.h
 *
 * @copyright
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#ifndef POSA_THREAD_ATTR_CFH_I_H_
#define POSA_THREAD_ATTR_CFH_I_H_
/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
#include "posal_thread_attr_def.h"

/* ----------------------------------------------------------------------------
 * Global Declarations/Definitions
 * ------------------------------------------------------------------------- */
#define SIZE_OF_ARRAY(a) (sizeof(a) / sizeof((a)[0]))

#define THREAD_PRIORITY_TIME_CRITICAL	 49
#define THREAD_PRIORITY_HIGHEST			 45
#define THREAD_PRIORITY_ABOVE_NORMAL	 35
#define THREAD_PRIORITY_NORMAL			 25
#define THREAD_PRIORITY_BELOW_NORMAL	 15
#define THREAD_PRIORITY_LOWEST			 10
#define THREAD_PRIORITY_IDLE			  1

/* ----------------------------------------------------------------------------
 * Highest and lowest thread priorities.
 * ------------------------------------------------------------------------- */
#define SPF_THREAD_CRITICAL_PRIO 		THREAD_PRIORITY_TIME_CRITICAL
#define SPF_THREAD_HIGH_PRIO     		THREAD_PRIORITY_HIGHEST
#define SPF_THREAD_MEDIUM_PRIO   		THREAD_PRIORITY_ABOVE_NORMAL
#define SPF_THREAD_LOW_PRIO      		THREAD_PRIORITY_NORMAL
#define SPF_THREAD_VERY_LOW_PRIO 		THREAD_PRIORITY_BELOW_NORMAL
#define SPF_THREAD_PRIO_LOWEST   		THREAD_PRIORITY_LOWEST
#define SPF_IST_PRIO             		THREAD_PRIORITY_TIME_CRITICAL

/**
 * Container priority when it's not processing any data (container is not started).
 * Also used for containers situated in non-real-time graphs.
 */
#define SPF_CNTR_FLOOR_THREAD_PRIO      (SPF_THREAD_PRIO_LOWEST + 2)

#endif /* POSA_THREAD_ATTR_CFH_I_H_ */
