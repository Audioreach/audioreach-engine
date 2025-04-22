#ifndef _POSAL_TGT_UTIL_H_
#define _POSAL_TGT_UTIL_H_
/**
 * \file posal_tgt_util.h
 * \brief
 *  	 This file contains few macros related to memory
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


/**
 * max number of memory map clients
 */
#define POSAL_MEMORY_MAP_MAX_CLIENTS  8

/**
 * maximum number of heaps (determined by number of bits used for actual heap)
 */
#define POSAL_HEAP_MGR_MAX_NUM_HEAPS 1

/**
 * Controls buffer pool reserved for queue elements.
 * Total memory = memory for pool instance + first array size.
 *
 *

 * memory for pool instance = sizeof(posal_bufpool_pool_t) + (sizeof(posal_bufpool_nodes_t) * num_arrays) +
 * first array size = (sizeof(posal_bufpool_node_header_t) + align_padding_size + node_size) * REGULAR_QUEUE_BUF_POOL_NODES_PER_ARR=32;
 *
 * For below queue pool node_size=sizeof(posal_queue_element_list_t) =16, num_arrays = 1024
 * [32 + 8 * 1024] + [(4+0+16)*32]
 * */
#define REGULAR_QUEUE_BUF_POOL_NUM_ARRAYS 1024
#define REGULAR_QUEUE_BUF_POOL_NODES_PER_ARR 32

/**
 * Controls the buffer pool reserved for list elements
 * Total memory depends on sizeof(spf_list_node_t)=12
 */
#define REGULAR_LIST_BUF_POOL_NUM_ARRAYS 1024
#define REGULAR_LIST_BUF_POOL_NODES_PER_ARR 32

/**
 * Similar to REGULAR_QUEUE_BUF_POOL_NUM_ARRAYS, but for LPI heap.
 * [32 + 8 * 64] + [(4+0+16)*32] = 544 + 640.
 */
#define LPI_QUEUE_BUF_POOL_NUM_ARRAYS 64
#define LPI_QUEUE_BUF_POOL_NODES_PER_ARR 32

/**
 * Similar to REGULAR_BUF_POOL_NUM_ARRAYS but for LPI heap.
 * [32 + 8 * 64] + [(4+0+8)*32] + (4+0+12)*32 = 544 + 512
 */
#define LPI_LIST_BUF_POOL_NUM_ARRAYS 64
#define LPI_LIST_BUF_POOL_NODES_PER_ARR 32

/**
 * size of each node in the SPF LPI pool (lpi_pools_info: used for Metadata, Control port etc)
 * Controls the buffer pool (lpi_pools_info) reserved for general elements of size LPI_GENERAL_POOL_NODE_SIZE
 * Total memory = num_arrays * LPI_GENERAL_BUF_POOL_NODES_PER_ARR (32) * LPI_GENERAL_POOL_NODE_SIZE + memory for pool instance.
 */
#define LPI_GENERAL_BUF_POOL_NODE_SIZE 128
#define LPI_GENERAL_BUF_POOL_NODES_PER_ARR 8
#define LPI_GENERAL_BUF_POOL_NUM_ARRAYS 4


/**
 * size of each node in the sensors SPF LPI pool
 * Controls the sensor LPI buffer pool (lpi_pools_info) reserved for general elements of size LPI_SNS_BUF_POOL_NODE_SIZE
 * Total memory = num_arrays * LPI_SNS_BUF_POOL_NODES_PER_ARRAY (4) * LPI_SNS_BUF_POOL_NODE_SIZE + memory for pool instance.
 */
#define LPI_SNS_BUF_POOL_NODE_SIZE 512
#define LPI_SNS_BUF_POOL_NODES_PER_ARRAY 4
#define LPI_SNS_BUF_POOL_NUM_ARRAYS 1
/*** APM **/

/*** AMDB **/
/**
 * Number of AMDB threads created to dynamically load the modules.
 * Each thread will typically wait for the apps to read then do sig verification and linking. Since the only opportunity
 * for a thread to wait is while waiting for the apps, which is in the beginning, so there is no point in having more threads.
 */
#define AMDB_MAX_THREADS 2 // POSAL_MAX_HW_THREADS;

/*** Container Utils **/

/** Generic Container **/

/*** Generic Topology **/



#endif /* #ifndef _POSAL_TGT_UTIL_H_ */

