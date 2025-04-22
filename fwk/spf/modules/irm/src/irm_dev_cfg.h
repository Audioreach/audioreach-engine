/**
@file irm_dev_cfg.h

@brief IRM platform specific configuration.

================================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
==============================================================================*/
// clang-format off
// clang-format on

#pragma once

// In bytes
#define IRM_THREAD_STACK_SIZE 2048

#define IRM_NUM_BUF_POOL_ARRAYS 40
#define IRM_NODES_PER_ARRAY 32
#define IRM_MAX_SYS_Q_ELEMENTS 8
#define IRM_PREALLOC_SYS_Q_ELEMENTS 3

#define IRM_MAX_ALLOWED_STATIC_SERVICES 16

#define IRM_NUM_BLOCKS 6

// Magic number of 2 comes from 1 special instance from processor metrics and 1 special instance for pool metrics
#define IRM_MAX_DYNAMIC_INSTANCES ((IRM_NODES_PER_ARRAY * IRM_NUM_BUF_POOL_ARRAYS) - (IRM_NUM_BLOCKS + IRM_MAX_ALLOWED_STATIC_SERVICES + 2))

#define IRM_MAX_NUM_CONTAINERS_SUPPORTED (IRM_MAX_DYNAMIC_INSTANCES >> 2)
#define IRM_MAX_NUM_MODULES_SUPPORTED (IRM_MAX_DYNAMIC_INSTANCES - IRM_MAX_NUM_CONTAINERS_SUPPORTED)