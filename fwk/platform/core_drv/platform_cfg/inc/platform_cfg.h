/*========================================================================
   This file contains HWD device configuration functions

  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
 ====================================================================== */

#ifndef _PLATFORM_CFG_H_
#define _PLATFORM_CFG_H_

/* =======================================================================
                     INCLUDE FILES FOR MODULE
========================================================================== */
#include <stdlib.h>
#include "posal_types.h"

typedef enum
{
   ROOT_THREAD_STACK_PROP,
} platform_prop_t;

/*==========================================================================
  Structure definitions
  ========================================================================== */

typedef struct
{
   uint32 module_id;
   uint32 required_root_thread_stack_size_in_cntr;
} mid_stack_pair_info_t;

mid_stack_pair_info_t *get_platform_prop_info(platform_prop_t prop, uint32_t *elem_num);

#endif /* _PLATFORM_CFG_H_ */
