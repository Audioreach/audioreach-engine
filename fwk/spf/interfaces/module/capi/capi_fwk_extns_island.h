#ifndef _CAPI_FWK_EXTN_ISLAND_H_
#define _CAPI_FWK_EXTN_ISLAND_H_

/**
 *   \file capi_fwk_extns_island.h
 *   \brief
 *        This file contains extension for island handling
 *
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "capi_types.h"

/** @weakgroup weakf_capi_chapter_island
The island framework extension (#FWK_EXTN_ISLAND) provides utilities to exit island from CAPI.

@note1hang This framework extension is deprecated.
*/

/** @addtogroup capi_fw_ext_island
@{ */

/*==============================================================================
   Constants
==============================================================================*/

/** Unique identifier of the framework extension that modules use to exit from
    island.
 */
#define FWK_EXTN_ISLAND 0x0A001057

/** ID of the event a module uses to trigger island exit.
*/
#define FWK_EXTN_EVENT_ID_ISLAND_EXIT 0x0A001058

/** @} */ /* end_group capi_fw_ext_island */

#endif /* _CAPI_FWK_EXTN_ISLAND_H_ */
