#ifndef CAPI_INTF_EXTN_STM_TS
#define CAPI_INTF_EXTN_STM_TS

/**
 *   \file capi_intf_extn_stm_ts.h
 *   \brief
 *        intf_extns related to updating the latest timestamp.
 *
 *    This file defines interface extensions that would allow modules to get the latest 
 *    signal triggered timestamp value from framework.
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

/*------------------------------------------------------------------------------
 * Include Files
 *----------------------------------------------------------------------------*/
#include "capi_types.h"

/** @weakgroup weakf_capi_chapter_stm_ts
The STM TS (Signal Triggered Module Timestamp) interface extension (#INTF_EXTN_STM_TS)
allows modules to get the latest signal triggered timestamp value.

Signal Triggered Container gets the handle to query the latest latched signal trigger or interrupt timestamp 
from Signal-Triggered Module under #FWK_EXTN_PARAM_ID_LATEST_TRIGGER_TIMESTAMP_PTR. Framework shares a timestamp_ptr 
with modules supporting this interface extension and ensures that the timestamp value is updated before invoking 
topo-process for each signal-trigger.

Note: If a module is placed in a container that does not have any signal triggered module, that module will not receive any timestamp ptr.

@latexonly

@endlatexonly
*/

/** @addtogroup capi_if_stm_ts_op
*/

/** @addtogroup capi_if_stm_ts_op
@{ */

/** Unique identifier of the Signal Triggered Module Timestamp interface extension. */
#define INTF_EXTN_STM_TS 0x0A001BAF

/** ID of the parameter the framework uses to share the latest signal-trigger timestamp value
 * to the module.
*/
#define INTF_EXTN_PARAM_ID_STM_TS 0x0A001BB0

typedef struct intf_extn_param_id_stm_ts_t intf_extn_param_id_stm_ts_t;
/** Structure used in #INTF_EXTN_PARAM_ID_STM_TS */

typedef struct stm_latest_trigger_ts_t stm_latest_trigger_ts_t;

#include "spf_begin_pragma.h"

struct intf_extn_param_id_stm_ts_t
{
	stm_latest_trigger_ts_t *ts_ptr;
   /**< Pointer to store the address pointing to the latest signal trigger timestamp. */
};

#include "spf_end_pragma.h"
;
/** @} */ /* end_weakgroup weak_intf_extn_stm_ts_t */

/** @} */ /* end_addtogroup capi_if_ext_stm_ts */

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* CAPI_INTF_EXTN_STM_TS_H*/
