/*========================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
 ====================================================================== */

#ifndef _CORE_DRV_H_
#define _CORE_DRV_H_

/* =======================================================================
                     INCLUDE FILES FOR MODULE
========================================================================== */
#include "ar_defs.h"
#include "ar_error_codes.h"

/*==========================================================================
  Structure definitions
  ========================================================================== */

/*==========================================================================
  Function declarations
========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

ar_result_t core_drv_pre_fwk_init(void);

ar_result_t core_drv_post_fwk_deinit(void);

// To initialize core_drv post fwk init dependencies like init ASPS.
ar_result_t core_drv_post_fwk_init();

// To deinitialize core_drv pre fwk deinit dependencies like deinit ASPS.
ar_result_t core_drv_pre_fwk_deinit();

// To call asps_reset() from APM.
ar_result_t core_drv_reset();

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /* _CORE_DRV_H_ */