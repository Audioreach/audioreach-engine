#ifndef __GPR_IDS_DOMAINS_H__
#define __GPR_IDS_DOMAINS_H__

#include "ar_guids.h"

/**
 * @file  gpr_ids_domains.h
 * @brief This file contains GPR IDs and domains
 *
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


/** @addtogroup gpr_macros
@{ */
/** @name GPR Domain IDs
@xreflabel{hdr:DomainIds}
A domain ID is to a unique ID that identifies a process, a processor, or an
off-target location that houses GPR. The transport mechanism between the
published domains can vary.

Domain IDs are directly used as port indices to access GPR routing arrays.
Hence they have to always have values starting from 0 such as 0,1,2,3 and
not Globally Unique IDs (GUIDS).
@{ */

/** Invalid domain. */
#define GPR_IDS_DOMAIN_ID_INVALID_V   0

/** Modem DSP (mDSP) domain. */
#define GPR_IDS_DOMAIN_ID_MODEM_V     1

/** Audio DSP (aDSP) domain. */
#define GPR_IDS_DOMAIN_ID_ADSP_V      2

/** Applications domain. */
#define GPR_IDS_DOMAIN_ID_APPS_V      3

/** Sensors DSP (sDSP) domain. */
#define GPR_IDS_DOMAIN_ID_SDSP_V      4

/** Compute DSP (cDSP) domain. */
#define GPR_IDS_DOMAIN_ID_CDSP_V      5

/** Companion chip DSP (CC_DSP) domain */
#define GPR_IDS_DOMAIN_ID_CC_DSP_V     6

/** Reserved domain */
#define GPR_IDS_DOMAIN_ID_RESERVED_1     7
#define GPR_IDS_DOMAIN_ID_RESERVED_2     8
#define GPR_IDS_DOMAIN_ID_RESERVED_3     9

/** Generic chip DSP (gDSP0) domain */
#define GPR_IDS_DOMAIN_ID_GDSP0_V     0xA

/** Generic chip DSP (gDSP1) domain */
#define GPR_IDS_DOMAIN_ID_GDSP1_V     0xB

/** SPF-on-ARM HLOS (APPS2) domain */
#define GPR_IDS_DOMAIN_ID_APPS2_V     0xC
 
/** Highest domain ID. @hideinitializer */
#define GPR_PL_MAX_DOMAIN_ID_V AR_NON_GUID( GPR_IDS_DOMAIN_ID_APPS2_V )

/** Total number of domains. @hideinitializer */
#define GPR_PL_NUM_TOTAL_DOMAINS_V ( GPR_PL_MAX_DOMAIN_ID_V + 1 )

/** @} */ /* end_name GPR Domain IDs */
/** @} */ /* end_addtogroup gpr_macros */

#endif /* __GPR_IDS_DOMAINS_H__ */

