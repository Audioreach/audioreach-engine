#ifndef AR_OSAL_SYS_ID_H
#define AR_OSAL_SYS_ID_H

/**
 * \file ar_osal_sys_id.h
 * \brief
 *        Defines Supported Sub-System IDs.
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/


/**
* Invalid sub system
*/
#define AR_SUB_SYS_ID_INVALID         0
/**
* Used for MODEM DSP sub system
 */
#define AR_MODEM_DSP                 1
/**
* Used for ADSP sub system
*/
#define AR_AUDIO_DSP                 2
/**
* Used for AP sub system
*/
#define AR_APSS                      3
/**
* Used for SENSOR DSP sub system
*/
#define AR_SENSOR_DSP                4
/**
* Used for COMPUTE DSP sub system
*/
#define AR_COMPUTE_DSP               5
/**
* Used for Companion chip DSP (CC_DSP) sub system
*/
#define AR_CC_DSP                    6
/**
* Used for APSS2 sub system
*/
#define AR_APSS2                     0xC

/**
* First sub system ID
*/
#define AR_SUB_SYS_ID_FIRST    AR_MODEM_DSP
/**
* Last sub system ID
*/
#define AR_SUB_SYS_ID_LAST     AR_APSS2



#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif //AR_OSAL_SYS_ID_H
