#ifndef AR_IDS_H_
#define AR_IDS_H_
/**
  \file ar_ids.h
  \brief 
       GUID interpretation
  
  \copyright
     Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
     SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_guids.h"

/** @addtogroup spf_utils_guids
@{ */
/** @name Instance IDs
@{ */

/** Instance ID is invalid. A container, sub-graph, or module instance cannot
    be assigned an invalid instance ID.
 */
#define AR_INVALID_INSTANCE_ID                           0

/** Start of the range of SPF static instance IDs.

    The range is from 1 through 0x2000. For example, 1 is
    #APM_MODULE_INSTANCE_ID. The range is applicable for module instance IDs,
    sub-graph IDs, and container IDs.

    All IDs (module instance, container instance, and sub-graph instance) must
    be unique. For example, a number used for a module instance ID cannot be
    used for a container ID.
 */
#define AR_SPF_STATIC_INSTANCE_ID_RANGE_BEGIN             1

/** End of the range of SPF static instance IDs. */
#define AR_SPF_STATIC_INSTANCE_ID_RANGE_END               AR_NON_GUID(0x2000)

/** Start of the range of static instance IDs for a platform driver.

    The range is from 0x2001 through 0x4000. The range is applicable for module
    instance IDs, sub-graph IDs, and container IDs.
 */
#define AR_PLATFORM_DRIVER_STATIC_INSTANCE_ID_RANGE_BEGIN   AR_NON_GUID(0x2001)

/** End of the range of static instance IDs for a platform driver. */
#define AR_PLATFORM_DRIVER_STATIC_INSTANCE_ID_RANGE_END     AR_NON_GUID(0x4000)

/** Range of dynamic instance IDs (starting from 0x4001 through 0xFFFFFFFF).

    The QACT\tm platform uses this range to assign instance IDs. The range is
    applicable for module instance IDs, sub-graph IDs, and container IDs.
 */
#define AR_DYNAMIC_INSTANCE_ID_RANGE_BEGIN                  AR_NON_GUID(0x4001)

/** @} */ /* end_name Instance IDs */


/** @name Port IDs
  A port ID must be unique for a module, but two modules can have the same port
  ID. A port ID cannot be zero.

  The MSB is set for control ports and cleared for data ports. Static control
  ports have two MSBs set. The LSB determines if a port ID is input (0) or
  output (1).
@{ */

/** Invalid or unknown port ID. */
#define AR_PORT_ID_INVALID                      0


/** Mask for the data type of a port. */
#define AR_PORT_DATA_TYPE_MASK                       AR_NON_GUID(0x80000000)

/** Bit shift for the data type of a port. */
#define AR_PORT_DATA_TYPE_SHIFT                      31

/** Port type is Data. */
#define AR_PORT_DATA_TYPE_DATA                       0

/** Port type is Control. */
#define AR_PORT_DATA_TYPE_CONTROL                    1


/** Mask for the control type of a port. */
#define AR_CONTROL_PORT_TYPE_MASK                    AR_NON_GUID(0x40000000)

/** Bit shift for the control type of a port. */
#define AR_CONTROL_PORT_TYPE_SHIFT                   30

/** Control type of a port is Static. */
#define AR_CONTROL_PORT_TYPE_STATIC                  1

/** Control type of a port is Dynamic. */
#define AR_CONTROL_PORT_TYPE_DYNAMIC                 0


/** Mask for the direction type of a port. */
#define AR_PORT_DIR_TYPE_MASK                       AR_NON_GUID(0x00000001)

/** Bit shift for the direction type of a port. */
#define AR_PORT_DIR_TYPE_SHIFT                      0

/** Direction type of a port is Input. */
#define AR_PORT_DIR_TYPE_INPUT                      0

/** Direction type of a port is Output. */
#define AR_PORT_DIR_TYPE_OUTPUT                     1

/** @} */ /* end_name Port ID types */
/** @} */ /* end_addtogroup spf_utils_guids */

#endif /* AR_IDS_H_ */

