
/**
 * \file gpr_pack_end.h
 * \brief
 *  	This file contains GPR pack end
 *
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All rights reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */


#if defined( __qdsp6__ )
/* No packing atrributes for Q6 compiler; all structs manually packed */
#elif defined( __GNUC__ )
  __attribute__((packed));
#elif defined( __arm__ )
#elif defined( _MSC_VER )&& !defined(_KERNEL_MODE)
  #pragma pack( pop )
#elif defined (__H2XML__)
  __attribute__((packed))
#elif defined( _KERNEL_MODE)
#else
  #error "Unsupported compiler."
#endif /* __GNUC__ */

