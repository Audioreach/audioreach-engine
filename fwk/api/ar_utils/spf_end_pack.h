/**
 * \file spf_end_pack.h
 * \brief 
 *  	 This file defines pack attributes for different compilers to be used to pack spf API data structures
 * 
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */
 

#if defined( __qdsp6__ )
/* No packing atrributes for Q6 compiler; all structs manually packed */
#elif defined( __XTENSA__)
#elif defined( __GNUC__ )
  __attribute__((packed));
#elif defined( __arm__ )
#elif defined( _MSC_VER )
  #pragma pack( pop )
#elif defined (__H2XML__)
  __attribute__((packed))
#else
  #error "Unsupported compiler."
#endif /* __GNUC__ */

