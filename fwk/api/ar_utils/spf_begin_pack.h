/**
 * \file spf_begin_pack.h
 * \brief 
 *  	 This file defines pack attributes for different compilers to be used to pack spf API data structures
 * 
 * \copyright
 *    Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *    SPDX-License-Identifier: BSD-3-Clause-Clear
 */

 
#if defined( __qdsp6__ )
/* No packing atrributes for Q6 compiler; all structs manually packed */
#elif defined( __GNUC__ )
#elif defined( __arm__ )
  __packed
#elif defined( _MSC_VER )
  #pragma warning( disable:4103 )  /* Another header changing "pack". */
  #pragma pack( push, 1 )
#elif defined( __H2XML__)
#else
  #error "Unsupported compiler."
#endif /* __GNUC__ */
