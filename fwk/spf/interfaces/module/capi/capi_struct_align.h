/**
 * \file capi_struct_align.h
 * \brief
 *  This file defines the alignment attributes for different compilers to use
 *  with CAPI structures.
 *  This file is initially created to support aligned structures for 64-bit
 *  system[Windows].
 *  Example case: In gen_topo_capi.c file gen_topo_module_intf_extn_list_t
 *  struct is created with following two structs nested in it.
 *  gen_topo_module_intf_extn_list_t - top-level struct
 *      capi_interface_extns_list_t - 1st nested struct
 *          uint32_t num_extensions
 *      capi_interface_extn_desc_t  - 2nd nested struct
 *          uint32_t it
 *          bool_t is_supported
 *          capi_buf_t capcabilities
 *              int8_t *data_ptr
 *              uint32_t actual_data_len
 *              uint32_t max_data_len
 *  As the capi_buf_t struct has a pointer variable [8-byte in Windows] the
 *  top-level struct is expected to be 8 byte alignment. Due to this there is
 *  a padding inserted after the first struct, which is after num_extensions
 *  variabble. If the nested struct is not 8 byte aligned then the module CAPI
 *  interface will read an incorrect address to fetch the struct elements and
 *  then it crashes.
 *  To avoid this run-time crash due to the struct padding, it is recommented
 *  to use this capi_struct_align.h file to align the structs
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* ----------------------------------------------------------------------------
 * Include Files
 * ------------------------------------------------------------------------- */
 #include <stdint.h>

/* ----------------------------------------------------------------------------
 * Global Definitions
 * ------------------------------------------------------------------------- */
#if UINTPTR_MAX == 0xFFFF
    //do nothing
#elif UINTPTR_MAX == 0xFFFFFFFF
    //do nothing
#elif UINTPTR_MAX == 0xFFFFFFFFFFFFFFFFu
    //align to 8 bytes
    __attribute__((aligned(8)))
#else
    #error Max pointer value is unknown
#endif
