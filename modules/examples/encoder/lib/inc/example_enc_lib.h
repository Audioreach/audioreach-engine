/**
 * \file example_encoder_lib.h
 *  
 * \brief
 *  
 *     Example Encoder Module
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef _EXAMPLE_ENC_LIB_H
#define _EXAMPLE_ENC_LIB_H

/*------------------------------------------------------------------------
 Includes
 * -----------------------------------------------------------------------*/
#include "ar_defs.h"
#include "posal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/*------------------------------------------------------------------------
 Function declaration
 * -----------------------------------------------------------------------*/
void example_enc_init(uint32_t abc,  /* abc config */
                      uint32_t xyz); /* xyz config */

void example_enc_deinit();

void example_enc_process(int8_t * outptr, /* Pointer to output */
                         int8_t * inptr,  /* Pointer to input*/
                         uint32_t num_samples);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif // _EXAMPLE_ENC_LIB_H
