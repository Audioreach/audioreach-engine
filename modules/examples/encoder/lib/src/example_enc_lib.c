/**
 * \file example_encoder_lib.c
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

/*------------------------------------------------------------------------
 Includes
 * -----------------------------------------------------------------------*/
#include "example_enc_lib.h"

/*------------------------------------------------------------------------
 Functions
 * -----------------------------------------------------------------------*/
/*=============================================================================
FUNCTION      void example_enc_init
DESCRIPTION   Initialize the library with the new lib variables
===============================================================================*/

void example_enc_init(uint32_t abc, /* abc config */
                      uint32_t xyz)
{
}

/*=============================================================================
FUNCTION      void example_enc_deinit
DESCRIPTION   Set the lib variables to 0 and fre any lib structures that
              may have been allocated
===============================================================================*/

void example_enc_deinit()
{
}

/*=============================================================================
FUNCTION      void example_enc_process
DESCRIPTION   Library process function
===============================================================================*/
/* As an example this encoder lib just picks every fourth sample*/
void example_enc_process(int8_t *outptr, /* Pointer to output */
		                 int8_t *inptr, /* Pointer to input */
						 uint32_t num_samples)
{
   /* copy every fourth sample */
   while (num_samples--)
   {
	  *outptr++ = (*inptr);
	  inptr = inptr+4;
   }
}
