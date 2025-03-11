/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*===========================================================================*]
[* FILE NAME: audio_panner_asm.S                                             *]
[* DESCRIPTION:                                                              *]
[*    Assembly Code of functions in audio_panner.cpp                         *]
[* FUNCTION LIST :                                                           *]
[*    buffer_scale_mix_asm()                                                 *]
[*===========================================================================*/

#if ((defined __hexagon__) || (defined __qdsp6__))
/*****************************************************************************/
/* Function:         buffer_scale_mix_asm                              */
/*---------------------------------------------------------------------------*/
/* Description:      Apply incremental gain to input, mix (sum) it with      */
/*                   another output buffer and store in output buffer.       */
/*---------------------------------------------------------------------------*/
/* C Prototype:      void buffer_scale_mix_asm(int16 *srcPtr,          */
/*                        int16 *destPtr, int32 currentGainL32Q31,           */
/*                        int32 deltaL32Q31, int16 rampSamples);             */
/*---------------------------------------------------------------------------*/
/* Reg Inputs:       R0 : Pointer to input vector       (i)                  */
/*                   R1 : Pointer to output vector      (i/o)                */
/*                   R2 : Initial gain                  (i)                  */
/*                   R3 : Increment in Gain Value       (i)                  */
/*                   R4 : No of Input samples           (i)                  */
/*---------------------------------------------------------------------------*/
/* Stack Arguments:      None                                                */
/*---------------------------------------------------------------------------*/
/* Register Outputs:                                                         */
/*---------------------------------------------------------------------------*/
/* Registers Affected:   R0-R9                                               */
/*---------------------------------------------------------------------------*/
/* Hardware Loops Usage: LOOP0                                               */
/*---------------------------------------------------------------------------*/
/* Stack Memory Usage (in Bytes):   None                                     */
/*---------------------------------------------------------------------------*/
/* Thread Cycles:    None                                                    */
/*---------------------------------------------------------------------------*/
/* Notes:                                                                    */
/*      1. Presently assembly consumes 2 cycles per sample                   */
/*****************************************************************************/
.p2align 2
.globl buffer_scale_mix_asm
buffer_scale_mix_asm:
    { 
	    R5=MEMH(R0++#2);                        //*srcPtr++
		R6=MEMH(R1);                            //*destPtr
		R4=ADD(R4,#-1);                         //samples = samples - 1
    }
	{ 
        P0 = CMP.GT(R4,#0)			            //P0 = (nsampe > 1 )
        IF !P0.new JUMP:nt .lbuffer_scale_mix_asm_single_sample   	
	    R9:8=MPY(R5.l,R2.h):<<1;                //tmpL16 = ((*srcPtr++) * currentGainL16Q15) << 1 
		R6=ASLH(R6);                            //shift R6 from lower to higher half
	}
	{
		LOOP0(.lbuffer_scale_mix_asm_loop,R4);       
	}	
.lbuffer_scale_mix_asm_loop:
	{
	    R8=ADD(R8,R6):sat;                      //tmpL16 = tmpL16 + *destPtr
		R2=ADD(R2,R3):sat		                //gain increment by deltaL32Q31
	    R5=MEMH(R0++#2);                        //load next sample *srcPtr++
		R6=MEMH(R1+#2);                         //load next sample *(destPtr+2)
	}
	{
	    R6=ASLH(R6);                            //shift R6 from lower to higher half
        R9:8=MPY(R5.l,R2.h):<<1;                //tmpL16 = ((*srcPtr++) * currentGainL16Q15) << 1 
        MEMH(R1++#2) = R8.h;                    //store sample
	}:endloop0
.lbuffer_scale_mix_asm_single_sample:
	{
	    R8=ADD(R8,R6):sat;                      //tmpL16 = tmpL16 + *destPtr
	}
#ifndef FEATURE_POPQUEUE_CONFLICT
	{
        MEMH(R1++#2) = R8.h;                    //store last sample
		JUMPR R31;                              //return 
	}
#else
	{
        MEMH(R1++#2) = R8.h;                    //store last sample
	
}
{
		JUMPR R31;                              //return 
}
#endif
#endif

