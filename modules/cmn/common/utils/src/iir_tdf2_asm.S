/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if ((defined __hexagon__) || (defined __qdsp6__))
#include "asm_macros.h"

/*===========================================================================*]
[* FILE NAME: iir_tdf2_asm.S                                                 *]
[* DESCRIPTION:                                                              *]
[*    Assembly Code of functions in iir_tdf2.c                               *]
[* FUNCTION LIST :                                                           *]
[*    iirTDF2()                                                              *]
[*===========================================================================*/
    .TEXT
    .p2align 2
    .globl iirTDF2
#define IIR_INTERNAL_SHIFT 3
#define NEG_IIR_INTERNAL_SHIFT -3
/*****************************************************************************/
/* Function:         iir_TDF2()                                              */
/*---------------------------------------------------------------------------*/
/* Description:      This function calculates the response of a two-pole IIR */
/*                   filter that is implemented in TDF2 form Input and output*/
/*                   are in same q-format and single precision while the     */
/*                   memory and the coefficients are in dual precision       */
/*---------------------------------------------------------------------------*/
/* C Prototype:      int iirTDF2(int16 *inp, int16 *out, uint16 samples,     */
/*                            int32 *numcoefs, int32 *dencoefs, int32 *mem,  */
/*                            int16 shiftn, int16 shiftd)                    */
/*---------------------------------------------------------------------------*/
/* Reg Inputs:      R0 : input buffer. (should be 8-byte aligned)            */
/*                  R1 : output buffer.(should be 8-byte aligned)            */
/*                  R2 : no. of samples.                                     */
/*                  R3 : numerator coefficients.                             */
/*                  R4 : denominator coefficients.                           */
/*                  R5 : memory of the filter.                               */
/*---------------------------------------------------------------------------*/
/* Stack Arguments: stack(0) : shiftn.                                       */
/*                  stack(1) = shiftd.                                       */
/*---------------------------------------------------------------------------*/
/* Register Outputs:R0:  block normalization factor                          */
/*---------------------------------------------------------------------------*/
/* Registers Affected:   R0-20                                               */
/*---------------------------------------------------------------------------*/
/* Hardware Loops Usage: LOOP0                                               */
/*---------------------------------------------------------------------------*/
/* Stack Memory Usage (in Bytes):   None                                     */
/*---------------------------------------------------------------------------*/
/* Cycles:    5*(Number of Samples)+5                                        */
/*---------------------------------------------------------------------------*/
/* Notes:                                                                    */
/*      1. Filter memory and cofficients should be 8-byte aligned            */
/*      2. Code will not work for inplace computation and input, output      */
/*         buffers should be different                                       */
/*****************************************************************************/
iirTDF2:
    __saveonentry_32                            //save R16:R23 on stack
    {
        P0 = CMP.GT(R2,#0)			         //P0 = (samples >0 )
		R18= MEMH(R29+#(4+SAVEONENTRY_32_OFFSET))// read shiftd (R18)
        IF !P0.new JUMP:nt .liir_tdf2_END
    }
    {   
	    R6=MEMH(R0++#2)                         //read x0;
        R10=MEMH(R29+#(0+SAVEONENTRY_32_OFFSET))// read shiftn
        R19=#0x1000
        R2=ADD(R2,#-1);                         //samples=samples-1
    }
    {
        R9:8=MEMD(R3++#8)                       // load b1:b0;
        R13:12=MEMD(R4)                         // load a2:a1
        R10=ADD(R10,#NEG_IIR_INTERNAL_SHIFT)    // SR=numqfctshift - 3
        R7=R6                                   // get x into both R6 & R7
    }
    {
        R15:14=MEMD(R5)                         //load w2,w1
        R11=MEMW(R3)                            // load b2
        R17:16=VMPYWEH(R9:8,R7:6):SAT           // R17= b1*x, R16=b0*x (32-bits)
        LOOP0(.liir_tdf2_loop,R2)
    }
    {	    
        P0 = CMP.GT(R2,#0)			            //P0 = (samples >1 )
        IF !P0.new JUMP:nt .liir_tdf2_single_sample
    }
.liir_tdf2_loop:
    {
        R6=MEMH(R0++#2)                         // read next input.
        R14+=ASL(R16,R10)                       // y = b0*x+w1  
        R15+=ASL(R17,R10);                      // temp = b1*x+w2
    }
    {
        R7=R6;
        R20=ADD(R19,R14)                        // R20 = y.
        R17:16=MPY(R14,R12)                     // a1*y
        R23:22=VMPYWEH(R11:10,R7:6):SAT         // R23=b2*x, R22=dummy multiplication.
    }
    {
        R15-=ASL(R17,R18);                       // w1 calculated.(R18:shiftd)
        R20=ASL(R20,#IIR_INTERNAL_SHIFT):SAT;   // R20.H = output.
    }
    {
        R14=R15;                                // R14 = w1
        MEMH(R1++#2)=R20.H;                     // store the output
        R17:16=MPY(R14,R13)                     // a2*y
        R15=ASL(R23,R10);                       // R23= b2*x
    }
    {
        R15-=ASL(R17,R18)                        // R15 = w2 = b2*x-a2*y(R18:shiftd)
        R17:16=VMPYWEH(R9:8,R7:6):SAT           // R17 = b1*x, R16=b0*x
    }:endloop0
.liir_tdf2_single_sample:	
    {
        R14+=ASL(R16,R10)                       // y = b0*x+w1  
        R15+=ASL(R17,R10);                      // temp = b1*x+w2
    }
    {
        R20=ADD(R19,R14)                        // R20 = y.
        R17:16=MPY(R14,R12)                     // a1*y
        R23:22=VMPYWEH(R11:10,R7:6):SAT         // R23=b2*x, R22=dummy multiplication.
    }
    {
        R15-=ASL(R17,R18);                       // w1 calculated.(R18:shiftd)
        R20=ASL(R20,#IIR_INTERNAL_SHIFT):SAT;   // R20.H = output.
    }
    {
        R14=R15;                                // R14 = w1
        MEMH(R1++#2)=R20.H;                     // store the output
        R17:16=MPY(R14,R13)                     // a2*y
        R15=ASL(R23,R10);                       // R23= b2*x
    }
    {
        R15-=ASL(R17,R18)                        // R15 = w2 = b2*x-a2*y(R18:shiftd)
    }	
    { 
        R0=#0
        MEMD(R5)=R15:14                         // store the memory for IIR filter.        
    }
.liir_tdf2_END:	
#ifndef FEATURE_POPQUEUE_CONFLICT
    __restoreonexit_32                          //restore R16:R23 from stack
#else
{ 
  R19:18 = MEMD(SP+#8);    /*restore r18,r19     */
  R21:20 = MEMD(SP+#16);   /*restore r20,r21     */
} 
{ R23:22 = MEMD(SP+#24);   /*restore r22,r23     */
  R17:16 = MEMD(SP++#32);  /*restore r16,r17 & dealloc stack */
}
{
  JUMPR R31;       /*return to caller    */
}
#endif
#endif
