/*======================= COPYRIGHT NOTICE ==================================*]
//        Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
//        SPDX-License-Identifier: BSD-3-Clause-Clear                                  *]
[*===========================================================================*/
/*============================================================================*]
[* FILE NAME: latency_delayline_update.S                                              *]
[* DESCRIPTION:                                                               *]
[*    Assembly Code for latency_delayline_update function                             *]
[* FUNCTION LIST :                                                            *]
[*    latency_delayline_update()                                                      *]
[*============================================================================*]
[******************************************************************************/
/* Function:         latency_delayline_update                                         */
/*----------------------------------------------------------------------------*/
/* Description:      Update the buffer in a delayline struct with new         */
/*                   input samples from input buffer.                         */
/*----------------------------------------------------------------------------*/
/* C Prototype:      int16 latency_delayline_update(delaylineStruct *delayline,       */
/*                   int16 *srcBuf, int16 samples)                            */
/*----------------------------------------------------------------------------*/
/* Reg Inputs:       R0  : Pointer to structure           (i)                 */
/*                   R1  : Pointer to source vector       (o)                 */
/*                   R2  : No of Samples                  (i)                 */
/*----------------------------------------------------------------------------*/
/* Stack Arguments:      None                                                 */
/*----------------------------------------------------------------------------*/
/* Register Ouputs:      R0 -> index                                          */
/*----------------------------------------------------------------------------*/
/* Registers Affected:   R0-R10                                               */
/*----------------------------------------------------------------------------*/
/* Hardware Loops Usage: LOOP0                                                */
/*----------------------------------------------------------------------------*/
/* Stack Memory Usage (in Bytes):   None                                      */
/*----------------------------------------------------------------------------*/
/* Thread Cycles:    None                                                     */
/*----------------------------------------------------------------------------*/
/* Notes:                                                                     */
/*      1. To match QSound code, there is no discussion of the gain values,   */
/*         compared to function buffer_delay_mix                              */
/*      2. latency_delayline_update function updates the delay buffer                 */
/*         (circular) from the samples of the linear buffer.As the delay      */
/*         buffer(delayPtr)is half word aligned,vectorization is not effective*/
/*      3. The assembly function takes 2 cycle per sample.                    */
/******************************************************************************/
#if  defined(__qdsp6__) && defined(LATENCY_DELAYLINE_UPDATE_ASM_COMPILATION)
#include "asm_macros.h"

#define OFFSETOF_DELAYBUF    8
#define OFFSETOF_DELAYINDEX  0
#define OFFSETOF_DELAYLENGTH 4

.text
.p2align 2

.globl latency_delayline_update
latency_delayline_update:
    __saveonentry_32                            //save R16:R23 on stack (arch = v3)
    {
        R3 = MEMW(R0 + #OFFSETOF_DELAYLENGTH);  // offset of delayline.delaylength
        R4 = MEMW(R0 + #OFFSETOF_DELAYBUF);     // offset of delayline.delaybuf
        R8  = R1;                               // srcPtr
    }
    {
        R5 = MEMW(R0 + #OFFSETOF_DELAYINDEX);   // offset of delayline.delayindex
        R7 = ADDASL(R4, R3, #1);                // tmptr = delayline->delayBuf + delayline->delayLength;
        P0 = CMP.GT(R2, R3);                    // if(samples > delayline->delayLength)
        R10 = SUB(R2, R3);                      // samples = samples - delayline->delayLength
                                                // for finding srcPtr
    }
    {
        R6 = ADDASL(R4, R5, #1);                // delayptr = delayline->delaybuf + delay_index;
        IF(!P0) JUMP delayline_start ;          // Jump to delayline_loop
        R12 = SUB(R3, R5);                      // temp
        P2 = CMP.GT(R2, #0);                    // while(samples > 0)
    }
    {
        R8 = ADDASL(R8, R10, #1);               // srcPtr = srcBuf + samples - delayline->delayLength;
        R2 = ZXTH(R3);                          // samples = (int16)delayline->delayLength
    }

delayline_start:

    {
        R12 = MIN(R2, R12);                     // Minimum of samples & temp
        IF(!P2) JUMP .llatency_delayline_update_END;    // if not, quit while loop
    }
    {
        R22 = ASR(R12,#1);                      // for loop unrolling
        R23 = AND(R12,#1);                      // for loop unrolling
    }
    {
        LOOP0(delayline_loop_even,R22);         // delayline_loop
    }

delayline_while:                                // forloop, unrolled here

    {
        LOOP1(delayline_loop_odd, R23);         // delayline_loop
        P0 = CMP.EQ(R22, #0);                   // for loop unrolling, odd loop count
        P1 = CMP.EQ(R23, #0);                   // for loop unrolling, odd loop count
    }
    {
        IF(P0) JUMP delayline_loop_odd;         // Jump after loop
        R9  = MEMH(R8++#4);                     // loading from srcPtr
        R10 = MEMH(R8+#2);                      // loading from srcPtr
    }

delayline_loop_even:                            // even forloop

    {
        MEMH(R6++#4) = R9;                      // storing to delayPtr
        R9  = MEMH(R8++#4);                     // loading from srcPtr
    }
    {
        MEMH(R6+#-2) = R10;                     // storing to delayPtr
        R10 = MEMH(R8+#-2);                     // loading from srcPtr
    }:endloop0

delayline_loop_odd:                             // odd loop, loop unrolling

    {
        IF(P1) JUMP exit_delayloop;             // Jump after loop
        R8 = ADD(R8,#-4);                       // compensating for an extra address increment
    }
    {
        MEMH(R6++#2) = R9;                      // storing to delayPtr
        R8 = ADD(R8,#+2);
    }

exit_delayloop:

    {
        P0 = CMP.EQ(R6, R7);                    // if (delayPtr == tmptr)
        IF(P0.new) R6 = R4;                     // delayPtr = delayline->delayBuf
        R2 = SUB(R2,R12);
    }
    {
        R5 = SUB(R6, R4);                       // delayline->index = (delayPtr - delayline->delayBuf)
    }
    {
        R5 = ASR(R5, #1);                       // for adjusting word count
    }
    {
        R19 = SUB(R3,R5);                       // delayLine->delayLength-delayline->index
    }
    {
        R12 = MIN(R2, R19);                      // Minimum of samples & delayLine->delayLength
    }
    {
        R22 = ASR(R12,#1);                      // for loop unrolling
        R23 = AND(R12,#1);                      // for loop unrolling
    }
    {
        P2 = CMP.GT(R2, #0);                    // while(samples > 0)
        LOOP0(delayline_loop_even,R22);         // delayline_loop
    }
    {
        IF(P2) JUMP delayline_while;            // if not, quit while loop
        MEMW(R0 + #OFFSETOF_DELAYINDEX) = R5;   // Updating Index value
    }
.llatency_delayline_update_END:
#ifndef FEATURE_POPQUEUE_CONFLICT
    __restoreonexit_32                          //restore R16:R23 from stack (arch = v3) and return
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

    .size latency_delayline_update, .-latency_delayline_update
#endif
