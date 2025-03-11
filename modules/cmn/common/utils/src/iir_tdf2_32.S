/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*===========================================================================*]
[* FILE NAME: iir_tdf2_32.S                                                  *]
[* DESCRIPTION:                                                              *]
[*    Assembly Code of functions in iir_tdf2.c                               *]
[* FUNCTION LIST :                                                           *]
[*    iirTDF2_32()                                                           *]
[*===========================================================================*/
/*****************************************************************************/
/* Function:         iirTDF2_32()                                            */
/*---------------------------------------------------------------------------*/
/* Description:      This function calculates the response of a two-pole IIR */
/*                   filter that is implemented in TDF2 form Input and output*/
/*                   are in same q-format and single precision while the     */
/*                   memory and the coefficients are in dual precision       */
/*---------------------------------------------------------------------------*/
/* C Prototype:                                                              */
/*                 void iirTDF2_32(int32 *inp,                               */
/*                                 int32 *out,                               */
/*                                 int32 samples,                            */
/*                                 int32 *numcoefs,                          */
/*                                 int32 *dencoefs,                          */
/*                                 int64 *mem,                               */
/*                                 int16 shiftn,                             */
/*                                 int16 shiftd)                             */
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
/*****************************************************************************/
#include "audio_iir_tdf2_asm.h"

#ifdef QDSP6_ASM_IIRTDF2_32

    .TEXT
    .p2align 4,,15
    .globl iirTDF2_32
    .type   iirTDF2_32, @function
iirTDF2_32:
    { P0 = CMP.GT (R2, #0)          //samples > 0 ? 
      R29 = ADD (R29, #-64)
      R6 = MEMH(R29)                //R6 = shiftn
      R7 = MEMH(R29 + #4) }         //R7 = shiftd

    { P1 = CMP.GT (R7, R6)          //shiftn < shiftd
      R9:8 = COMBINE (#0, #0)       //shiftDiffX : shiftDiffY = 0:0
      MEMD (R29 + #56) = R17:16 
      MEMD (R29 + #48) = R19:18 }

    { IF P1 R9  = SUB (R6, R7)          //if (shiftn < shiftd) R9 = shiftDiffX = shiftn - shiftd;
      IF !P1 R8 = SUB (R7, R6)          //else                 R8 = shiftDiffY = shiftd - shiftn;
      MEMD (R29 + #24) = R25:24
      R24 = #27 
    }
    {
      MEMD (R29 + #16) = R27:26
      IF P1 R25 = SUB (R24, R7)             //(28 - 1 - shiftd) when shiftd > shiftn
      IF !P1 R25 = SUB (R24, R6)            //(28 - 1 - shiftn) when shiftn <= shiftd
      R24 = #1
    }

    { IF P1  R6 = ADD (R7, #-28)       //if (shiftn < shiftd) R6 = shiftY = shiftd-28;
      IF !P1 R6 = ADD (R6, #-28)       //else                 R6 = shiftY = shiftn-28;
      MEMD (SP + #32) = R23:22 
      MEMD (SP + #40) = R21:20 }

    { IF !P0 JUMP .LIIR_TDF2_32_END     //if (samples <=0) return
      R18 = ADD (R9, #-4)              //R18 = shiftDiffX - 4
      R7  = MEMW (R3 + #0)             //R7  = b0 = *numcoefs;
      R10 = MEMW (R3 + #4) }           //R10 = b1 = *(numcoefs + 1);

    { R24 = ASL(R24, R25)              //p5 = (int32)(1 << (-shiftY - 1)) 
      R19 = ADD (R9, #-4)              //R19 = shiftDiffX - 4
      R11 = MEMW (R3 + #8)             //R11 = b2 = *(numcoefs + 2);
      R15:14 = MEMD (R5 + #0) }        //R15:14 = mem[0] = w1

    { R22 = ADD (R8, #-4)              //R22 = shiftDiffY - 4
      R21 = ADD (R8, #-4)              //R21 = shiftDiffY - 4
      R13    = MEMW (R4 + #4)          //R13 = a2 = *(dencoefs + 1);
      R12    = MEMW (R4 + #0) }        //R12 = a1 = *dencoefs;

    { 
      R23    = MEMW (R0 + #0)          //R23 = *inp = x
      R17:16 = MEMD (R5 + #8)          //R17:16 = mem[1] = w2
      P1 = CMP.GT(R6, #-1)             //if shiftY > -1
      R20 = R24                        //store p5 in R3
    }

    { 
      R0     = ADD(R0, #4)             //inp++
      R25:24 = MPY (R7, R23)           //R25:24 = b0TimesX = b0 * *inp ;
      R27:26 = MPY (R10, R23)          //R27:26 = b1TimesX = b1 * *inp ;
      IF P1 R20 = #0                   //p5 = 0 if (shiftY > -1)
    }
      
    { R25:24 = ASL (R25:24, R18)       //R25:24 = b0TimesX = b0TimesX << shiftDiffX - 4
      R27:26 = ASL (R27:26, R19)  }    //R27:26 = b1TimesX = b1TimesX << shiftDiffX - 4 ;

    { R25:24 = ADD (R25:24, R15:14)    //R25:24 = y = b0TimesX + w1Temp
      NOP                         }
      
    {
      R25:24 = ADD(R20, R25:24)        //y = s64_add_s64_s32(y, p5)
    }

    { R25:24 = ASL (R25:24, R6)        //R25:24 = y << shiftY
      R2     = ADD(R2, #-1)       }    //reduce loop count by 1

    { LOOP0 (.LIIRTDF2_32_LOOP, R2)    //for (i = 0; i < samples; ++i)
      R28    = SAT (R25:24)            //R28 = yScaled = sat(y)
      P0 = CMP.EQ(R2, #0)
      MEMW  (R29)  = R5           }

      R3:2   = MPY (R12, R28)          //R3:2   = a1TimesY = a1 * yScaled ;

    .falign
.LIIRTDF2_32_LOOP:

    { R23    = MEMW (R0++#4)           //R23 = *inp++ = x
      R4     = R28                     //R4  = yScaled (for *out)
      R9:8   = MPY (R11, R23)          //R9:8   = b2TimesX = b2 * *inp ;
      R27:26 -= ASL (R3:2, R21)   }    //R27:26 = w1 = Q6_P_sub_PP(b1TimesX,(a1TimesY << (shiftDiffY - 4)));

    { NOP
      R25:24 = MPY (R7, R23)           //R25:24 = b0TimesX = b0 * *inp ;
      MEMW (R1++#4) = R4               //*out = yScaled
      R27:26 = ADD (R27:26, R17:16) }  //R27:26 = w1 = Q6_P_add_PP(w1, w2Temp);

    { R17:16   = ASL (R9:8, R19)       //R17:16   = b2TimesX = b2TimesX << shiftDiffX - 4 ;
      IF (P0) MEMD (R5 + #0) = R27:26  //mem[0] = w1, when samples = 1 
      R27:26 += ASL (R25:24, R18) }    //R27:26 = y = w1Temp + (b0TimesX << shiftDiffX - 4)
      
    {
      R27:26 = ADD(R20, R27:26)        //R27:26 = y = s64_add_s64_s32(p5, y)
      R5:4   = MPY (R13, R28)          //R5:4   = a2TimesY = a2 * yScaled ;
    }

    { 
      R27:26 = ASL (R27:26, R6)       //R27:26 = y << shiftY
      R25:24 = MPY (R10, R23)         //R25:24 = b1TimesX = b1 * *inp ;
    }
    
    { 
      R28    = SAT (R27:26)           //R28 = yScaled = sat(y)    
      R17:16 -= ASL (R5:4, R22)       //R17:16 = w2 = Q6_P_sub_PP( b2TimesX , (a2TimesY << shiftDiffY - 4));      
    }

    { R27:26 = ASL (R25:24, R19)       //R27:26 = b1TimesX = b1TimesX << shiftDiffX - 4 ;
      R3:2   = MPY (R12, R28)  }:endloop0         //R3:2   = a1TimesY = a1 * yScaled ;

    { R4     = R28                     //R4     = yScaled  (for *out)
      R3:2   = ASL (R3:2, R21)         //R3:2   = a1TimesY = a1TimesY << shiftDiffY - 4;
      R9:8   = MPY (R11, R23)    }     //R9:8   = b2TimesX = b2 * *inp ;

    { R15:14 = SUB (R27:26, R3:2)      //R15:14 = w1 = Q6_P_sub_PP(b1TimesX,a1TimesY);
      R5:4   = MPY (R13, R28)          //R5:4  = a2TimesY = a2 * yScaled ;
      IF (!P0) MEMW (R1 + #0) = R4 }   //*out = yScaled

    { R15:14 = ADD (R15:14, R17:16)    //R15:14 = w1 = Q6_P_add_PP(w1, w2Temp);
      R9:8   = ASL (R9:8, R19)   }     //R9:8   = b2TimesX = b2TimesX << shiftDiffX - 4 ;

      R5:4   = ASL (R5:4, R22)         //R5:4  = a2TimesY = a2TimesY << shiftDiffY - 4;

    { R19:18 = SUB (R9:8, R5:4)       //R19:18 = w2 = Q6_P_sub_PP( b2TimesX , a2TimesY);
      R5  = MEMW (R29)           }

    IF (P0) MEMD (R5 + #8) = R17:16         //mem[1] = w2, when samples = 1
     
    { IF (!P0) MEMD (R5 + #8) = R19:18      //mem[1] = w2, samples > 1
      IF (!P0) MEMD (R5 + #0) = R15:14 }    //mem[0] = w1, samples > 1

.LIIR_TDF2_32_END:
    { R27:26 = MEMD (R29 + #16)           //restore registers and return
      R25:24 = MEMD (R29 + #24) }
    { R23:22 = MEMD (SP + #32) 
      R21:20 = MEMD (SP + #40) } 
#ifndef FEATURE_POPQUEUE_CONFLICT
    { R29 = ADD (R29, #64)
      JUMPR R31
      R19:18 = MEMD (SP + #48) 
      R17:16 = MEMD (SP + #56) }
#else
{
 R29 = ADD (R29, #64)
      R19:18 = MEMD (SP + #48) 
      R17:16 = MEMD (SP + #56) 
}
{
      JUMPR R31
}
#endif
    .size iirTDF2_32, .-iirTDF2_32

#endif

