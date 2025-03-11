/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*===========================================================================*]
[* FILE NAME: iir_tdf2_16.S                                                  *]
[* DESCRIPTION:                                                              *]
[*    Assembly Code of functions in iir_tdf2.c                               *]
[* FUNCTION LIST :                                                           *]
[*    iirTDF2_16()                                                           *]
[*===========================================================================*/
/*****************************************************************************/
/* Function:         iirTDF2_16()                                            */
/*---------------------------------------------------------------------------*/
/* Description:      This function calculates the response of a two-pole IIR */
/*                   filter that is implemented in TDF2 form Input and output*/
/*                   are in same q-format and single precision while the     */
/*                   memory and the coefficients are in dual precision       */
/*---------------------------------------------------------------------------*/
/* C Prototype:                                                              */
/*                 void iirTDF2_16(int16 *inp,                               */
/*                                 int16 *out,                               */
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
/* Registers Affected:   R0-28                                               */
/*---------------------------------------------------------------------------*/
/* Hardware Loops Usage: LOOP0                                               */
/*---------------------------------------------------------------------------*/
/* Stack Memory Usage (in Bytes):   None                                     */
/*---------------------------------------------------------------------------*/
/* Cycles:    10*(Number of Samples)+12                                      */
/*---------------------------------------------------------------------------*/
/* Notes:                                                                    */
/*      1. Filter memory and cofficients should be 8-byte aligned            */
/*****************************************************************************/
#include "audio_iir_tdf2_asm.h"

#ifdef QDSP6_ASM_IIRTDF2_16

#if 0
/* Old ASM function, will be removed after more tests on the new ASM are done */
    .TEXT
    .p2align 4,,15
    .globl iirTDF2_16
    .type   iirTDF2_16, @function
iirTDF2_16:
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
      MEMD (SP + #32) = R23:22 
      MEMD (SP + #40) = R21:20 }

    { IF P1  R6 = ADD (R7, #-12)       //if (shiftn < shiftd) R6 = shiftY = shiftd - 12
      IF !P1 R6 = ADD (R6, #-12)       //else                 R6 = shiftY = shiftn - 12;
      MEMD (R29 + #24) = R25:24
      MEMD (R29 + #16) = R27:26 }

    { IF !P0 JUMP .LIIR_TDF2_16_END    //if (samples <=0) return
      R18 = ADD (R9, #-4)              //R18 = shiftDiffX - 4
      R7  = MEMW (R3 + #0)             //R7  = b0 = *numcoefs;
      R10 = MEMW (R3 + #4) }           //R10 = b1 = *(numcoefs + 1);

    { R19 = ADD (R9, #-3)              //R19 = shiftDiffX - 3
      R20 = ADD (R9, #-1)              //R20 = shiftDiffX - 1
      R11 = MEMW (R3 + #8)             //R11 = b2 = *(numcoefs + 2);
      R15:14 = MEMD (R5 + #0) }        //R15:14 = mem[0] = w1

    { R22 = ADD (R8, #-17)             //R22 = shiftDiffY - 17
      R21 = ADD (R8, #-19)             //R21 = shiftDiffY - 19
      R13    = MEMW (R4 + #4)          //R13 = a2 = *(dencoefs + 1);
      R12    = MEMW (R4 + #0) }        //R12 = a1 = *dencoefs;

    { NOP
      NOP
      R23    = MEMH (R0)               //R23 = *inp = x
      R17:16 = MEMD (R5 + #8) }        //R17:16 = mem[1] = w2

    { NOP
      R0     = ADD(R0, #2)             //inp++
      R25:24 = MPY (R7, R23)           //R25:24 = b0TimesX = b0 * *inp ;
      R27:26 = MPY (R10, R23)     }    //R27:26 = b1TimesX = b1 * *inp ;
      
    { R25:24 = ASL (R25:24, R18)       //R25:24 = b0TimesX = b0TimesX << shiftDiffX - 4
      R27:26 = ASL (R27:26, R19)  }    //R27:26 = b1TimesX = b1TimesX << shiftDiffX - 3 ;

    { R25:24 += ASR (R15:14, #1)       //R25:24 = y = b0TimesX + w1Temp
      NOP                         }

    { R25:24 = ASL (R25:24, R6)        //R25:24 = y << shiftY
      R2     = ADD(R2, #-1)            //reduce loop count by 1
      NOP
      NOP                         }

    { LOOP0 (.LIIRTDF2_16_LOOP, R2)     //for (i = 0; i < samples; ++i)
      R28    = SAT (R25:24)             //R28 = yScaled = sat(y)
      MEMW  (R29)  = R5           }

      R3:2   = MPY (R12, R28)          //R3:2   = a1TimesY = a1 * yScaled ;

    .falign
.LIIRTDF2_16_LOOP:

    { R23    = MEMH (R0++#2)           //R23 = *inp = x
      R4     = ASRH (R28)              //R4  = yScaled >> 16 (for *out)
      R9:8   = MPY (R11, R23)          //R9:8   = b2TimesX = b2 * *inp ;
      R3:2   = ASL (R3:2, R21)   }     //R3:2   = a1TimesY = a1TimesY << shiftDiffY - 19;

    { NOP
      R15:14 = SUB (R27:26, R3:2)      //R15:14 = w1 = Q6_P_sub_PP(b1TimesX,a1TimesY);
      R25:24 = MPY (R7, R23)           //R25:24 = b0TimesX = b0 * *inp ;
      MEMH (R1++#2) = R4         }     //*out = Q6_R_sath_R(yScaled >> 16)

    { R25:24 = ASL (R25:24, R18)       //R25:24 = b0TimesX = b0TimesX << shiftDiffX - 4
      R15:14 += ASR (R17:16, #2) }     //R15:14 = w1 = Q6_P_add_PP(w1, w2Temp);

    { R9:8   = ASL (R9:8, R20)         //R9:8   = b2TimesX = b2TimesX << shiftDiffX - 1 ;
      R25:24 += ASR (R15:14, #1) }     //R25:24 = y = b0TimesX + w1Temp

    { R5:4   = MPY (R13, R28)          //R5:4   = a2TimesY = a2 * yScaled ;
      R25:24 = ASL (R25:24, R6)  }     //R25:24 = y << shiftY

    { R5:4   = ASL (R5:4, R22)         //R5:4  = a2TimesY = a2TimesY << shiftDiffY - 17;
      R28    = SAT (R25:24)      }     //R28 = yScaled = sat(y)

    { R17:16 = SUB (R9:8, R5:4)        //R17:16 = w2 = Q6_P_sub_PP( b2TimesX , a2TimesY);
      R27:26 = MPY (R10, R23)    }     //R27:26 = b1TimesX = b1 * *inp ;

    { R27:26 = ASL (R27:26, R19)       //R27:26 = b1TimesX = b1TimesX << shiftDiffX - 3 ;
      R3:2   = MPY (R12, R28)  }:endloop0         //R3:2   = a1TimesY = a1 * yScaled ;

    { NOP
      R4     = ASRH (R28)              //R4  = yScaled >> 16 (for *out)
      R3:2   = ASL (R3:2, R21)         //R3:2   = a1TimesY = a1TimesY << shiftDiffY - 19;
      R9:8   = MPY (R11, R23)    }     //R9:8   = b2TimesX = b2 * *inp ;

    { NOP
      R15:14 = SUB (R27:26, R3:2)      //R15:14 = w1 = Q6_P_sub_PP(b1TimesX,a1TimesY);
      R5:4   = MPY (R13, R28)          //R5:4  = a2TimesY = a2 * yScaled ;
      MEMH (R1) = R4             }     //*out = Q6_R_sath_R(yScaled >> 16)

    { R15:14 += ASR (R17:16, #2)       //R15:14 = w1 = Q6_P_add_PP(w1, w2Temp);
      R9:8   = ASL (R9:8, R20)   }     //R9:8   = b2TimesX = b2TimesX << shiftDiffX - 1 ;

      R5:4   = ASL (R5:4, R22)         //R5:4  = a2TimesY = a2TimesY << shiftDiffY - 17;

    { R17:16 = SUB (R9:8, R5:4)        //R17:16 = w2 = Q6_P_sub_PP( b2TimesX , a2TimesY);
      R5  = MEMW (R29)           }

    { MEMD (R5 + #8) = R17:16          //mem[1] = w2
      MEMD (R5 + #0) = R15:14 }        //mem[0] = w1

.LIIR_TDF2_16_END:
    { R27:26 = MEMD (R29 + #16)           //restore registers and return
      R25:24 = MEMD (R29 + #24) }
    { R23:22 = MEMD (SP + #32) 
      R21:20 = MEMD (SP + #40) } 
    { R29 = ADD (R29, #64)
      JUMPR R31
      R19:18 = MEMD (SP + #48) 
      R17:16 = MEMD (SP + #56) }
    .size iirTDF2_16, .-iirTDF2_16

#else

    .TEXT
    .p2align 4,,15
    .globl iirTDF2_16
    .type   iirTDF2_16, @function
iirTDF2_16:
        { R29 = ADD (R29, #-48)
          MEMD (SP + #-8) = R17:16 ; MEMD (SP + #-16) = R19:18 }
        {
          MEMD (SP + #0) = R25:24
        }

        { R6 = MEMH (R29 + #48)                          //R6 = shiftn
          R7 = MEMH (R29 + #52) }                        //R7 = shiftd
          R24 = #14

        { P0 = CMP.GT (R7, R6)                           //P0 = shiftn < shiftd
          IF !P0.new R12 = SUB (R7, R6)                  //R12 = shiftDiffY = shiftd - shiftn 
          MEMD (SP + #24) = R21:20 ; MEMD (SP + #16) = R23:22 }

        { MEMD (SP + #8) = R27:26 
          IF P0 R25 = SUB (R24, R7)                     //((12 + GUARD_BITS_16 - 1) - shiftd) when shiftd > shiftn
          IF !P0 R25 = SUB (R24, R6)                    //((12 + GUARD_BITS_16 - 1) - shiftn) when shiftn <= shiftd
        }

        { IF !P0 R28 = ADD (R6, #-15)                    //R28 = shiftY = (shiftn-12-GUARD_BITS_16)
          IF (P0) R6 = SUB (R6, R7)                      //R6 = shiftDiffX = shiftn - shiftd
          IF (P0) R28 = ADD (R7, #-15)                   //R28 = shiftY = (shiftd-12-GUARD_BITS_16)
          IF (P0) R12 = #0 }                             //R12 = shiftDiffY = 0

        { IF !P0 R6 = #0                                 //R6 = shiftDiffX = 0
          P0 = CMP.GT (R2, #0)                           //samples > 0 ?
          R19 = MEMW (R3 + #8)                           //R19 = b2 = *(numcoefs + 2)
          R18 = MEMW (R4 + #4) }                         //R18 = a2 = *(dencoefs + 1)

        { R20 = MEMW (R3 + #0)                          //R20 = b0 = *numcoefs
          R21 = MEMW (R3 + #4)                          //R21 = b1 = *(numcoefs + 1)
          R24 = #1
          P1 = CMP.GT(R28, #-1)				                  //if shiftY > -1
        }

        { R4 = ADD (R6, #-4)                             //R4 = shiftDiffX - 4
          IF !P0 JUMP .LIIR_TDF2_16_END                  //if (samples <= 0) return
          IF (P0) R12 = ADD (R12, #-17)                  //R12 = shiftDiffY-20+GUARD_BITS_16 
          R22 = MEMW (R4 + #0) }                         //R22 = a1 = *dencoefs

        { R3 = MEMH (R0 ++ #2)                           //R3 = *inp++
          R6 = #4096                                     //R6 = 0x1000
          R24 = ASL(R24, R25)                            //p5 = (int32)(1 << (-shiftY - 1))
        }

        { R15:14 = MPY (R20, R3)                         //R15:14 = b0TimesX = Q6_P_mpy_RR( b0 , *inp)
          R17:16 = MPY (R21, R3)                         //R17:16 = b1TimesX = Q6_P_mpy_RR( b1 , *inp )
          R11:10 = MEMD (R5 + #8)                        //R11:10 = w2Temp = mem[1]
          R9:8 = MEMD (R5 + #0)   }                      //R9:8 = w1Temp = mem[0]

        { R15:14 = ASL (R15:14, R4)                      //R15:14 = b0TimesX = Q6_P_asl_PR( b0TimesX , shiftDiffX - 4 )
          R17:16 = ASL (R17:16, R4) }                    //R17:16 = b1TimesX = Q6_P_asl_PR( b1TimesX , shiftDiffX - 4 )
        {
          R9:8 = ADD (R15:14, R9:8)                      //R9:8 = y = Q6_P_add_PP( b0TimesX , w1Temp )          
          IF (P1) R24 = #0                               //p5 = 0 if shiftY > -1
        }
        {
          R9:8 = ADD (R24, R9:8)                         //y = s64_add_s64_s32(y, p5)
        }
        {
          R27:26 = ASL (R9:8, R28)                       //R27:26 = Q6_P_asl_PR(y,shiftY)
        }
        { R26  = SAT (R27:26)                            //R26 = yScaled = Q6_R_sat_P(Q6_P_asl_PR(y,shiftY))
          R2   = ADD(R2, #-1)      }                     //reduce loop count by 1

        { P3 = SP1LOOP0 (.LIIRTDF2_16_LOOP, R2)
          P0 = CMP.EQ(R2, #0)
          R9:8 = MPY (R22, R26)                         //R9:8 = a1TimesY = Q6_P_mpy_RR( a1 , yScaled )
          R2 = R24  }

  .falign
.LIIRTDF2_16_LOOP:
        { R25:24 = MPY (R19, R3)                         //R25:24 = b2TimesX = Q6_P_mpy_RR( b2 , *inp++ )
          R17:16 -= ASL (R9:8, R12)                      //R17:16 = w1 = Q6_P_sub_PP(b1TimesX,Q6_P_asl_PR( a1TimesY, shiftDiffY-20+GUARD_BITS_16));
          R13 = ADD (R26, R6):sat                        //R13  = yScaled = Q6_R_add_RR_sat(yScaled, 0x1000)
          R3 = MEMH (R0 ++ #2)    }                      //R3 = *inp++

        { 
          R15:14 = MPY (R20, R3)                         //R15:14 = b0TimesX = Q6_P_mpy_RR( b0 , *inp)
          IF P3 MEMH (R1 ++ #2) = R7                     //*out++ = Q6_R_sath_R( yScaled >> (16-GUARD_BITS_16) )
          R17:16 = ADD (R17:16, R11:10)     }            //R17:16 = mem[0] = w1 = Q6_P_add_PP(w1,w2Temp)        

        {  R25:24 = ASL (R25:24, R4)                     //R25:24 = b2TimesX = Q6_P_asl_PR( b2TimesX , shiftDiffX - 4)
           R17:16 += ASL (R15:14, R4)                    //R17:16 = y = Q6_P_add_PP( w1Temp, Q6_P_asl_PR( b0TimesX , shiftDiffX - 4 ))
           IF (P0) MEMD (R5 + #0) = R17:16   }           //mem[0] = w1
           
        { 
           R9:8 = ADD (R2, R17:16)                       //R9:8 = y = s64_add_s64_s32(p5, y)
           R13 = ASR (R13, #13)                          //R13 = yScaled >> (16-GUARD_BITS_16)
        }

        { R15:14 = MPY (R18, R26)                        //R15:14 = a2TimesY = Q6_P_mpy_RR( a2 , yScaled )
          R27:26 = ASL (R9:8, R28)   }                   //R27:26 = Q6_P_asl_PR(y,shiftY)

        { 
          R25:24 -= ASL (R15:14, R12)                    //R25:24 = mem[1] = w2 = Q6_P_sub_PP( b2TimesX , (a2TimesY << shiftDiffY-20+GUARD_BITS_16))
          R26 = SAT (R27:26)         }                   //R26 = yScaled = Q6_R_sat_P(Q6_P_asl_PR(y,shiftY))

        { 
          R7 = SATH (R13)                                //R13 = Q6_R_sath_R( yScaled >> (16-GUARD_BITS_16) )
          R11:10 = R25:24                               
          R17:16 = MPY (R21, R3) }                       //R17:16 = b1TimesX = Q6_P_mpy_RR( b1 , *inp )

        { 
          R17:16 = ASL (R17:16, R4)                      //R17:16 = b1TimesX = Q6_P_asl_PR( b1TimesX , shiftDiffX - 4 )
          IF (P0) MEMD (R5 + #8) = R11:10                //mem[1] = w2         
          R9:8 = MPY (R22, R26)     }:ENDLOOP0           //R9:8 = a1TimesY = Q6_P_mpy_RR( a1 , yScaled )

        { R25:24 = MPY (R19, R3)                         //R25:24 = b2TimesX = Q6_P_mpy_RR( b2 , *inp++ )
          R9:8 = ASL (R9:8, R12)                         //R9:8 = a1TimesY = Q6_P_asl_PR( a1TimesY, shiftDiffY-20+GUARD_BITS_16)
          R13 = ADD (R26, R6):sat                        //R13  = yScaled = Q6_R_add_RR_sat(yScaled, 0x1000)
          IF (P0) MEMH (R1) = R7 }                       //*out++ = Q6_R_sath_R( yScaled >> (16-GUARD_BITS_16) )

        { R9:8 = SUB (R17:16, R9:8)                      //R9:8 = w1 = Q6_P_sub_PP(b1TimesX,a1TimesY)
          R25:24 = ASL (R25:24, R4)                      //R25:24 = b2TimesX = Q6_P_asl_PR( b2TimesX , shiftDiffX - 4)
          IF (!P0) MEMH (R1 ++ #2) = R7 }                //*out++ = Q6_R_sath_R( yScaled >> (16-GUARD_BITS_16) )

        { R9:8 = ADD (R9:8, R11:10)                      //R9:8 = mem[0] = w1 = Q6_P_add_PP(w1,w2Temp)
          R15:14 = MPY (R18, R26)   }                    //R15:14 = a2TimesY = Q6_P_mpy_RR( a2 , yScaled )

        { R15:14 = ASL (R15:14, R12)                     //R15:14 = a2TimesY = Q6_P_asl_PR( a2TimesY, shiftDiffY-20+GUARD_BITS_16)
          R13 = ASR (R13, #13)      }                    //R13 = yScaled >> (16-GUARD_BITS_16)

        { R11:10 = SUB (R25:24, R15:14)                  //R11:10 = mem[1] = w2 = Q6_P_sub_PP( b2TimesX , a2TimesY)
          R7 = SATH (R13)                                //R13 = Q6_R_sath_R( yScaled >> (16-GUARD_BITS_16) )
          IF (!P0) MEMD (R5 + #0) = R9:8    }            //mem[0] = w1

        { IF (!P0) MEMD (R5 + #8) = R11:10               //mem[1] = w2
          IF (!P0) MEMH (R1) = R7           }            //*out++ = Q6_R_sath_R( yScaled >> (16-GUARD_BITS_16) )

.LIIR_TDF2_16_END:
        { R25:24 = MEMD (SP + #0) ; R27:26 = MEMD (SP + #8) }

        { R23:22 = MEMD (SP + #16) ; R21:20 = MEMD (SP + #24) }

#ifndef FEATURE_POPQUEUE_CONFLICT
        { R29 = ADD (R29, #48)
          JUMPR R31
          R19:18 = MEMD (SP + #32) ; R17:16 = MEMD (SP + #40) }
#else
{
 R29 = ADD (R29, #48)
          R19:18 = MEMD (SP + #32) ; R17:16 = MEMD (SP + #40) 
}
{
          JUMPR R31
}
#endif
    .size iirTDF2_16, .-iirTDF2_16

#endif //#if 1, temporarily keep the old ASM

#endif

