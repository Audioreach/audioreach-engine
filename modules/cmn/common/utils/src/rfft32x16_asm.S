	.file	"rfft.S"
#if ((defined __hexagon__) || (defined __qdsp6__))
/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  FUNCTIONS      : N-point real-valued FFT                            */
/*  ARCHITECTURE   : QDSP6V2                                            */
/*  VERSION        : 0001                                               */
/*                                                                      */ 
/*  DESCRIPTIONS:                                                       */
/*  =============                                                       */ 
/*   - FORMAT:                                                          */
/*      o INPUT: 32-bit real numbers                                    */
/*      o OUTPUT: 32-bit for real or image part                         */
/*      o TWIDDLES: 16-bit for real or image part                       */
/*   - ARRARY STRUCTURE:                                                */
/*      o Complex number: interleave real/image with real part at       */
/*                        low memory address                            */
/*   - ASSUMPTIONS:                                                     */
/*      o N > 16                                                        */
/*      o INPUT  ARRAY IS ALIGNED BY ARRAY SIZE                         */
/*      o OUTPUT ARRAY IS ALIGNED BY 8                                  */
/*      o OUTPUT HAS ONLY N/2+1 points because of Hermitia symmetry     */
/*                                                                      */
/*                                                                      */
/************************************************************************/
/*                                                                      */
/*   PEFORMANCE SUMMARY:                                                */
/*   ===================                                                */
/*                                                                      */
/*   ================================================================   */
/*                 |      Cycle Count Formulas                          */
/*   --------------+-------------------------------------------------   */
/*                 |                                                    */
/*   ================================================================   */
/*                                                                      */
/*======================================================================*/

	.text
/*======================================================================*/
/*  FUNCTION  rfft()                                                    */
/*======================================================================*/
/*  DESCRIPTIONS                                                        */
/*  ============                                                        */
/*   Implement N-point real-valued FFT with scaling factor 1/N          */
/*                                                                      */
/*  INPUTS                                                              */
/*  ============                                                        */
/*  R0:  Word32 *input -- pointer to input array                        */
/*  R1:  int N  -- number of points                                     */
/*  R2:  CWord2x16 Wt1 -- twiddle factors for N/2-point complex FFT     */
/*  R3:  CWord2x16 Wt2 -- twiddle factors for last stage                */
/*  R4:  CWord2x32 *output -- pointer to output array                   */
/*                                                                      */
/*======================================================================*/
    .p2align 2
    .p2align 4,,15
    .globl rfft
    .type	rfft, @function
rfft:
    { ALLOCFRAME(#16)                           // allocate stack
    }
    { MEMD(R29+#8) = R25:24                     // callee-saved registers
      R25:24 = COMBINE(R3,R1)                   // save Wt2 : N/4 in R25:24
      R1 = LSR(R1,#1)                           // func call: R1 = N/2    (for func call) 
    }
    /* ----------------------------------------------- */
    /* call N/2-point FFT                              */
    /* ----------------------------------------------- */
    { MEMW(R29+#0) = R26                        // callee-saved registers
      R3 = R4                                   // func call: R3 = output (for func call)
      R26 = R4                                  // save *output in R26
      CALL fft32x16
    }

    /* ----------------------------------------------- */
    /* calcuate last stage butterflies                 */
    /* ----------------------------------------------- */
.Lrfft_LastStage:
    { R1 = ADDASL(R26,R24,#2)                   // R1 = &Z[N/2]
      R0 = R26                                  // R0 = &Z[0]
      R14 = ASR(R24,#2)                         // N/4
      R7:6 = MEMD(R26+#0)                       // load Z[0]
    }
    { R10 = ADD(R6,R7):sat                      // L_real(Z[0]) + L_imag(Z[0])
      R11 = #0                                  // 
      R12 = SUB(R6,R7):sat                      // L_real(Z[0]) - L_imag(Z[0])
      R13 = #0                                  // 
    }
    { MEMD(R0++#8) = R11:10                     // save output[0]
      R2 = ADD(R0,#8)                           // &Z[1]
      R3 = ADD(R1,#-8)                          // &Z[N/2-1]
      R26 = MEMW(R29+#0)                        // restore callee-saved regs
    }
    { MEMD(R1++#-8) = R13:12                    // save output[N/2]
      R5 = R25                                  // Wt2
      R25:24 = MEMD(R29+#8)                     // restore callee-saved regs
      P3 =SP1LOOP0(.Lrfft_LastStage_LOOP,R14)   // 
    }

    .falign
.Lrfft_LastStage_LOOP:
    { R9:8 = MEMD(R3++#-8)                      //[1]load Z[N/2-i]
      R11:10 = VSUBW(R7:6,R11:10):sat           //[2]output[i]= L_Vsub(X,Y)
      R13:12 = VADDW(R7:6,R11:10):sat           //[2]L_Vadd(X,Y)
    }
    { R7:6 = MEMD(R2++#8)                       //[1]load Z[i]
      R14 = MEMW(R5++#4)                        //[1]load wt2[i-1]
      R9 = NEG(R9):sat                          //[1]L_conjugate(Z[N/2-i]
      R13 = NEG(R13):sat                        //[2]L_conjugate(L_Vadd(X,Y))
    }
    { R7:6 =  VAVGW(R7:6,R9:8):crnd             //[1]X= L_Vavg()
      R9:8 = VNAVGW(R7:6,R9:8):crnd:sat         //[1]Y= LVnavg()
      R15 = R14                                 //[1]
      IF P3 MEMD(R0++#8) = R11:10               //[2]save output[i]
    }
    { R11:10 = VMPYWEH(R9:8,R15:14):<<1:rnd:sat //[1]Y *Wt2[i-1]
      R8 = NEG(R9):sat                          //[1]Y *Wt2[i-1]
      R9 = R8                                   //[1]Y *Wt2[i-1]
      IF P3 MEMD(R1++#-8) = R13:12              //[2]save output[N/2-i]
    }
    { R11:10 +=VMPYWOH(R9:8,R15:14):<<1:rnd:sat //[1]Y *Wt2[i-1]
    }:endloop0

    { R11:10 = VSUBW(R7:6,R11:10):sat           //[e]output[i]= L_Vsub(X,Y)
      DEALLOCFRAME                              //
    }
#ifndef FEATURE_POPQUEUE_CONFLICT
    { MEMD(R0++#8) = R11:10                     //[e]save output[i]
      JUMPR R31
    }
#else
{
 MEMD(R0++#8) = R11:10                     //[e]save output[i]
    
}
{
      JUMPR R31
}
#endif
    .size	rfft, .-rfft
#endif

