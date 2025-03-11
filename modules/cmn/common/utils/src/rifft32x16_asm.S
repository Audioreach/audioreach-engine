    .file	"rifft32x16.S"
#if ((defined __hexagon__) || (defined __qdsp6__))

/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  FUNCTIONS      : N-point real-valued IFFT                           */
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
/*  FUNCTION  rifft()                                                   */
/*======================================================================*/
/*  DESCRIPTIONS                                                        */
/*  ============                                                        */
/*   Implement N-point real-valued IFFT with scaling factor 1/N         */
/*                                                                      */
/*  INPUTS                                                              */
/*  ============                                                        */
/*  R0:  Word32 *input -- pointer to input array                        */
/*  R1:  int N  -- number of points                                     */
/*  R2:  CWord2x16 Wt1 -- twiddle factors for N/2-point complex FFT     */
/*  R3:  CWord2x16 Wt2 -- twiddle factors for first stage               */
/*  R4:  CWord2x32 *output -- pointer to output array                   */
/*                                                                      */
/*======================================================================*/
    .p2align 2
    .p2align 4,,15
    .globl rifft
    .type	rifft, @function
rifft:
    /* ----------------------------------------------- */
    /* calcuate first stage butterflies                */
    /* ----------------------------------------------- */
    { R29 = ADD(R29,#-8)                        // 
      R28 = ASR(R1,#1)                          // N/2
      R1 = ADDASL(R0,R1,#2)                     // R1 = &input[N/2]
      R7:6 = MEMD(R0+#0)                        // load input[0]
    }
    { MEMD(R29+#0) = R27:26                     // callee-saved registers
      R9:8 = MEMD(R1++#-8)                      // load input[N/2]
      R27:26 = COMBINE(R2,R0)                   // save Wt1 : input
    }
    { R7:6 =  VAVGW(R7:6,R9:8):crnd             // L_Vavg(input[0], input[N/2])
      R9:8 = VNAVGW(R7:6,R9:8):crnd:sat         // L_Vnavg(input[0], input[N/2])
    }
    { R11:10 = COMBINE(R8,R6)                   // input[0] = L_complex()
      R5 = R3                                   // Wt2 
      R14 = ASR(R28,#1)                         // N/4
    }
    { MEMD(R0++#8) = R11:10                     // save input[0]
      R2 = ADD(R0,#8)                           // &input[1]
      R3 = R1                                   // &input[N/2-1]
      P3 =SP1LOOP0(.Lrifft_FirstStage_LOOP,R14) // 
    }

    .falign
.Lrifft_FirstStage_LOOP:
    { R9:8 = MEMD(R3++#-8)                      //[1]load input[N/2-i]
      R11:10 = VSUBW(R7:6,R11:10):sat           //[2]input[i]= L_Vsub(X,Y)
      R13:12 = VADDW(R7:6,R11:10):sat           //[2]L_Vadd(X,Y)
    }
    { R7:6 = MEMD(R2++#8)                       //[1]load Z[i]
      R9 = NEG(R9):sat                          //[1]L_conjugate(input[N/2-i]
      R14 = MEMW(R5++#4)                        //[1]load wt2[i-1]
      R13 = NEG(R13):sat                        //[2]L_conjugate(L_Vadd(X,Y))
    }
    { R7:6 =  VAVGW(R7:6,R9:8):crnd             //[1]X = L_Vavg(input[i], input[N/2-i])
      R9:8 = VNAVGW(R7:6,R9:8):crnd:sat         //[1]Y = L_Vnavg(input[i], input[N/2-i])
      R15 = R14                                 //[1]
      IF P3 MEMD(R0++#8) = R11:10               //[2]save output[i]
    }
    { R11:10 = VMPYWEH(R9:8,R15:14):<<1:rnd:sat //[1]Y *conj(Wt2[i-1])
      R8 = R9                                   //[1]Y *conj(Wt2[i-1])
      R9 = NEG(R8):sat                          //[1]Y *conj(Wt2[i-1])
      IF P3 MEMD(R1++#-8) = R13:12              //[2]save input[N/2-i]
    }
    { R11:10 += VMPYWOH(R9:8,R15:14):<<1:rnd:sat //[1]Y *Wt2[i-1]
    }:endloop0

    { R11:10 = VSUBW(R7:6,R11:10):sat           //[e]input[e]= L_Vsub(X,Y)
      R3:2 = COMBINE(R4,R27)                    // func call: R2 = Wt1, R3 = output
      R29 = ADD(R29,#8)                         // pop stack
    }
    /* ----------------------------------------------- */
    /* call N/2-point FFT                              */
    /* ----------------------------------------------- */
    { MEMD(R0) = R11:10                         //[e]save output[i]
      R1:0 = COMBINE(R28,R26)                   // func call: R0 = input, R1 = N/2
      R27:26 = MEMD(R29+#-8)                    // restore callee-saved registers
      JUMP ifft32x16                            // call ifft32x16(input, N/2, Wt1, output)
    }
    .size	rifft, .-rifft
#endif

