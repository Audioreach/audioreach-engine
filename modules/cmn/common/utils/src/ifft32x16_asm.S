/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if ((defined __hexagon__) || (defined __qdsp6__))
#include "asm_macros.h"
/*  FUNCTIONS      : N-point Complex IFFT                               */
/*  ARCHITECTURE   : QDSP6V2                                            */
/*  VERSION        :                                                    */
/*                                                                      */ 
/************************************************************************/
/*                                                                      */
/*  DESCRIPTIONS:                                                       */
/*  =============                                                       */ 
/*   - FORMAT:                                                          */
/*      o INPUT/OUTPUT:  complex number with 32-bit real and image part */ 
/*                       interleaved                                    */
/*      o TWIDDLES:      twiddle factor as complex number with 16-bit   */
/*                        real and imaginary part interleaved           */
/*   - ASSUMPTIONS:                                                     */
/*      o INPUT  ARRAY IS ALIGNED BY ARRAY SIZE                         */
/*      o OUTPUT ARRAY IS ALIGNED BY 8                                  */
/*                                                                      */
/*   - IMPLEMENTATION:                                                  */
/*      o Radix-4 DIF                                                   */
/*      o TWIDDLE FACTOR ARRAY: IN BIT-REVERSE ORDER & INTERLEAVED      */
/*                                                                      */
/************************************************************************/
/*                                                                      */
/*   PEFORMANCE SUMMARY:                                                */
/*   ===================                                                */
/*                                                                      */
/*   ================================================================   */
/*                 |      Cycle Count Formulas                          */
/*   --------------+-------------------------------------------------   */
/*   N = 4^K       | (9N/4)*K + (N-4)/12 + 5K + 5                       */
/*   --------------+-------------------------------------------------   */
/*   N = 2*(4^K)   | (9N/4)*K + (13N-8)/12 + 5K + 9                     */
/*   ================================================================   */
/************************************************************************/	

    .text
/************************************************************************/
/*                                                                      */
/*                FFT FUNCTION                                          */
/*               ===============                                        */
/*   R0:  pointer to Input buffer                                       */
/*   R1:  Number of points N                                            */
/*   R2:  pointer to Wtwiddles factors                                  */
/*   R3:  pointer to Output buffer                                      */
/*                                                                      */
/************************************************************************/
    .p2align 2
    .p2align 4,,15
    .globl ifft32x16
    .type	ifft32x16, @function
ifft32x16:
     __saveonentry_32
    { R29 = ADD(R29,#-16)                           // allocate stack 
      R4 = #0                                       // R4 = 0
      R5 = BREV(R0)                                 // to bit reverse R0.L
      R28 = CL0(R1)                                 // used to calculate log2(N) 
    }
    { MEMD(R29+#0) = R27:26                         // push R27:26 to stack
      R0 = COMBINE(R0.H,R5.H)                       // bit reverse R0.L for bit-reversed addr
      R6 = ASR(R1, #2)                              // LC = N/4 for .Lifft32x16_Stage1_LOOP
      R7 = ADD(R28,#-18)                            // 16-log2(8N) = cl0(N)-18  
    }
    { MEMD(R29+#8) = R25:24                         // push R25:24 to stack
      R25 = R1                                      // save R1, i.e., N, in R25
      R4 = SETBIT(R4,R7)                            // R4 = 1<<(15 - log2(4N)) = 1<<(13-LOG2N) 
      P3 = SP1LOOP0(.Lifft32x16_Stage1_LOOP,R6)     // setup loop  
    }
    { M0 = R4                                       // set M0 for bit-reversed addressing
      R28 = SUB(#31,R28)                            // log2(N) = 31-cl0(N)  
      R1 = R3                                       // set R1=  Output
      R6 = R2                                       // set R6 = twiddle 
    }
    
    .falign
.Lifft32x16_Stage1_LOOP:
    { R11:10 = MEMD(R0++M0:brev)                    //[1] load A  
      R15:14 = VMPYWEH(R15:14, R13:12):<<1:rnd:sat  //[2]Im(C")*Re(Wb): Re(C")*Re(Wb) 
      R27:26 = VMPYWEH(R17:16, R19:18):<<1:rnd:sat  //[2]Im(D")*Re(Wc): Re(D")*Re(Wc)
      IF P3 MEMD(R1++#8 ) = R11:10                  //[2]Store B"
    }
    { R13:12 = MEMD(R0++M0:brev)                    //[1]load B 
      R15:14 += VMPYWOH(R21:20, R13:12):<<1:rnd:sat //[2]C"*conj(Wb)= Im(C")*Re(Wb)-Re(C")*Im(Wb): Re(C")*Re(Wb)+Im(C)*Im(Wb) 
      R20 = R17                                     //[2]
      R21 = NEG(R16):sat                            //[2]R21:20= -Re(D"): Im(D")
    }
    { R15:14 = MEMD(R0++M0:brev)                    //[1]load C 
      R11:10 =  VAVGW(R11:10,R13:12):crnd           //[1]A'= (A+B +1)/2
      R13:12 = VNAVGW(R11:10,R13:12):crnd:sat       //[1]B'= (A-B +1)/2
      IF P3 MEMD(R1++#8) = R15:14                   //[2]Store C""
    }
    { R17:16 = MEMD(R0++M0:brev)                    //[1]load D  
      R12 = R13                                     //[1]B'= -j*B' 
      R13 = NEG(R12):sat                            //[1]B'= -j*B' 
      R27:26 += VMPYWOH(R21:20, R19:18):<<1:rnd:sat //[2]D"*conj(Wc)= Im(D")*Re(Wc)-Re(D")*Im(Wc): Re(D")*Re(Wc)+Im(D)*Im(Wc)
    }
    { R15:14 =  VAVGW(R15:14,R17:16):crnd           //[1]C'= (C+D +1)/2 
      R17:16 = VNAVGW(R15:14,R17:16):crnd:sat       //[1]D'= (C-D +1)/2  
      R7 = MEMW(R6+#4)                              //[1]Load Wa    
      IF P3 MEMD(R1++#8) = R27:26                   //[2]Store D" 
    }	
    { R11:10 =  VAVGW(R11:10,R15:14):crnd           //[1]A"= (A'+C' +1)/2 
      R15:14 = VNAVGW(R11:10,R15:14):crnd:sat       //[1]C"= (A'-C' +1)/2
      R19:18 = COMBINE(R7,R7)                       //[1]R19:18 = Wa : Wa
      R9 = MEMW(R6+#8)                              //[1]Load Wc
    }
    { R13:12 =  VAVGW(R13:12,R17:16):crnd           //[1]B"= (B'+D' +1)/2 
      R17:16 = VNAVGW(R13:12,R17:16):crnd:sat       //[1]D"= (B'-D' +1)/2
      MEMD(R1++#8) = R11:10                         //[1]Store A" 
      R8 = MEMW(R6++#12)                            //[1]Load Wb
    }
    { R11:10 = VMPYWEH(R13:12, R19:18):<<1:rnd:sat  //[1]Im(B")*Re(Wa): Re(B")*Re(Wa)
      R20 = R13                                     //[1]
      R21 = NEG(R12):sat                            //[1]R21:20= -Re(B"): Im(B")
      R13:12 = COMBINE(R8,R8)                       //[1]R13:12 = Wb : Wb
    }
    { R11:10 += VMPYWOH(R21:20, R19:18):<<1:rnd:sat //[1]B"*conj(Wa)= Im(B")*Re(Wa)-Re(B")*Im(Wa): Re(B")*Re(Wa)+Im(B)*Im(Wa)
      R20 = R15                                     //[1]
      R21 = NEG(R14):sat                            //[1]R21:20= -Re(C"): Im(C")
      R19:18 = COMBINE(R9,R9)                       //[1]R19:18 = Wc : Wc
    }:endloop0

.Lifft32x16_Stage1_LOOPEND:
    { MEMD(R1++#8 ) = R11:10                        //[e]Store B"
      R15:14 = VMPYWEH(R15:14, R13:12):<<1:rnd:sat  //[e]Im(C")*Re(Wb): Re(C")*Re(Wb) 
      R27:26 = VMPYWEH(R17:16, R19:18):<<1:rnd:sat  //[e]Im(D")*Re(Wc): Re(D")*Re(Wc)
      R22 = #4*8                                    // initial value of M0 = 4 samples, i.e., 8*k1
    }
    { R15:14 += VMPYWOH(R21:20, R13:12):<<1:rnd:sat //[e]C"*conj(Wb)= Im(C")*Re(Wb)-Re(C")*Im(Wb): Re(C")*Re(Wb)+Im(C)*Im(Wb) 
      R20 = R17                                     //[e]
      R21 = NEG(R16):sat                            //[e]R21:20= Re(D"): -Im(D")
      R23 = #-3*4*8                                 // -3M0 
    }
    { MEMD(R1++#8) = R15:14                         //[e]Store C""
      R27:26 += VMPYWOH(R21:20, R19:18):<<1:rnd:sat //[e]D"*conj(Wc)= Im(D")*Re(Wc)-Re(D")*Im(Wc): Re(D")*Re(Wc)+Im(D)*Im(Wc)
      M0 = R22                                      // set M0 = 8*k1
    }
    { MEMD(R1) = R27:26                             //[e]Store D" 
      R5 = ASR(R25,#4)                              // initial k2 = N/16
      R1:0 = COMBINE(R3,R3)                         // set read/write pointer R0/R1 
    }	

    .falign
.Lifft32x16_MAIN_LOOP:
    { R15 = ADD(R23,#8)                             //-3*M0+8
      R4 = ASR(R22,#3)                              // R4 = k1
      LOOP1(.Lifft32x16_MAIN_OUTER_LOOP,R5)         // setup loop1  
      R11:10 = MEMD(R0++M0)                         //[p0]load A  
    }
    { M1 = R15                                      //set M1 = -3*M0 + 8
      R4 = ADD(R4,#-1)                              // R4 = k1-1
      R6 = R2                                       // set R6 = *Wtwiddles
      R13:12 = MEMD(R0++M0)                         //[p0]load B 
    }
    { R15:14 = MEMD(R0++M0)                         //[p0]load C 
      R11:10 =  VAVGW(R11:10,R13:12):crnd           //[p0]A'= (A+B +1)/2
      R13:12 = VNAVGW(R11:10,R13:12):crnd:sat       //[p0]B'= (A-B +1)/2
      R24 = #1                                      // cnt
    }

    .falign
.Lifft32x16_MAIN_OUTER_LOOP:
    { R17:16 = MEMD(R0++M1)                         //[p0]load D  
      R12 = R13                                     //[p0]B'= -j*B' 
      R13 = NEG(R12):sat                            //[p0]B'= -j*B' 
      LOOP0(.Lifft32x16_MAIN_INNER_LOOP,R4)         //setup loop0 with LC= k2-1
    }
    { R15:14 =  VAVGW(R15:14,R17:16):crnd           //[p0]C'= (C+D+1)/2 
      R17:16 = VNAVGW(R15:14,R17:16):crnd:sat       //[p0]D'= (C-D+1)/2  
      R7 = MEMW( R6+#4)                             //Load Wa
      R9 = MEMW(R6+#8)                              //Load Wc 
    }	
    { R8 = MEMW( R6++#12)                           // Load Wb 
      R11:10 =  VAVGW(R11:10,R15:14):crnd           //[1]A"= (A'+C'+1)/2 
      R15:14 = VNAVGW(R11:10,R15:14):crnd:sat       //[1]C"= (A'-C'+1)/2
      R19:18 = COMBINE(R7,R7)                       //[1]R19:18 = Wa : Wa
    }
    { P1 = CMP.EQ(R24,R5)                           // last iteration of outer loop?
      R13:12 =  VAVGW(R13:12,R17:16):crnd           //[1]B"= (B'+D'+1)/2 
      R17:16 = VNAVGW(R13:12,R17:16):crnd:sat       //[1]D"= (B'-D'+1)/2
      MEMD(R1++M0) = R11:10                         //[1]Store A" 
    }
    { R11:10 = VMPYWEH(R13:12, R19:18):<<1:rnd:sat  //[1]Im(B")*Re(Wa): Re(B")*Re(Wa)
      R20 = R13                                     //[1]
      R21 = NEG(R12):sat                            //[1]R21:20= -Re(B"): Im(B")
      R13:12 = COMBINE(R8,R8)                       //[1]R13:12 = Wb : Wb
    }
    { R11:10 += VMPYWOH(R21:20, R19:18):<<1:rnd:sat //[1]B"*conj(Wa)= Im(B")*Re(Wa)-Re(B")*Im(Wa): Re(B")*Re(Wa)+Im(B)*Im(Wa)
      R20 = R15                                     //[1]
      R21 = NEG(R14):sat                            //[1]R21:20= -Re(C"): Im(C")
      R19:18 = COMBINE(R9,R9)                       //[1]R19:18 = Wc : Wc
    }

    .falign
.Lifft32x16_MAIN_INNER_LOOP:
    { R11:10 = MEMD(R0++M0)                         //[1] load A  
      R15:14 = VMPYWEH(R15:14, R13:12):<<1:rnd:sat  //[2]Im(C")*Re(Wb): Re(C")*Re(Wb) 
      R27:26 = VMPYWEH(R17:16, R19:18):<<1:rnd:sat  //[2]Im(D")*Re(Wc): Re(D")*Re(Wc)
      MEMD( R1++M0 ) = R11:10                       //[2]Store B"
    }
    { R13:12 = MEMD(R0++M0)                         //[1]load B 
      R15:14 += VMPYWOH(R21:20, R13:12):<<1:rnd:sat //[2]C"*conj(Wb)= Im(C")*Re(Wb)-Re(C")*Im(Wb): Re(C")*Re(Wb)+Im(C)*Im(Wb) 
      R20 = R17                                     //[2]
      R21 = NEG(R16):sat                            //[2]R21:20= -Re(D"): Im(D")
    }
    { R15:14 = MEMD(R0++M0)                         //[1]load C 
      R11:10 =  VAVGW(R11:10,R13:12):crnd           //[1]A'= (A+B+1)
      R13:12 = VNAVGW(R11:10,R13:12):crnd:sat       //[1]B'= (A-B+1)
      MEMD( R1++M0) = R15:14                        //[2]Store C""
    }
    { R17:16 = MEMD(R0++M1)                         //[1]load D  
      R12 = R13                                     //[1]B'= -j*B' 
      R13 = NEG(R12):sat                            //[1]B'= -j*B' 
      R27:26 += VMPYWOH(R21:20, R19:18):<<1:rnd:sat //[2]D"*conj(Wc)= Im(D")*Re(Wc)-Re(D")*Im(Wc): Re(D")*Re(Wc)+Im(D)*Im(Wc)
    }
    { R15:14 =  VAVGW(R15:14,R17:16):crnd           //[1]C'= (C+D+1)/2 
      R17:16 = VNAVGW(R15:14,R17:16):crnd:sat       //[1]D'= (C-D+1)/2  
      MEMD(R1++M1) = R27:26                         //[2]Store D" 
    }	
    { R11:10 =  VAVGW(R11:10,R15:14):crnd           //[1]A"= (A'+C'+1)/2 
      R15:14 = VNAVGW(R11:10,R15:14):crnd:sat       //[1]C"= (A'-C'+1)/2
      R19:18 = COMBINE(R7,R7)                       //[1]R19:18 = Wa : Wa
    }
    { R13:12 =  VAVGW(R13:12,R17:16):crnd           //[1]B"= (B'+D'+1)/2
      R17:16 = VNAVGW(R13:12,R17:16):crnd:sat       //[1]D"= (B'-D'+1)/2
      MEMD(R1++M0) = R11:10                         //[1]Store A" 
    }
    { R11:10 = VMPYWEH(R13:12, R19:18):<<1:rnd:sat  //[1]Im(B")*Re(Wa): Re(B")*Re(Wa)
      R20 = R13                                     //[1]
      R21 = NEG(R12):sat                            //[1]R21:20= -Re(B"): Im(B")
      R13:12 = COMBINE(R8,R8)                       //[1]R13:12 = Wb : Wb
    }
    { R11:10 += VMPYWOH(R21:20, R19:18):<<1:rnd:sat //[1]B"*conj(Wa)= Im(B")*Re(Wa)-Re(B")*Im(Wa): Re(B")*Re(Wa)+Im(B)*Im(Wa)
      R20 = R15                                     //[1]
      R21 = NEG(R14):sat                            //[1]R21:20= Re(C"): -Im(C")
      R19:18 = COMBINE(R9,R9)                       //[1]R19:18 = Wc : Wc
    }:endloop0

    { R15:14 = VMPYWEH(R15:14, R13:12):<<1:rnd:sat  //[e]Im(C")*Re(Wb): Re(C")*Re(Wb) 
      R27:26 = VMPYWEH(R17:16, R19:18):<<1:rnd:sat  //[e]Im(D")*Re(Wc): Re(D")*Re(Wc)
      MEMD( R1++M0 ) = R11:10                       //[e]Store B"
      IF (!P1) R0 = SUB(R0,R23)                     // R0 = R0+3*M0 
                                                    // if it's last iteration, no update to control out-of-range load)  
    }
    { R15:14 += VMPYWOH(R21:20, R13:12):<<1:rnd:sat //[e]C"*conj(Wb)= Im(C")*Re(Wb)-Re(C")*Im(Wb): Re(C")*Re(Wb)+Im(C)*Im(Wb) 
      R20 = R17                                     //[e]
      R21 = NEG(R16):sat                            //[e]R21:20= -Re(D"): Im(D")
      R11:10 = MEMD(R0++M0)                         //[p0] load A  
    }
    { MEMD( R1++M0) = R15:14                        //[e]Store C""
      R27:26 += VMPYWOH(R21:20, R19:18):<<1:rnd:sat //[e]D"*conj(Wc)= Im(D")*Re(Wc)-Re(D")*Im(Wc): Re(D")*Re(Wc)+Im(D)*Im(Wc)
      R13:12 = MEMD(R0++M0)                         //[p0]load B 
      R24 = ADD(R24,#1)                             // cnt++
    }
    { MEMD(R1++#8) = R27:26                         //[e]Store D" 
      R11:10 =  VAVGW(R11:10,R13:12):crnd           //[p0]A'= A+B
      R13:12 = VNAVGW(R11:10,R13:12):crnd:sat       //[p0]B'= A-B
      R15:14 = MEMD(R0++M0)                         //[p0]load C 
    }:endloop1 
    

    { R23:22 = VASLW(R23:22,#2)                     //update 3*8*k1 : 8*k1 
      R1:0 = COMBINE(R3,R3)                         // R1:0 = output
      R5 = ASR(R5,#2)                               // k2 >>= 2
      P0 = CMP.GT(R5,#3)                            // P0 = !(k2 < 4)
    }
    { M0 = R22                                      //set M0
      IF P0 JUMP .Lifft32x16_MAIN_LOOP              // IF P0 continue loop
    }

.Lifft32x16_RADIX4_STAGES_END:
    { R5 = R25                                      // R5 = N
      R27:26 = MEMD(R29+#0)                         // restore R27:26
      R25:24 = MEMD(R29+#8)                         // restore R25:24
      P0 = TSTBIT(R28,#0)                           // N is 2*(4^K) ?   
    }
    { R29 = ADD(R29,#16)                            // pop stack
      IF !P0 JUMP  .Lifft32x16_LEAVE                   // If P0=FALSE Return via main exit
    }

.Lifft32x16_RADIX2_STAGE:
    { R1 = R3                                       // points to output
      R0 = ADDASL(R3,R5,#2)                         // points to output[N/2] 
      R4 = ASR(R5,#1)                               // N/2
    }
    { R2 = R0                                       // R2 points to output[N/2]
      P3 = SP1LOOP0(.Lifft32x16_RADIX2_LOOP,R4)	
      R7:6 = MEMD(R1++#8)                           //[p]load A
    }

    .falign
.Lifft32x16_RADIX2_LOOP:
    { R9:8 = MEMD(R0++#8)                           //[2]load B
      IF (P3) MEMD(R2++#8) = R13:12                 //[3]save B'
    }
    { R11:10 =  VAVGW(R7:6,R9:8):crnd               //[2]A'= (A+B+1)/2
      R13:12 = VNAVGW(R7:6,R9:8):crnd:sat           //[2]B'= (A-B+1)/2 
      IF (P3) MEMD(R3++#8) = R11:10                 //[3]save A'
      R7:6 = MEMD(R1++#8)                           //[1]load A 
    }:endloop0

    MEMD(R2) = R13:12                               //[e]save results
    { MEMD(R3) = R11:10                             //[e]save results
    }
.Lifft32x16_LEAVE:
#ifndef FEATURE_POPQUEUE_CONFLICT
    __restoreonexit_32
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
    .size	ifft32x16, .-ifft32x16
#endif	
