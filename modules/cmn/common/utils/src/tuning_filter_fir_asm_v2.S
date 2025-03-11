/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  FUNCTIONS      : Real Block FIR                                     */
/*  ARCHITECTURE   : QDSP6 V2                                           */
/*  VERSION        : Rehash of bkfir function from QDSP6 examples       */
/*                   for Voice applications                             */
/*                                                                      */
#if ((defined __hexagon__) || (defined __qdsp6__))
#include "asm_macros.h"
/************************************************************************/
/*                                                                      */
/*  DESCRIPTIONS:                                                       */
/*  =============                                                       */
/*   - FORMAT:                                                          */
/*      o INPUT/OUTPUT/COEFFICIENT: in Half-Word                        */
/*   - ARRARY STRUCTURE:                                                */
/*      o Coefficients arranged in reverse order                        */
/*                                                                      */
/*   - ASSUMPTIONS:                                                     */
/*      o N multiple of 4, and T multiples of 4                         */
/*      o Input/Output array is aligned by 8                            */
/*      o Coefficient array is aligned by 8                             */
/*                                                                      */
/*   - IMPLEMENTATION:                                                  */
/*      o process 4 samples per iteration in inner loop                 */
/*      o generate 4 outputs per iteration in outer loop                */
/*                                                                      */
/*  CYCLE-COUNT:                                                        */
/*  ============                                                        */
/*                                                                      */
/************************************************************************/


/************************************************************************/
/*                                                                      */
/*                Block FIR FUNCTION                                    */
/*                ==================                                    */
/*   R0:  Word16 *xin   -- pointer to Input buffer                      */
/*   R1:  Word16 *coefs -- pointer to Coefficients                      */
/*   R2:  int taps      -- Number of Taps                               */
/*   R3:  int length    -- Number of Samples                            */
/*   R4:  int Qshift    -- shift factor                                 */
/*   R5:  Word16 *yout  -- pointer to output buffer                     */
/*                                                                      */
/************************************************************************/
   .text
   .p2align 5
   .globl tuning_filter_fir
   .type tuning_filter_fir, @function
tuning_filter_fir:
    __saveonentry_32                   //save R16:R23 on stack (arch = v3)
   { R3:2  = VASRW(R3:2, #2)           // N/4 : T/4
     R28 = R1                          // save coefs in R28
     R7:6 = MEMD(R0++#8)               // R7:6 = x3|x2|x1|x0
     R10 = MEMW(R0+#8)                 // R10 = x5|x4
   }
   { R15:14 = MEMD(R1++#8)             // load coefficients
     R2 = ADD(R2,#-2)                  // LC0 = T/4-2
     P0 = CMP.GT(R3,#0)                // P0 = (N/4 > 0)
     P3 = CMP.GT(R2,R2)                // clean P3
   }
   { R9:8 = COMBINE(R10,R7)            // R9:8 = x5|x4|x3|x2,
     R4 = ADD(R4,#16)
     R17:16 = VRMPYH(R7:6,R15:14)      //[p] sum0 += coefs[] * x[]
                                       //[x3|x2|x1|x0]*[C3|C2|C1|C0]]
     IF !P0 JUMP .ltuning_filter_fir_END  // if N<4 return
   }
   { LOOP1(MAIN_LOOPSTART,R3)
     R3 = ADD(R0,#8)                   // set read pointer
     R12 = MEMW(R0+#4)                 //[p] R12 = x7|x6
     R19:18 = VRMPYH(R9:8,R15:14)      //[p] sum2 += coefs[] * x[]
   }

   .falign
MAIN_LOOPSTART:
   { IF P3 MEMD(R5++#8) = R21:20       // save yout[i] -- yout[i+4]
     R7:6 = COMBINE(R12,R10)           // R7:6 = x7|x6|x5|x4
     R11:10 = VALIGNB(R11:10,R7:6,#2)  // R11:10 = x4|x3|x2|x1
     LOOP0(INNER_LOOPSTART,R2)
   }
   { R13:12 = VALIGNB(R13:12,R9:8,#2)  // R13:12 = x6|x5|x4|x3
     R21:20 = VRMPYH(R11:10,R15:14)    // sum1 += coefs[] * x[]
     R9 = MEMW(R3++#4)                 // R8 = x9|x8
     P0 = CMP.GT(R2,#0)                // P0 = (LC0 > 0)
   }
   { R15:14 = MEMD(R1++#8)             // load coefficients
     R23:22 = VRMPYH(R13:12,R15:14)    // sum3 += coefs[] * x[]
     R8 = R7                           // R9:8 = x9|x8|x7|x6
     IF !P0 JUMP INNER_LOOPEND
   }

   .falign
INNER_LOOPSTART:
   { R17:16 += VRMPYH(R7:6,R15:14)     // sum0 += coefs[] * x[]
     R19:18 += VRMPYH(R9:8,R15:14)     // sum2 += coefs[] * x[]
     R10 = R9                          // R10 = x5|4
     R12 = MEMW(R3++#4)                // R12 = x7|x6
   }
   { R11:10 = VALIGNB(R11:10,R7:6,#2)  // R11:10 = x4|x3|x2|x1
     R13:12 = VALIGNB(R13:12,R9:8,#2)  // R13:12 = x6|x5|x4|x3
     R7:6 = COMBINE(R12,R10)           // R7:6 = x7|x6|x5|x4
     R9 = MEMW(R3++#4)                 // R8 = x9|x8
   }
   { R21:20 += VRMPYH(R11:10,R15:14)   // sum1 += coefs[] * x[]
     R23:22 += VRMPYH(R13:12,R15:14)   // sum3 += coefs[] * x[]
     R8 = R7                           // R9:8 = x9|x8|x7|x6
     R15:14 = MEMD(R1++#8)             // load coefficients
   }:endloop0

INNER_LOOPEND:
   { R17:16 += VRMPYH(R7:6,R15:14)     // sum0 += coefs[] * x[]
     R19:18 += VRMPYH(R9:8,R15:14)     // sum2 += coefs[] * x[]
     R10 = R9                          // R10 = x5|4
     R12 = MEMW(R3++#4)                // R12 = x7|x6
   }
   { R11:10 = VALIGNB(R11:10,R7:6,#2)  // R11:10 = x4|x3|x2|x1
     R13:12 = VALIGNB(R13:12,R9:8,#2)  // R13:12 = x6|x5|x4|x3
     R7:6 = MEMD(R0++#8)               // [2] R7:6 = x3|x2|x1|x0
     R9 = MEMW(R0+#8)                  // [2] R9 = x5|x4
   }
   { R21:20 += VRMPYH(R11:10,R15:14)   // sum1 += coefs[] * x[]
     R23:22 += VRMPYH(R13:12,R15:14)   // sum3 += coefs[] * x[]
     R3 = ADD(R0,#4)                   // [2]
     R1 = R28                          // [2]
   }
   { R17:16 = ASR(R17:16,R4)           // sum0 >>= Qshift
     R21:20 = ASR(R21:20,R4)           // sum1 >>= Qshift
     R8 = R7                           // [2] R9:8 = x5|x4|x3|x2
     R10 = R9                          // [2] R10 = x5|4
   }
   { R19:18 = ASR(R19:18,R4)           // sum2 >>= Qshift
     R23:22 = ASR(R23:22,R4)           // sum3 >>= Qshift
     R15:14 = MEMD(R1++#8)             // [2] load coefficients
   }
   { R21:20 = COMBINE(R20,R16)         // pack to sum1:sum0
     R23:22 = COMBINE(R22,R18)         // pack to sum3:sum2
     R17:16 = VRMPYH(R7:6,R15:14)      // [2] sum0 += coefs[] * x[]
     R19:18 = VRMPYH(R9:8,R15:14)
   }
   { R20 = VSATWH(R21:20)              // saturate to 16-bit
     R21 = VSATWH(R23:22)              // saturate to 16-bit
     P3 = CMP.EQ(R2,R2)                // set P3 (for conditional write)
     R12 = MEMW(R3++#4)                //[2] R12 = x7|x6
   }:endloop1

   { MEMD(R5) = R21:20
   }
.ltuning_filter_fir_END:
#ifndef FEATURE_POPQUEUE_CONFLICT
   __restoreonexit_32                  //restore R16:R23 from stack (arch = v3) and return
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
   .size tuning_filter_fir, .-tuning_filter_fir
   
   
/************************************************************************/
/*                                                                      */
/*                Block FIR FUNCTION                                    */
/*                ==================                                    */
/*   R0:  Word16 *xin   -- pointer to Input buffer                      */
/*   R1:  Word16 *coefs -- pointer to Coefficients                      */
/*   R2:  int taps      -- Number of Taps                               */
/*   R3:  int length    -- Number of Samples                            */
/*   R4:  int Qshift    -- shift factor                                 */
/*   R5:  Word16 *yout  -- pointer to output buffer                     */
/*                                                                      */

/*  EDIT HISTORY FOR MODULE                                             */
/*    when          who             what, where, why                    */
/*  ---------- 	----------- 	---------------------------------       */
/*  2015-09-06   adeepak       Added rounding in fir_c16xd16            */
/************************************************************************/
   .text
   .p2align 5
   .globl tuning_filter_fir_rnd
   .type tuning_filter_fir_rnd, @function
tuning_filter_fir_rnd:
   {
     ALLOCFRAME(#40)                   //save R16:R25 on stack
   }
   {
     MEMD(R29 + #0) = R17:16;
     MEMD(R29 + #8) = R19:18;
   }
   {
     MEMD(R29 + #16) = R21:20;
     MEMD(R29 + #24) = R23:22;
   }
   {
     MEMD(R29 + #32) = R25:24;
   }
   { R3:2  = VASRW(R3:2, #2)           // N/4 : T/4
     R28 = R1                          // save coefs in R28
     R7:6 = MEMD(R0++#8)               // R7:6 = x3|x2|x1|x0
     R10 = MEMW(R0+#8)                 // R10 = x5|x4
   }
   { R15:14 = MEMD(R1++#8)             // load coefficients
     R2 = ADD(R2,#-2)                  // LC0 = T/4-2
     P0 = CMP.GT(R3,#0)                // P0 = (N/4 > 0)
     P3 = CMP.GT(R2,R2)                // clean P3
   }
   { R9:8 = COMBINE(R10,R7)            // R9:8 = x5|x4|x3|x2,
     R4 = ADD(R4,#16)
     R17:16 = VRMPYH(R7:6,R15:14)      //[p] sum0 += coefs[] * x[]
                                       //[x3|x2|x1|x0]*[C3|C2|C1|C0]]
     IF !P0 JUMP .ltuning_filter_fir_END_RND  // if N<4 return
   }
   { LOOP1(MAIN_LOOPSTART_C16XD16_RND,R3)
     R3 = ADD(R0,#8)                   // set read pointer
     R12 = MEMW(R0+#4)                 //[p] R12 = x7|x6
     R19:18 = VRMPYH(R9:8,R15:14)      //[p] sum2 += coefs[] * x[]
   }
   {
     R25:24 = ##16384                 //offset of 0x4000 since accumulator is right-shifted by 15
   }

   .falign
MAIN_LOOPSTART_C16XD16_RND:
   { IF P3 MEMD(R5++#8) = R21:20       // save yout[i] -- yout[i+4]
     R7:6 = COMBINE(R12,R10)           // R7:6 = x7|x6|x5|x4
     R11:10 = VALIGNB(R11:10,R7:6,#2)  // R11:10 = x4|x3|x2|x1
     LOOP0(INNER_LOOPSTART_C16XD16_RND,R2)
   }
   { R13:12 = VALIGNB(R13:12,R9:8,#2)  // R13:12 = x6|x5|x4|x3
     R21:20 = VRMPYH(R11:10,R15:14)    // sum1 += coefs[] * x[]
     R9 = MEMW(R3++#4)                 // R8 = x9|x8
     P0 = CMP.GT(R2,#0)                // P0 = (LC0 > 0)
   }
   { R15:14 = MEMD(R1++#8)             // load coefficients
     R23:22 = VRMPYH(R13:12,R15:14)    // sum3 += coefs[] * x[]
     R8 = R7                           // R9:8 = x9|x8|x7|x6
     IF !P0 JUMP INNER_LOOPEND_C16XD16_RND
   }

   .falign
INNER_LOOPSTART_C16XD16_RND:
   { R17:16 += VRMPYH(R7:6,R15:14)     // sum0 += coefs[] * x[]
     R19:18 += VRMPYH(R9:8,R15:14)     // sum2 += coefs[] * x[]
     R10 = R9                          // R10 = x5|4
     R12 = MEMW(R3++#4)                // R12 = x7|x6
   }
   { R11:10 = VALIGNB(R11:10,R7:6,#2)  // R11:10 = x4|x3|x2|x1
     R13:12 = VALIGNB(R13:12,R9:8,#2)  // R13:12 = x6|x5|x4|x3
     R7:6 = COMBINE(R12,R10)           // R7:6 = x7|x6|x5|x4
     R9 = MEMW(R3++#4)                 // R8 = x9|x8
   }
   { R21:20 += VRMPYH(R11:10,R15:14)   // sum1 += coefs[] * x[]
     R23:22 += VRMPYH(R13:12,R15:14)   // sum3 += coefs[] * x[]
     R8 = R7                           // R9:8 = x9|x8|x7|x6
     R15:14 = MEMD(R1++#8)             // load coefficients
   }:endloop0

INNER_LOOPEND_C16XD16_RND:
   { R17:16 += VRMPYH(R7:6,R15:14)     // sum0 += coefs[] * x[]
     R19:18 += VRMPYH(R9:8,R15:14)     // sum2 += coefs[] * x[]
     R10 = R9                          // R10 = x5|4
     R12 = MEMW(R3++#4)                // R12 = x7|x6
   }
   { R11:10 = VALIGNB(R11:10,R7:6,#2)  // R11:10 = x4|x3|x2|x1
     R13:12 = VALIGNB(R13:12,R9:8,#2)  // R13:12 = x6|x5|x4|x3
     R7:6 = MEMD(R0++#8)               // [2] R7:6 = x3|x2|x1|x0
     R9 = MEMW(R0+#8)                  // [2] R9 = x5|x4
   }
   { R21:20 += VRMPYH(R11:10,R15:14)   // sum1 += coefs[] * x[]
     R23:22 += VRMPYH(R13:12,R15:14)   // sum3 += coefs[] * x[]
     R3 = ADD(R0,#4)                   // [2]
     R1 = R28                          // [2]
   }
   {
     R17:16 = ADD(R25:24,R17:16):SAT   // sum0 += 0x4000
     R21:20 = ADD(R25:24,R21:20):SAT   // sum1 += 0x4000
   }
   {
     R19:18 = ADD(R25:24,R19:18):SAT   // sum2 += 0x4000
     R23:22 = ADD(R25:24,R23:22):SAT   // sum3 += 0x4000
   }
   { R17:16 = ASR(R17:16,R4)           // sum0 >>= Qshift
     R21:20 = ASR(R21:20,R4)           // sum1 >>= Qshift
     R8 = R7                           // [2] R9:8 = x5|x4|x3|x2
     R10 = R9                          // [2] R10 = x5|4
   }
   { R19:18 = ASR(R19:18,R4)           // sum2 >>= Qshift
     R23:22 = ASR(R23:22,R4)           // sum3 >>= Qshift
     R15:14 = MEMD(R1++#8)             // [2] load coefficients
   }
   { R21:20 = COMBINE(R20,R16)         // pack to sum1:sum0
     R23:22 = COMBINE(R22,R18)         // pack to sum3:sum2
     R17:16 = VRMPYH(R7:6,R15:14)      // [2] sum0 += coefs[] * x[]
     R19:18 = VRMPYH(R9:8,R15:14)
   }
   { R20 = VSATWH(R21:20)              // saturate to 16-bit
     R21 = VSATWH(R23:22)              // saturate to 16-bit
     P3 = CMP.EQ(R2,R2)                // set P3 (for conditional write)
     R12 = MEMW(R3++#4)                //[2] R12 = x7|x6
   }:endloop1

   {
     MEMD(R5) = R21:20
   }
.ltuning_filter_fir_END_RND:
   {
     R17:16 = MEMD(R29 + #0)
     R19:18 = MEMD(R29 + #8)
   }
   {
     R21:20 = MEMD(R29 + #16)
     R23:22 = MEMD(R29 + #24)
   }
   {
     R25:24 = MEMD(R29 + #32)
     DEALLOC_RETURN
   }
   .size tuning_filter_fir_rnd, .-tuning_filter_fir_rnd
   
#endif