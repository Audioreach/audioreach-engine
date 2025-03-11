/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ***************************************************************
 * FILE NAME:   dsp_libs.S                                      
 * DESCRIPTION: Library of math functions 
 * PROGRAMMERS: MZ
 ***************************************************************
 */


/* ------------------------------------------------------------- */
/*  INCLUDES                                                     */
/* ------------------------------------------------------------- */

//#include "asm_macros.h"
#include "adsp_redef.h"



/*======================================================================*/
/*  FUNCTION  audio_divide_sp                                           */
/*======================================================================*/
/*  DESCRIPTIONS                                                        */
/*  ============                                                        */
/*  Single precision divide                                             */
/*  INPUTS                                                              */
/*  ============                                                        */
/*  R0: int32 Lnum                                                      */
/*  R1: int32 Ldenom                                                    */
/*  R2: Word16 Qadj                                                     */
/*  OUTPUTS                                                             */
/*  R0: Output int32 Lquotient in Q(31-Qadj)                            */
/*  Registers Affected:                                                 */
/*  Hardware Loops affected :                                           */
/*  Assumptions:None                                                    */
/* Stack Memory Frame Allocated (in Bytes): 0                           */
/*  Cycle Count:                                                        */
/*            Author         Date          Comments                     */
/*           --------       ------        ----------                    */
/*           danielp       12/13/2010      Creation                     */
/*           katitkar      04/19/2011     Reduction of inner loop to    */
/*                                        3 cycles per changes suggested*/
/*					  by cpoliset                   */
/*           katitkar      04/22/2011     Change inputs to 32 bits      */
/*======================================================================*/
/* **********************************************************************/
/*======================================================================*/
/*                                                                      */
/*======================================================================*/
   .p2align 4
   .globl audio_divide_sp
   .type audio_divide_sp, @function
audio_divide_sp:
   {
      R14 = SUB(#0, R1);                // -Ldenom
      R9   = NORMAMT(R0);                  // norm_l(Lnum)
      P3 = CMP.GT(R1, #-1);               // test if Ldenom negative
      R4 = ADD(R2,#-1);                  // Qadj - 1
   }
   {
      R1 = MUX(P3,R1,R14);       // if( Ldenom < 0) Ldenom = -Ldenom
      R4 = ADD(R9,R4);                   // Qadj = nnum + (Qadj - 1)
      R0 = ASL(R0,R9);               // Lnum <<= n1
   }
   {
      P1   = CMP.EQ(R1,#0);          // Ldenom == 0 test
      R8   = NORMAMT(R1);                  // norm_l(Ldenom)
      P2 = CMP.GT(R0, #-1);                // test if Lnum negative
      R5 = SUB(#0, R0);             //negative num, to use for multiplication in case of -ve denom
   }
   {
      if( P1) JUMP .Ldivide_dp_fix_denom0
      R14 = #0;
      R4   = SUB(R8,R4);                 // Qadj = ndenom - Qadj
      R1 = ASL(R1,R8);               // Ldenom << ndenom
   }
   {
      LOOP0(.Ldivide_dp_loop,#5)
#if (__QDSP6_ARCH__ > 3)
      R15 = ##0x40000000;        // setup R15:14 = 2.0
#else
      R15.L = #0;
#endif
      R0 = MUX(P3, R0, R5);
   }
   {
#if (__QDSP6_ARCH__ <= 3)
      R15.H = #0x4000;
#endif
      R12  = ##(0x55555555);       // initial guess
	}

.falign
.Ldivide_dp_loop:
   {
      R15:14 -= MPY(R12,R1);                  // (Ltemp*(int32) Ldenom) >> 32...multiply and use upper (implicit shift by 32)
   }
   {
      R13:12 = MPY(R12,R15);              // Ltemp*Lacc
      R15.L = #0;
   }
   {
      R15.H = #0x4000;
      R13:12 = ASR(R13:12,#29);           // finish l_mult_s32xs32_shift(Ltemp3, (int32)Lacc, 3);
      R14 = #0;
   }:endloop0

   {
      R13:12 = MPY(R12,R0);               // Ltemp3*(int32)Lnum (num negated if den<0)
      R4 = ADD(R4,#-31);                  // combine ASL of -31 and Qadj
      P0 = CMP.GT(R4, #-33);
      if (!P0.new) JUMP:nt .Ldivide_dp_large_shift;
   }
   {
      R13:12 = ASL(R13:12,R4);
   }
   {
      R0 = SAT(R13:12);
      JUMPR R31;
   }
.falign
.Ldivide_dp_fix_denom0:
   {
      R0 = ##(0x80000000);
      R1 = ##(0x7FFFFFFF);
   }
   {
      R12  = ##(0x55555555);       // initial guess   
      R0 = MUX(P2,R1,R0);                 // max positive value if Lnum >= 0, max negative value otherwise
      JUMPR R31;
   }
.falign
.Ldivide_dp_large_shift:
   {
      R9:8 = COMBINE(#0, #0);
   }
   {
      P0 = CMP.GT(R9:8, R13:12);
   }
   {
      R0 = MUX(P0, #-1, #0);
      JUMPR R31;
   }
.size audio_divide_sp, .-audio_divide_sp
