#if ((defined __hexagon__) || (defined __qdsp6__))

#include "asm_macros.h"

/*======================================================================*/
/*  FUNCTION  exp_compute                                               */
/*======================================================================*/
/*  DESCRIPTIONS                                                        */
/*  ============                                                        */
/*   Implements a normalized cubic approximation for computing exp      */
/*       of a 32-bit number                                             */
/*                                                                      */
/*  INPUTS                                                              */
/*  ============                                                        */
/*  R0:  Input 32-bit power value                                       */
/*  R1:  Integer part of Q-factor for the input                         */
/*  R2:  Integer part of Q-factor for the exp output                    */
/*  R3:  Constant to convert from 2^(x) to exp(x)                       */
/*  OUTPUTS                                                             */
/*  R0: Output 32-bit                                                   */
/*  Registers Affected: R6-18                                           */
/*  Hardware Loops affected : None                                      */
/*  Assumptions:None                                                    */
/* Stack Memory Frame Allocated (in Bytes): 0                           */
/*  Cycle Count: 17                                                     */
/* **********************************************************************/
/* Exponential is too expensive, so it is approximated as a cubic       */
/* third order polynomial. The input number is scaled down to the range */
/* [0,1] and exp approximation is applied and the output is scaled back */
/* to the original range.                                               */
/*    expL32 = (((C0*a*x + C1)*a*x + C2)*a*x + C3)*2.^n.                */
/*  a = log2(e) = 1.4427 Q2.13 - can be modified to compute power to any*/
/* base C0 = 0.0778 Q15, C1 = 0.2258 Q2.29, C2 = 0.6962 Q2.29, C3 = 1   */
/* Q2.29 mean squared error ratio is less than -50dB for all realnumbers*/
/*======================================================================*/

   .p2align 4
   .globl exp_compute
   .type exp_compute, @function
exp_compute:   
     
    { 
      R9 = #11                                  // R9 to be used for comparing in min
      SP = ADD(SP, #-16);              /*allocate stack ptr*/
      MEMD(SP+#-16) = R17:16;            /*save R17,R16      */
      R7 = NORMAMT(R0)                          // x_norm = norm_l(xInL32);            
    }
    {
      R7 = MIN(R7,R9)                           // x_norm = s16_min_s16_s16(x_norm, 11);
      R9 = #1  
    }
    {
      R11 = ADD(R7,#-2)
      R17.H = #0x073A
      MEMD(SP+#8) = R19:18;                     /*save r19,r18      */
      R8 = ADD(R1,#-29)                         // iqInL16-29
 
    }
   
    {
     R11 = ASL(R0,R11):SAT
     R17.L = #0x5F9A
     R12 = ADD(R2,#-2)                          //out_norm = iqOutL16-2;
     R19 = SUB(R8,R7)                           //iqInL16-29-x_norm
    }
    
    {
     R11 = MPY(R11,R3.L):<<1:RND:SAT            //xInL32=Mpy_32_16_round(xInL32,expConstL16Q13);
     R8  = MPYI(R19,#-1)                        //x_norm+29-iqInL16
     R19  = ADD(R19,#18)
    }
    {
     R13 = ASL(R11,R19):SAT                     //x_floor = S_saturate(L_shl(xInL32, iqInL16-29-x_norm));
     R11 = ASL(R11,#2):SAT
    }
    {
     R13 = ASRH(R13)
    }
    {
     R14 = ASL(R13,R8):SAT                      //x_shift32 = L_shl((int32 )x_floor, x_norm+29-iqInL16);
     R7  = SUB(R1,R7)                           //iqInL16-x_norm
     R12 = SUB(R12,R13)                         //out_norm = (iqOutL16-2) - x_floor;
    }
    
    {
     R11 = SUB(R11,R14):SAT                     //xInL32 = L_sub(xInL32,x_shift32);
     R18 = ADD(R12,#-1)                         //out_norm-1 
    }
    {
     R11 = ASL(R11,R7):SAT                      //xInL32 = L_shl(xInL32, iqInL16-x_norm)
     R7  = #0x09F6
    
    }
    {
     R17:16 + = VMPYWEH(R11:10,R7:6):<<1:RND:SAT     
     R10.H =#0x1646
     R11 + = ASL(R9,#15)
     R19 = SUB(R19,R2)                          //(32-exp_norm)-iqOutL16
    }
    {
     R13  = MPY(R17,R11.H):<<1:RND:SAT
     R10.L = #0xFEAF 
     R9 = ASL(R9,R18):sat                           //rnd32 = L_shl(((int32 )0x0001),out_norm-1);
    }
    
    {
     R10 + = ASL(R13,#2)
     R13.H =#0x2000
   
    }
    {
     R10 = MPY(R10,R11.H):<<1:RND:SAT
     R13.L = #0x0000
    }
    {
     R13 + = ASL(R10,#2)  
     R12  = MPYI(R12,#-1)
    }
    {
      R13 = ADD(R13,R9):SAT                     //logL32 = L_add(logL32, rnd32);
    }
    {
      R13 = ASL(R13,R12):SAT                    //logL32 = L_shl(logL32, -out_norm);
      R17:16 = MEMD(SP+#0);
      R19:18 = MEMD(SP+#8);  
    }

    .falign
    .Lexp_compute_exit: 
    {
//       R17:16 = MEMD(SP+#0);
//       R19:18 = MEMD(SP+#8);    
    }
    { 
      R0=R13
      SP = ADD(SP,#16);       
      NOP
      JUMPR R31 //return
    }

/*======================================================================*/
/*  FUNCTION  sqrt_compute                                              */
/*======================================================================*/
/*  DESCRIPTIONS                                                        */
/*  ============                                                        */
/*  Implements a cubic spline approximation for computing square-root of*/
/*  of a 32-bit fractional number                                       */
/*  INPUTS                                                              */
/*  ============                                                        */
/*  R0:  Input 32-bit power value                                       */
/*  R1:  Integer part of Q-factor for the input power                   */
/*  R2:  Integer part of Q-factor for the square-root output            */
/*  R3:  CWord2x16 Wt2 -- twiddle factors for last stage                */
/*  R4:  Word16 *output -- pointer to output array                      */
/*  OUTPUTS                                                             */
/*  R0: Output 32-bit                                                   */
/*  Registers Affected: R6-17                                           */
/*  Hardware Loops affected : None                                      */
/*  Assumptions:None                                                    */
/* Stack Memory Frame Allocated (in Bytes): 0                           */
/*  Cycle Count: 15                                                     */
/*            Author         Date          Comments                     */ 
/*           --------       ------        ----------                    */
/*         P.ChandraMouli  2009/Oct/12     Creation                     */   
/*======================================================================*/
/* **********************************************************************/
/* Square root too expensive, so the square root is approximated        */
/* with third order polynomial computed by Song Wang.The input power is */
/* normalized to be in the range 1 <= cal_in < 4 and the approximate    */
/* square-root is computed and then it is normalized back appropriately.*/
/*                                                                      */
/*   sqrtL32 = ((S0*pwrInL32 + S1)*pwrInL32 + S2)*pwrInL32 + S3.        */
/*   S0 = 0.0089 Q15, S1 = -0.1029 Q29, S2 = 0.6605 Q29, S3 = 0.4334 Q29*/
/*   sqrt error ratio is less than -50 dB for all positive real numbers */
/*======================================================================*/
/*                                                                      */
/*======================================================================*/

   .p2align 4
   .globl sqrt_compute
   .type sqrt_compute, @function
sqrt_compute:   
     __saveonentry_8
     
    { P0 = CMP.EQ(R0,#0)
      IF (P0.new) JUMP:nt .Lsqrt_compute_exit   // if (inp==0),
      R13 = #0 
      R7 = NORMAMT(R0)                          // [Compute norm of the input power]
    }
   
    {
      R8 = ADD(R1,#-1)                          // iqInL16-1
      R9 = #0                                     
      R17.H = #0xFCB4
           
    }
    
    {
     R8 = SUB(R8,R7)                            // mant_norm:Compute integer part of log2 norm of the input power
     R11= ADD(R1,#-2)
     R17.L = #0x5476
    }
    {
     R8 = ASR(R8,#1)
     R9 = SETBIT(R9,#15)
    }
    {
     R11 - = ASL(R8,#1)                         // pwr_offset = (iqInL16-2) - mant_norm
     R8   = ADD(R8,R8)                          // mant_norm = (mant_norm >>1)<<1;   
     R2   = ADD(R2,#-3)                         // iqOutL16-3
    }
    {
     R11 = ASL(R0,R11):SAT                      //pwrInL32 = L_shl(pwrInL32, pwr_offset);
     R8  = ASR(R8,#1)                           //mant_norm = (mant_norm >>1)
     R7  = #0x0125
    }
    {
     R11 = ADD(R11,R9):SAT
     R8  = SUB(R2,R8)                           //out_norm-1 = ((iqOutL16-2) - (mant_norm >>1))-1;
     R15 = R11
     R9  = #1
    }
    
    {
     R17:16 + = VMPYWEH(R15:14,R7:6):<<1:RND:SAT     
     R9  = ASL(R9,R8):SAT                           //rnd32 = L_shl(((int32 )0x0001),out_norm-1);
     R8  = ADD(R8,#1)  
     R10.H =#0x1523   
    }
    {
     R13  = MPY(R17,R11.H):<<1:RND:SAT
     R10.L = #0x9E51 
     
    }
    
    {
     R10 + = ASL(R13,#2)
     R13.H =#0x0DDE
    }
    {
     R10 = MPY(R10,R11.H):<<1:RND:SAT
     R13.L = #0xCD39
    }
    {
     R13 + = ASL(R10,#2)  
     R8  = MPYI(R8,#-1)
    }
    {
      R13 = ADD(R13,R9):SAT
    }
    {
      R13 = ASL(R13,R8):SAT
    }
    
.falign
.Lsqrt_compute_exit: 
    { 
          R0=R13
          NOP       
          NOP
          //JUMPR R31 //return
    }
#ifndef FEATURE_POPQUEUE_CONFLICT
    __restoreonexit_8
#else
{
  R17:16 = MEMD(SP++#8);   /*restore r16,r17 & dealloc stack */
}
{
  JUMPR R31;       /*return to caller    */
}
#endif

/*======================================================================*/
/*  FUNCTION  audio_log_compute                                               */
/*======================================================================*/
/*  DESCRIPTIONS                                                        */
/*  ============                                                        */
/*   Implements a normalized cubic approximation for computing log10    */
/*   of a 32-bit number                                                 */
/*                                                                      */
/*  INPUTS                                                              */
/*  ============                                                        */
/*  R0:  Input 32-bit power value                                       */
/*  R1:  Integer part of Q-factor for the input power                   */
/*  R2:  Integer part of Q-factor for the square-root output            */
/*  R3:  Constant to convert from log2 to log_base                      */
/*  OUTPUTS                                                             */
/*  R0: Output 32-bit                                                   */
/*  Registers Affected: R6-19                                           */
/*  Hardware Loops affected : None                                      */
/*  Assumptions:None                                                    */
/*  Stack Memory Frame Allocated (in Bytes): 0                          */
/*  Cycle Count: 15                                                     */
/*            Author         Date          Comments                     */ 
/*           --------       ------        ----------                    */
/*         P.ChandraMouli  2009/Oct/12     Creation                     */   
/* **********************************************************************/
/* Logarithm is too expensive, so it is approximated as a cubic         */
/* third order polynomial. The input power is expressed in mantissa and */
/* exponent with mantissa in the range [1,2] and log2 approximation is  */
/* applied only to mantissa and log2 of the input power is computed.    */
/*    logL32 = ((C0*mant + C1)*mant + C2)*mant + C3 + exponent.         */
/* C0 = 0.1840 Q15, C1 = -1.1688 Q2.29, C2 = 3.2187 Q2.29, C3 = -2.2340 */
/* Q2.29 mean squared error ratio is less than -50 dB for all positive  */
/* real numbers.Finally, log to any base is computed from log2 using the*/
/* logConst value                                                       */
/*======================================================================*/
   .p2align 4
   .globl audio_log_compute
   .type audio_log_compute, @function
audio_log_compute: 
    { SP = ADD(SP, #-16);              /*allocate stack ptr*/
      MEMD(SP+#-16) = R17:16;            /*save R17,R16      */
      R9 = #1
    } 
     
    { P0 = CMP.GT(R0,#0)
      MEMD(SP+#8) = R19:18;                     /*save r19,r18      */
      R17.H = #0xDA99
      R7 = NORMAMT(R0)                          // [Compute norm of the input power]
    }
   
    {
      R8 = ADD(R1,#-1)                          // iqInL16-1
      IF !P0 JUMP .Llog_compute_exit            // if (inp==0),
      R13 = ASL(R9,#31)
      R11 = ADD(R7,#-1)                         //power offset
           
    }
    
    {
     R8 = SUB(R8,R7)                            // mant_norm:
     R17.L = #0x7212
     R12 = ADD(R2,#-3)                          //out_norm = iqOutL16-3;
     R18 = ADD(R2,#-4)
    }
    
    {
     R11 = ASL(R0,R11):SAT                      //pwrInL32 = L_shl(pwrInL32, pwr_offset);
     R19 = NORMAMT(R8)                          //exp_norm = norm_l(out_exp32);
     R7  = #0x178E
    }
    {
     R11 + = ASL(R9,#15)
     R15 = R11
     R8 = ASL(R8,R19):SAT                       //out_exp32 = L_shl(out_exp32, exp_norm);
     R19 = SUB(#32,R19)                         //(32-exp_norm)
    }
    {
     R17:16 + = VMPYWEH(R15:14,R7:6):<<1:RND:SAT     
     R10.H =#0x66FF
     R8  =MPY(R8,R3.L):<<1:RND:SAT              //out_exp32 = Mpy_32_16_round(out_exp32,logConstL16Q14);
     R19 = SUB(R19,R2)                          //(32-exp_norm)-iqOutL16
    }
    {
     R13  = MPY(R17,R11.H):<<1:RND:SAT
     R10.L = #0xB300 
     R9 = ASL(R9,R18):SAT                           //rnd32 = L_shl(((int32 )0x0001),out_norm-1);
    }
    
    {
     R10 + = ASL(R13,#2)
     R13.H =#0xB883
     R8 = ASL(R8,R19):SAT                       //out_exp32 = L_shl(out_exp32, (32-exp_norm)-iqOutL16);
    }
    {
     R10 = MPY(R10,R11.H):<<1:RND:SAT
     R13.L = #0x5AEE
    }
    {
     R13 + = ASL(R10,#2)  
     R12  = MPYI(R12,#-1)
    }
    {
     R13 = MPY(R13,R3.L):<<1:RND:SAT            //logL32 = Mpy_32_16_round(logL32,logConstL16Q14);
    }
    {
      R13 = ADD(R13,R9):SAT                     //logL32 = L_add(logL32, rnd32);
    }
    {
      R13 = ASL(R13,R12):SAT                    //logL32 = L_shl(logL32, -out_norm);
    }
    {
      R13 = ADD(R13,R8):SAT                     //logL32 = L_add(logL32, out_exp32);
    }
    
.falign
.Llog_compute_exit: 
    {
       R17:16 = MEMD(SP+#0);
       R19:18 = MEMD(SP+#8);
    
    }
    { 
      R0=R13
      SP = ADD(SP,#16);       
      NOP
      JUMPR R31 //return
    }
#endif