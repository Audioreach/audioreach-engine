#if  defined(__qdsp6__)
#include "asm_macros.h"
/************************************************************************/
/*                                                                      */
/*                Biquad IIR                                            */
/*                ===================                                   */
/*   R0: input samples xin in q15                                       */
/*   R1: pointer to Coeff buffer 8byte aligned a1 a2 -b1-1 -b2 in q13   */
/*   R2: pointer to states 8byte aligned x(n-1) x(n-2) y(n-1) y(n-2) q15*/
/*   R3: a0  coef in q13  int32                                         */
/*   R4: number of samples int32                                        */
/*   R5:  address of filter->yL32  int32                                */ 
/*                                                                      */
/************************************************************************/
	.text
	.p2align 5
	.globl IIR_Biquad
	.type	IIR_Biquad, @function
IIR_Biquad:
   __saveonentry_8                      //save R16, R17 to stack
   { 
    P0 = CMP.GT(R4,#0)		            //P0 = (nsampe >0 )
	R16 = MEMH(R0)		                //load xin 
    R10 = MEMW(R5)                      //filter->yL32 is y[n-1] in 32 bit
    IF !P0.new JUMP:nt .lIIR_Biquad_END  
   }
   { 
    R10 += mpy(R16.l,R3.l):sat          // a0 * xin	 in q28                 
	R7:6 = MEMD(R2)                     // R6 = x(n-2) x(n-1) R7 = y(n-2) y(n-1)
	R9:8 = MEMD(R1)                     //load coef R8 = a2 a1 R9 = -b2 -b1-1  
	R4 = ADD(R4,#-1)                    // samples = samples - 1
   }
   {
    P0 = CMP.GT(R4,#0)			        //P0 = (nsampe >1 )
    IF !P0.new JUMP:nt .lIIR_Biquad_single_sample
   }   
   {
    LOOP0(.lIIR_Biquad_loops,R4) 
   }
	.falign
.lIIR_Biquad_loops:
  { 
   R11:10 += VRMPYH(R7:6,R9:8)    	    // R10 = a0 xin - b2*y0 - b1*y1+ a2*x2 + a1*x1 in Q28
   R6 = COMBINE(R6.l, R16.l) 				    // x(n-2) = x(n-1) x(n-1)= xin in q15                      
   R16 = MEMH(R0+#2)                    //load xin 
  }
  {
   R13 = ASL(R10, #3):sat               // Q28 to Q31
   R11 = #0
   MEMW(R5) = R10                       //saving y[n-1] in Q28
  }
  {
   R7 = COMBINE(R7.l, R13.h)           //y(n-2)=y(n-1) y(n-1)=y(n)
   MEMH(R0++#0x2) = R13.h              //Saving output
   R10 += mpy(R16.l,R3.l):sat          //a0 * xin	 in q28  
  }:endloop0
.lIIR_Biquad_single_sample:  
  { 
   R11:10 += VRMPYH(R7:6,R9:8)    	    // R10 = a0 xin - b2*y0 - b1*y1+ a2*x2 + a1*x1 in Q28
   R6 = COMBINE(R6.l, R16.l) 				    // x(n-2) = x(n-1) x(n-1)= xin in q15                      
  }
  {
   R13 = ASL(R10, #3):sat               // Q28 to Q31
   MEMW(R5) = R10                       //saving y[n-1] in Q28
  }
  {
   R7 = COMBINE(R7.l, R13.h)           //y(n-2)=y(n-1) y(n-1)=y(n)
   MEMH(R0++#0x2) = R13.h              //Saving output
  }
  {
	 MEMD(R2) = R7:6                   // save x(n-1) x(n-2) y(n-1) y(n-2) 
  }
.lIIR_Biquad_END:
#ifndef FEATURE_POPQUEUE_CONFLICT
	__restoreonexit_8                 //restore R16, R17 from stack
#else
{
  R17:16 = MEMD(SP++#8);   /*restore r16,r17 & dealloc stack */
}
{
  JUMPR R31;       /*return to caller    */
}
#endif
	.size	IIR_Biquad, .-IIR_Biquad
	
	
/*****************************************************************************/
/* Function:         IIR_one_pole                                            */
/*---------------------------------------------------------------------------*/
/* Description:      Single pole IIR filter                                  */
/*---------------------------------------------------------------------------*/
/* C Prototype:      void IIR_one_pole ( int16 *inplaceBuf, int32 *yL40,     */
/*                   int32 c0L16Q14, int32 c1MinusOneL16Q14, int16 samples)  */
/*---------------------------------------------------------------------------*/
/* Reg Inputs:       R0 : input samples inplaceBuf in q15 (i/o)              */
/*                   R1 : address of filter->yL32         (i)                */
/*                   R2 : c0L16Q14                        (i)                */
/*                   R3 : c1MinusOneL16Q14                (i)                */
/*                   R4 : number of samples               (i)                */
/*---------------------------------------------------------------------------*/
/* Stack Arguments:      None                                                */
/*---------------------------------------------------------------------------*/
/* Register Outputs:      R0                                                 */
/*---------------------------------------------------------------------------*/
/* Registers Affected:   R0-R8                                              */
/*---------------------------------------------------------------------------*/
/* Hardware Loops Usage: LOOP0,                                              */
/*---------------------------------------------------------------------------*/
/* Stack Memory Usage (in Bytes):   None                                     */
/*---------------------------------------------------------------------------*/
/* Thread Cycles:    None                                                    */
/*---------------------------------------------------------------------------*/
/* Notes:                                                                    */
/*      1. code takes 2 cycles/sample                                        */
/*      2. Inplace computation                                               */
/*      3. Filter coefficients are multiplied by 2 before actual filter      */
/*         operations. This is different from c-code. And this is bit-exact  */
/*         specific implementation of QEnsemble. If this code is used for    */
/*         other applications please check before using it                   */
/*****************************************************************************/	

	.p2align 5
	.globl IIR_one_pole
	.type  IIR_one_pole, @function
IIR_one_pole:
   {
    R2 = ASL(R2,#1)                     //c0L16Q14 = c0L16Q14 << 1
    P0 = CMP.GT(R4,#0)			        //P0 = (nsampe >0 )
    IF !P0.new JUMP:nt .lIIR_one_pole_END	
   }
   { 
   	R3 = ASL(R3,#1)                     //c1MinusOneL16Q14 = c1MinusOneL16Q14 << 1
    R4 = ADD(R4,#-1);                   //samples = samples -1	
    R5 = MEMW(R1)                       //filter->yL32 is y[n-1] in 32 bit (tempL40)
	R6 = MEMH(R0)                       //load sample (*inplaceBuf)
   }
   {
    R5+= MPY(R3.l,R5.h):<<1:sat         // tempL40 = (tempL40 + (c1MinusOneL16Q14 * yL16) << 1)
	R8 = MPY(R2.l,R6.l):<<1:sat         //(c0L16Q14 * (*inplaceBuf)) << 1
   }
   {
    P0 = CMP.GT(R4,#0)			        //P0 = (nsampe > 1 )
    IF !P0.new JUMP:nt .lIIR_one_pole_single_sample   
   }
   {
     LOOP0(.lIIR_one_pole_loop,R4)      //configure loop
   }
	.falign
.lIIR_one_pole_loop:
  {
   R5 = ADD(R5,R8):sat                  // tempL40 + (c0L16Q14 * (*inplaceBuf)) << 2
   R6 = MEMH(R0+#2)                     // load next sample   
  }
  {
   R5+= MPY(R3.l,R5.h):<<1:sat          // tempL40 = (tempL40 + (c1MinusOneL16Q14 * yL16) << 1)
   R8 = MPY(R2.l,R6.l):<<1:sat          //(c0L16Q14 * (*inplaceBuf)) << 1
   MEMH(R0++#2)=R5.h                    // store sample
  }:endloop0
.lIIR_one_pole_single_sample:  
  {
   R5 = ADD(R5,R8):sat                  // tempL40 + (c0L16Q14 * (*inplaceBuf)) << 2
  }
  {
   MEMH(R0)=R5.h                        // store last sample
  }
  {
   MEMW(R1)=R5;                         // store filter->yL32
  }
.lIIR_one_pole_END:
   JUMPR R31 
   .size	IIR_one_pole, .-IIR_one_pole 
#endif
