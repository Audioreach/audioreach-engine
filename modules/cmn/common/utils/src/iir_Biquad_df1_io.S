#if  defined(__qdsp6__)
#include "asm_macros.h"
/************************************************************************/
/*                                                                      */
/*                Biquad IIR io not in place                            */
/*                ===================                                   */
/*   R0: input samples xin in q15                                       */
/*   R1: pointer to Coeff buffer 8byte aligned a1 a2 -b1-1 -b2 in q13   */
/*   R2: pointer to states 8byte aligned x(n-1) x(n-2) y(n-1) y(n-2) q15*/
/*   R3: a0  coef in q13  int32                                         */
/*   R4: number of samples int32                                        */
/*   R5:  address of filter->yL32  int32                                */ 
/*   Output pointer in Stack                                            */
/************************************************************************/
	.text
	.p2align 5
	.globl IIR_Biquad_io
	.type	IIR_Biquad_io, @function
IIR_Biquad_io:
  __saveonentry_8                       //save R16, R17 to stack
  { 
   P0 = CMP.GT(R4,#0)			             //P0 = (nsampe >0 )
   R16 = MEMH(R0++#2)			             //load xin  
   R10 = MEMW(R5)                       //filter->yL32 is y[n-1] in 32 bit
   IF !P0.new JUMP:nt .lIIR_Biquad_io_END
  }
  { 
   R10 += mpy(R16.l,R3.l):sat          // a0 * xin	 in q28                 
   R7:6 = MEMD(R2)                     // R6 = x(n-2) x(n-1) R7 = y(n-2) y(n-1)
   R9:8 = MEMD(R1)                     //load coef R8 = a2 a1 R9 = -b2 -b1-1    
   R4 = ADD(R4,#-1)                    //samples = samples - 1s
  }
  {
   P0 = CMP.GT(R4,#0)			        //P0 = (nsampe >1 )
   IF !P0.new JUMP:nt .lIIR_Biquad_io_single_sample
   R17 = MEMW(R29+#SAVEONENTRY_8_OFFSET)
  }
  {
    LOOP0(.lIIR_Biquad_io_loops,R4) 
  }
	.falign
.lIIR_Biquad_io_loops:
  { 
   R11:10 += VRMPYH(R7:6,R9:8)    	   // R10 = a0 xin - b2*y0 - b1*y1+ a2*x2 + a1*x1 in Q28
   R6 = COMBINE(R6.l, R16.l) 				   // x(n-2) = x(n-1) x(n-1)= xin in q15                      
   R16 = MEMH(R0++#2)                  //load xin 
  }
  {
   R13 = ASL(R10, #3):sat             //Q28 to Q31
   R11 = #0
   MEMW(R5) = R10                     //saving y[n-1] in Q28
  }
  {
   R7 = COMBINE(R7.l, R13.h)          //y(n-2)=y(n-1) y(n-1)=y(n)
   MEMH(R17++#0x2) = R13.h            //Saving output
   R10 += mpy(R16.l,R3.l):sat         //a0 * xin	 in q28  
  }:endloop0
.lIIR_Biquad_io_single_sample:
  { 
   R11:10 += VRMPYH(R7:6,R9:8)    	   // R10 = a0 xin - b2*y0 - b1*y1+ a2*x2 + a1*x1 in Q28
   R6 = COMBINE(R6.l, R16.l) 				   // x(n-2) = x(n-1) x(n-1)= xin in q15                      
  }
  {
   R13 = ASL(R10, #3):sat             //Q28 to Q31
   MEMW(R5) = R10                     //saving y[n-1] in Q28
  }
  {
   R7 = COMBINE(R7.l, R13.h)          //y(n-2)=y(n-1) y(n-1)=y(n)
   MEMH(R17++#0x2) = R13.h            //Saving last sample output
  }
  {
   MEMD(R2) = R7:6                    //save x(n-1) x(n-2) y(n-1) y(n-2)
  }
.lIIR_Biquad_io_END:
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
	.size	IIR_Biquad_io, .-IIR_Biquad_io
#endif
	