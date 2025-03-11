#include "apiir_df1_function_defines.h"
#ifdef IIR_DF1_32_AP_ASM

/*======================================================================*/
/*  FUNCTION  iirDF1_32_ap()                                            */
/*======================================================================*/
/*  DESCRIPTIONS                                                        */
/*  ============                                                        */
/*   This function calculates the response of a two-pole allpass        */
/*   IIR filter that is implemented in DF1 form                         */
/*   numcoefs->b(0), only 2 coefs are stored                            */
/*   To use the following function, it should be ensured that the       */
/*   Q-factor of den and num coefs should be <= Q27, in order to avoid  */
/*   overflow                                                           */
/*                                                                      */
/*  INPUTS                                                              */
/*  ============                                                        */
/*  R0 -> input buffer.                                                 */
/*  R1 -> output buffer.                                                */
/*  R2 = no. of samples.                                                */
/*  R3 -> numerator coefficients.                                       */
/*  R4 -> memory of the filter.                                         */
/*  R5 -> Q-factor.                                                     */
/*  OUTPUT                                                              */
/*  ============                                                        */
/*  Hardware Loops affected : Loop0                                     */
/*  Cycle Count: 6*(Number of Samples - 1) + 18                         */
/*                                                             	        */
/*======================================================================*/
/*   repeat the loop for every sample                                   */
/*   in allpass biquad, a1 = b1, a2 = b0, b2 = 1                        */
/*   equations are ff= b0*x+b1*w1+b2*w2                                 */ 
/*          i.e. ff= b0*x+b1*w1+w2                                      */
/*                         w2=w1                                        */
/*                         w1=x                                         */
/*               fb= a1*w3+a2*w4                                        */ 
/*          i.e. fb= b1*w3+b0*w4                                        */
/*                         w4=w3                                        */
/*                         y = ff - fb                                  */
/*                         w3=y                                         */
/*======================================================================*/
.p2align 4
.globl	iirDF1_32_ap
.type	iirDF1_32_ap,@function
.falign	
iirDF1_32_ap:
	{
		p0 = cmp.gt(r2, #0)
		if (p0.new) r12 = #0
		memd(r29+#-16) = r17:16
		allocframe(#16)
	}
	{
		if (!p0) jump  .L_sample0                   // if no. of samples is zero jump to L_sample0
		memd(r29+#0) = r19:18
	}
	{
		r7 = add(r5, #-1)						 //	(qfactor - 1)							
		p0 = cmp.gt(r5, #0)                      // compare qfactor with 0
		r6 = memw(r3 + #4)                       // load b1 in r6
	}
	{
		r7 = lsl(#1, r7)                         // r7 = 1<<(qfactor - 1)
		r18 = add(r2, #-1)                       // r18 = samples -1
		r3 = memw(r3 + #0)                       // load b0 in r3
		r19 = memw(r4 + #8)                      // load w3 in r19
	}
	{
		r17:16 = mpy(r6, r19)                    // calculate w3*b1 and load into r17:16
		r13 = asr(r7, #31)                     
		r28 = memw(r4 + #0)                      // load w1
		r14 = memw(r4 + #4)                      // load w2
	}
	{
		r9 = mux(p0, r13, r12)                   // if qfactor = 0   (p5 = 0)
		r8 = mux(p0, r7, r12)                    // if qfactor >= 1  (p5 = r7 = 1<<(qfactor - 1))
		p0 = cmp.gtu(r2, #1)
		r13 = memw(r4 + #12)                     // load w4 in r13
	}
	{
		r17:16 += mpy(r3, r13)                   // calculate fb = w3*b1 + w4*b0
		r15 = asr(r14, #31)                      // load sign extended bits in r15
		r7 = sub(#0, r5)                         // r7 = -qfactor
		r12 = memw(r0++#4)                       // load input x in r12
	}
	{
		loop0(.L_iirDF1, r18)                     // hardware loop0 initialized 
		r11:10 = mpy(r3, r12)                    // calculate b0*x and load into r11:10
		memw(r4+#4) = r28                        // store w1 at w2 location
	}
	{
		r11:10 += mpy(r6, r28)                   // r11:10 = b0*x + b1*w1
	}
	{
		r11:10 += asl(r15:14, r5)                // calculate ff = b0*x + b1*w1 + w2
		r28 = r12                                 // w1 = x
		memw(r4+#12) = r19                       // store w3 at w4 location
	}
	{
		r13:12 = sub(r11:10, r17:16)             // y = r13:12 = ff - fb
	}
	{
		r13:12 = add(r13:12, r9:8)               // y = y + p5
	}
	{
		r13:12 = asl(r13:12, r7)                 // y = y << -qfactor
		r14 = memw(r4 + #4)                      // load w2 in r14
	}
	{
		r17:16 = r13:12
		if (!p0) jump  .L_sample1                   // if no. of sample is 1 jump to L_sample1
		r13 = r19
	}
.L_iirDF1:										  // for Body
	{
		r2 = sat(r17:16)                          // output = yScaled = (int32)y
		r12 = memw(r0++#4)						  // load input x in r12
		r11:10 = mpy(r6, r28)                     // calculate b1*w1 and load into r11:10
		
	}
	{
		r11:10 += mpy(r3, r12)                    // r11:10 = b0*x + b1*w1
		r17:16 = mpy(r3, r13)                     // calculate w4*b0 and load into r17:16
		r18 = r2                                  // load yScaled into w3 (w3 = yScaled)
	}
	{
		r15 = asr(r14, #31)                       // load sign extended bits in r15
		r17:16 += mpy(r6, r18)                    // calculate fb = w3*b1 + w4*b0
	}
	{
		r17:16 = sub(r9:8, r17:16)                // p5 - fb
		r11:10 += asl(r15:14, r5)                 // calculate ff = b0*x + b1*w1 + w2 
		r14 = r28                                 // w2 = w1
		r28 = r12                                 // w1 = x
	}
	{
		r17:16 = add(r17:16, r11:10)              // y = ff - fb + p5
		memw(r1++#4) = r2                         // store output in output buffer
	}
	{
		r17:16 = asl(r17:16, r7)                  // y = y << -qfactor
		r13 = r18                                 // w4 = w3
	}:endloop0                                    // end for loop
	{
		memw(r4 + #4) = r14                       // store w2 
		memw(r4 + #12) = r13                      // store w4 
	}
.L_sample1:
	{	
		r2 = sat(r17:16)                          // output = yScaled = (int32)y
		memw(r1++#4) = r2.new                     // store output in output buffer
		r28 = memw(r0+#-4)	                      // r28 = x , x is the last sample
	}
	{
		memw(r4 + #0) = r28                       // store w1 = x
		memw(r4 + #8) = r2                        // store w3 
	}
.L_sample0:
	{
		r17:16 = memd(r29 + #8)
		r19:18 = memd(r29 + #0)
	}
	{
		dealloc_return
	}
.size   iirDF1_32_ap, .-iirDF1_32_ap

#endif