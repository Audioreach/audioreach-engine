/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*===========================================================================*]
[* FILE NAME: audio_buffer_asm.S                                             *]
[* DESCRIPTION:                                                              *]
[*    Assembly Code of functions in audio_buffer.c                           *]
[* FUNCTION LIST :                                                           *]
[*    buff_fill_mix(), buff_copy_mix(), buffer_fill(), buffer_mix()          *]
[*===========================================================================*/
#if ((defined __hexagon__) || (defined __qdsp6__))
/*****************************************************************************/
/* Function:         buffer_fill                                             */
/*---------------------------------------------------------------------------*/
/* Description:      Apply L16Q15 gain to input and store the result to      */
/*                   the output buffer.                                      */
/*---------------------------------------------------------------------------*/
/* C Prototype:      void buffer_fill(int16 *destBuf, int16 *srcBuf,         */
/*                   int16 gainL16Q15, int16 samples)                        */
/*---------------------------------------------------------------------------*/
/* Reg Inputs:       R0 : Pointer to output vector      (o)                  */
/*                   R1 : Pointer to input vector       (i)                  */
/*                   R2 : Gain Value                    (i)                  */
/*                   R3 : No of Input samples           (i)                  */
/*---------------------------------------------------------------------------*/
/* Stack Arguments:      None                                                */
/*---------------------------------------------------------------------------*/
/* Register Outputs:      R0                                                 */
/*---------------------------------------------------------------------------*/
/* Registers Affected:   R0-R15                                              */
/*---------------------------------------------------------------------------*/
/* Hardware Loops Usage: LOOP0,                                              */
/*---------------------------------------------------------------------------*/
/* Stack Memory Usage (in Bytes):   None                                     */
/*---------------------------------------------------------------------------*/
/* Thread Cycles:    None                                                    */
/*---------------------------------------------------------------------------*/
/* Notes:                                                                    */
/*      1. To match QSound code, there is no discussion of the gain values,  */
/*         compared to function buffer_mix                                   */
/*      2. The function buffer_fill used by chorus is called from the        */
/*         process function where input and output buffers are half word     */
/*         aligned. The gain from vectorization is compensated with the load,*/
/*         store cycle penalty.                                              */
/*      3. Present assembly with grouped unvectorized instructions consumes  */
/*         2 cycles per 4 samples for non unity and (-1) gain case.          */
/*      4. Present assembly with grouped unvectorized instructions consumes  */
/*         1 cycles per 4 samples for unity gain case.                       */
/*      5. This code assumes  samples to be >= 12 and multiple of 4          */
/*      6. This code assumes  dstBuf poiner to be 8 byte aligned             */
/*      7. This code assumes caller to handle out of boundary memory         */
/*         access that can happen on source pointers. First three and last   */
/*         four samples should be processed in c code.                       */
/*****************************************************************************/
#include "asm_macros.h"
.text
.p2align 2
#define Q15_ONE 32767
#define Q15_MINUS_ONE -32768

.globl buffer_fill_asm
buffer_fill_asm:
    {
        R4 = #Q15_ONE;                          // gain = 32767
        R5 = #Q15_MINUS_ONE;                    // gain = -32768	
    }
    {
	    R1=AND(R1,#0xFFFFFFF8);                 // 8 byte aligned srcBuf
		R6=AND(R1,#7);                          // offset to 8-bytes srcBuf boundary
	    R3=ASR(R3,#2);		                    // samples = samples / 4 
	}		
    {
        P2 = CMP.EQ(R2, R4);                    // if(gain = Q15_ONE)
        P3 = CMP.EQ(R2, R5);                    // if(gain = Q15_MINUSONE)		
        R11:10=MEMD(R1++#8);                    // load previous DW from srcBuf		
    }
    {
	    P0=R6;                                  // P0 = 8 byte alignment offset of dstBuf	
		R9:8=VSPLATH(R2);                       // copying gainL16Q15 		
		R3=ADD(R3,#-2);                         // samples = (samples/4) -2
        R13:12=MEMD(R1++#8);	                // load current DW from srcBuf
	}
    {
        R15:14=VALIGNB(R13:12,R11:10,P0)        // align data from pre and cur DW of srcBuf
	    R11:10=R13:12;                          // update previous DW of srcBuf		
        IF(P2) JUMP .lbuffer_fill_q15one;	    // if gain = 1 jump to label
	}
	{
      
      R4 = VMPYH(R14,R8):<<1:RND:SAT;          // ( *srcBuf * gainL16Q15 ) << 1:rnd:sat
	    IF(P3) JUMP .lbuffer_fill_q15minusone;	// if gain = -1 jump to label  
	}
	{
	  
	    R5 = VMPYH(R15,R9):<<1:RND:SAT;    // ( *srcBuf * gainL16Q15 ) << 1:rnd:sat
	    R13:12=MEMD(R1++#8);	                // load current DW from srcBuf
	    LOOP0(.lbuffer_fill_loop,R3);           // initialize loop   	
	}
.lbuffer_fill_loop:
	{
       
	    R15:14=VALIGNB(R13:12,R11:10,P0)        // align data from pre and cur DW of srcBuf
	    R11:10=R13:12;                          // update previous DW of srcBuf	   
	}	
	{
     
      R4 = VMPYH(R14,R8):<<1:RND:SAT;           // ( *srcBuf * gainL16Q15 ) << 1:rnd:sat
	 
	  R5 = VMPYH(R15,R9):<<1:RND:SAT;           // ( *srcBuf * gainL16Q15 ) << 1:rnd:sat
	    R13:12=MEMD(R1++#8);	                // load current DW from srcBuf	   
	    MEMD(R0++#8) = R5:4;                    // storing to dest buffer
	}:endloop0	
	{
	
	    R15:14=VALIGNB(R13:12,R11:10,P0)        // align data from pre and cur DW of srcBuf	   
	}
	{
        
      R4 = VMPYH(R14,R8):<<1:RND:SAT;           // ( *srcBuf * gainL16Q15 ) << 1:rnd:sat
	   
	  R5 = VMPYH(R15,R9):<<1:RND:SAT;           // ( *srcBuf * gainL16Q15 ) << 1:rnd:sat
	    MEMD(R0++#8) = R5:4;                    // storing to dstBuf
	}

	{
	    MEMD(R0++#8) = R5:4;                    // storing to dstBuf
	    JUMP .lbuffer_fill_asm_exit;            // exit
	}	
.lbuffer_fill_q15one:
	{
	    LOOP0(.lbuffer_fill_q15one_loop,R3);    // initialize loop	   	
	    R13:12=MEMD(R1++#8);	                // load current DW from srcBuf	   	   
	}
.lbuffer_fill_q15one_loop:
	{
	    R15:14=VALIGNB(R13:12,R11:10,P0)        // align data from pre and cur DW of srcBuf
	    R11:10=R13:12;                          // update previous DW of srcBuf
	    MEMD(R0++#8) = R15:14;                  // storing to dest buffer
	    R13:12=MEMD(R1++#8);	                // load current DW from srcBuf	   	   
	}:endloop0	
	{
	    R15:14=VALIGNB(R13:12,R11:10,P0)        // align data from pre and cur DW of srcBuf	
	    MEMD(R0++#8) = R15:14;                  // storing to dstBuf
	}
	{
	    MEMD(R0++#8) = R15:14;                  // storing to dstBuf
	    JUMP .lbuffer_fill_asm_exit;            // exit
	}	
.lbuffer_fill_q15minusone:
    {
	    R3=ADD(R3,#1);                          // samples = (samples/4) -1
		R7:6=#0;                                // R7:6=0
    }
	{
	    LOOP0(.lbuffer_fill_q15minusone_loop,R3); // initialize loop	   	
	}
.lbuffer_fill_q15minusone_loop:
	{
        R5:4=VSUBH(R7:6,R15:14):SAT;            // 0-srcBuf,
	    R13:12=MEMD(R1++#8);	                // load current DW from srcBuf	   
	}
	{
	    R15:14=VALIGNB(R13:12,R11:10,P0)        // align data from pre and cur DW of srcBuf
	    R11:10=R13:12;                          // update previous DW of srcBuf
	    MEMD(R0++#8) = R5:4;                    // storing to dest buffer
	}:endloop0	
	{
        R5:4=VSUBH(R7:6,R15:14):SAT;            // 0-srcBuf,
	}
	{
	    MEMD(R0++#8) = R5:4;                    // storing to dstBuf
	}
.lbuffer_fill_asm_exit:
    JUMPR R31;                                  // return from function

/*****************************************************************************/
/* Function:         buffer_mix_asm                                          */
/*---------------------------------------------------------------------------*/
/* Description:      Apply L16Q15 gain to input and mix (sum) it into a      */
/*                   running output buffer                                   */
/*---------------------------------------------------------------------------*/
/* C Prototype:      void buffer_mix(int16 *destBuf, int16 *srcBuf,          */
/*                   int16 gainL16Q15, int16 samples)                        */
/*---------------------------------------------------------------------------*/
/* Reg Inputs:       R0 : Pointer to output vector      (o)                  */
/*                   R1 : Pointer to input vector       (i)                  */
/*                   R2 : Gain Value                    (i)                  */
/*                   R3 : No of Input samples           (i)                  */
/*---------------------------------------------------------------------------*/
/* Stack Arguments:      None                                                */
/*---------------------------------------------------------------------------*/
/* Register Outputs:      R0                                                 */
/*---------------------------------------------------------------------------*/
/* Registers Affected:   R0-R17                                              */
/*---------------------------------------------------------------------------*/
/* Hardware Loops Usage: LOOP0                                               */
/*---------------------------------------------------------------------------*/
/* Stack Memory Usage (in Bytes):   None                                     */
/*---------------------------------------------------------------------------*/
/* Thread Cycles:    None                                                    */
/*---------------------------------------------------------------------------*/
/* Notes:                                                                    */
/*      1. To match QSound code, discuss gain value and divide into three    */
/*         cases compared to function buffer_fill                            */
/*      2. The function buffer_mix in chorus is called from the process      */
/*         function where input and output buffers are half word aligned. The*/
/*         gain achieved with the vectorization is compensated with the load,*/
/*         store cycle penalty.                                              */
/*      3. Presently assembly consumes 3 cycles per 4 samples(non unity gain)*/
/*      4. Presently assembly consumes 2 cycles per 4 samples(unity gain)    */
/*      5. This code assumes  samples to be >= 8 and multiple of 4           */
/*      6. This code assumes  dstBuf poiner to be 8 byte aligned             */
/*      7. This code assumes caller to handle out of boundary memory         */
/*         access that can happen on source pointers. First three and last   */
/*         four samples should be processed in c code.                       */
/*****************************************************************************/
.p2align 2
.globl buffer_mix_asm
buffer_mix_asm:
    __saveonentry_8                              // store R16, R17 on stack
    {
        R4 = #Q15_ONE;                           // gain = 32767
        R5 = #Q15_MINUS_ONE;                     // gain = -32768	
    }
    {
	    R1=AND(R1,#0xFFFFFFF8);                  // 8 byte aligned srcBuf
		R6=AND(R1,#7);                           // offset to 8-bytes srcBuf boundary
	    R3=ASR(R3,#2);		                     // samples = samples / 4 
        R11:10=MEMD(R0);    		             // load previous DW from dstBuf 
	}		
    {
        P2 = CMP.EQ(R2, R4);                     // if(gain = Q15_ONE)
        P3 = CMP.EQ(R2, R5);                     // if(gain = Q15_MINUSONE)		
        R13:12=MEMD(R1++#8);                     // load previous DW from srcBuf		
    }
    {
	    P0=R6;                                   // P0 = 8 byte alignment offset of dstBuf	
		R9:8=VSPLATH(R2);                        // copying gainL16Q15 		
		R3=ADD(R3,#-1);                          // samples = (samples/4) -1        
        R15:14=MEMD(R1++#8);	                 // load current DW from srcBuf
	}
    {
        R17:16=VALIGNB(R15:14,R13:12,P0)         // align data from pre and cur DW of srcBuf
	    R13:12=R15:14;                           // update previous DW of srcBuf		
        IF(P2) JUMP .lbuffer_mix_q15one;	     // if(gain = Q15_ONE)
	}
	{
     
      R4 = VMPYH(R16,R8):<<1:RND:SAT;             // ( *srcBuf * gainL16Q15 ) << 1:rnd:sat
	   IF(P3) JUMP .lbuffer_mix_q15minusone;     // if(gain = Q15_MINUSONE)
	}
	{
	  
	   R5 = VMPYH(R17,R9):<<1:RND:SAT;             // ( *srcBuf * gainL16Q15 ) << 1:rnd:sat
	   LOOP0(.lbuffer_mix_loop,R3);              // initialize loop	   	
	}
.lbuffer_mix_loop:
	{
      
	   R15:14=MEMD(R1++#8);	                     // load current DW from srcBuf
	}	
	{
       R5:4 = VADDH(R11:10,R5:4):SAT;            // *dstBuf + tmpL16,	   
	   R11:10=MEMD(R0+#8);		                 // load current DW from dstBuf
	   R17:16=VALIGNB(R15:14,R13:12,P0)          // align data from pre and cur DW of srcBuf
	   R13:12=R15:14;                            // update previous DW of srcBuf
	}
	{
      
	   R4 = VMPYH(R16,R8):<<1:RND:SAT;             // ( *srcBuf * gainL16Q15 ) << 1:rnd:sat
	   R5 = VMPYH(R17,R9):<<1:RND:SAT;             // ( *srcBuf * gainL16Q15 ) << 1:rnd:sat
	   MEMD(R0++#8) = R5:4;                      // storing to dest buffer
	}:endloop0	
		
	{
       R5:4 = VADDH(R11:10,R5:4):SAT;            // *dstBuf + tmpL16,	   
	}
	{
	   MEMD(R0++#8) = R5:4;                      // storing to dstBuf
	   JUMP .lbuffer_mix_asm_exit;               // exit
	}	
.lbuffer_mix_q15one:
	{
	   LOOP0(.lbuffer_mix_q15one_loop,R3);       // initialize loop	   	
	}
.lbuffer_mix_q15one_loop:
	{
       R5:4 = VADDH(R11:10,R17:16):SAT;          // *dstBuf + srcBuf,	   
	   R15:14=MEMD(R1++#8);	                     // load current DW from srcBuf	   
	   R11:10=MEMD(R0+#8);		                 // load current DW from dstBuf
	}
	{
	   R17:16=VALIGNB(R15:14,R13:12,P0)          // align data from pre and cur DW of srcBuf
	   R13:12=R15:14;                            // update previous DW of srcBuf
	   MEMD(R0++#8) = R5:4;                      // storing to dest buffer
	}:endloop0	
	{
       R5:4 = VADDH(R11:10,R17:16):SAT;          // *dstBuf + srcBuf,	   
	}
	{
	   MEMD(R0++#8) = R5:4;                      // storing to dstBuf
	   JUMP .lbuffer_mix_asm_exit;               // exit
	}
.lbuffer_mix_q15minusone:
	{
	   LOOP0(.lbuffer_mix_q15minusone_loop,R3);  // initialize loop	   	
	}
.lbuffer_mix_q15minusone_loop:
	{
       R5:4 = VSUBH(R11:10,R17:16):SAT;          // *dstBuf - srcBuf,	   
	   R15:14=MEMD(R1++#8);	                     // load current DW from srcBuf	   
	   R11:10=MEMD(R0+#8);		                 // load current DW from dstBuf
	}
	{
	   R17:16=VALIGNB(R15:14,R13:12,P0)          // align data from pre and cur DW of srcBuf
	   R13:12=R15:14;                            // update previous DW of srcBuf
	   MEMD(R0++#8) = R5:4;                      // storing to dest buffer
	}:endloop0	
	{
       R5:4 = VSUBH(R11:10,R17:16):SAT;          // *dstBuf - srcBuf,	   
	}
	{
	   MEMD(R0++#8) = R5:4;                      // storing to dstBuf
	}
.lbuffer_mix_asm_exit:
#ifndef FEATURE_POPQUEUE_CONFLICT
    __restoreonexit_8                            //restore R16, R17 from stack and return 
#else
{
  R17:16 = MEMD(SP++#8);   /*restore r16,r17 & dealloc stack */
}
{
  JUMPR R31;       /*return to caller    */
}
#endif
	
/*****************************************************************************/
/* Function:         buffer_fill_mix_asm                                     */
/*---------------------------------------------------------------------------*/
/* Description:      Apply L16Q15 gain to input(src2), mix (sum) it with     */
/*                   another source (src1) and store in output buffer.       */
/*---------------------------------------------------------------------------*/
/* C Prototype:      void buffer_fill_mix(int16 *src1,int16 *src2,int16 *dst,*/
/*                   int16 gainL16Q15, int16 samples)                        */
/*---------------------------------------------------------------------------*/
/* Reg Inputs:       R0 : Pointer to input vector       (i)                  */
/*                   R1 : Pointer to input vector       (i)                  */
/*                   R2 : Pointer to output vector      (o)                  */
/*                   R3 : Gain Value                    (i)                  */
/*                   R4 : No of Input samples           (i)                  */
/*---------------------------------------------------------------------------*/
/* Stack Arguments:      None                                                */
/*---------------------------------------------------------------------------*/
/* Register Outputs:      R2                                                 */
/*---------------------------------------------------------------------------*/
/* Registers Affected:   R0-R21                                              */
/*---------------------------------------------------------------------------*/
/* Hardware Loops Usage: LOOP0                                               */
/*---------------------------------------------------------------------------*/
/* Stack Memory Usage (in Bytes):   None                                     */
/*---------------------------------------------------------------------------*/
/* Thread Cycles:    None                                                    */
/*---------------------------------------------------------------------------*/
/* Notes:                                                                    */
/*      1. Presently assembly consumes 3 cycles per 4 samples(non unity gain)*/
/*      2. Presently assembly consumes 2 cycles per 4 samples(unity gain)    */
/*      3. This code assumes  samples to be >= 12 and multiple of 4          */
/*      4. This code assumes  dst poiner to be 8 byte aligned                */
/*      5. This code assumes caller to handle out of boundary memory         */
/*         access that can happen on source pointers. First three and last   */
/*         four samples should be processed in c code.                       */
/*****************************************************************************/
.p2align 2
.globl buffer_fill_mix_asm
buffer_fill_mix_asm:
    __saveonentry_32                            // Save R16:R23 on stack
    {
        R10 = #Q15_ONE;                         // gain = 32767
        R11 = #Q15_MINUS_ONE;                   // gain = -32768	
    }
    {
	    R1=AND(R1,#0xFFFFFFF8);                 // 8 byte aligned src2
		R7=AND(R1,#7);                          // offset to 8-bytes src2 boundary
	    R4=ASR(R4,#2);		                    // samples = samples / 4 
	}			
    {
        P2 = CMP.EQ(R3, R10);                   // if(gain = Q15_ONE)
        P3 = CMP.EQ(R3, R11);                   // if(gain = Q15_MINUSONE)		
        R15:14=MEMD(R1++#8);                    // load previous DW from src2		
	    P0=R7;                                  // P0 = 8 byte alignment offset of src2
    }
    {
	    R0=AND(R0,#0xFFFFFFF8);                 // 8 byte aligned src1
		R6=AND(R0,#7);                          // offset to 8-bytes src1 boundary
		R9:8=VSPLATH(R3);                       // copying gainL16Q15 		
        R17:16=MEMD(R1++#8);	                // load current DW from src2		
	}
	{
	    R11:10=MEMD(R0++#8);		            // load previous DW from src1 
        R19:18=VALIGNB(R17:16,R15:14,P0)        // align data from pre and cur DW of src2
	    R15:14=R17:16;                          // update previous DW of src2		
	}
    {
	    P1=R6;                                  // P1 = 8 byte alignment offset of src1	
		R4=ADD(R4,#-1);                         // samples = (samples/4) -1
		R13:12=MEMD(R0++#8);		            // load current DW from src1	   
        IF(P2) JUMP .lbuffer_fill_mix_q15one;   // if(gain = Q15_ONE)
    }
	{ 
        IF(P3) JUMP .lbuffer_fill_mix_q15minusone; // if(gain = Q15_MINUSONE)
	}
	{
	   LOOP0(.lbuffer_fill_mix_loop,R4);          // initialize loop
	}
	{
       R5:4 = VMPYH(R18,R8):<<1:SAT;             // ( *src2 * gainL16Q15 ) << 1	
	   R7:6 = VMPYH(R19,R9):<<1:SAT;             // ( *src2 * gainL16Q15 ) << 1	
	}
.lbuffer_fill_mix_loop:
	{
       R5:4 = VTRUNOWH(R7:6,R5:4);               // To get odd half words	
	   R21:20 = VALIGNB(R13:12,R11:10,P1)        // align data from pre and cur DW of src1
	   R11:10=R13:12;	                         // update previous DW of src1
	   R17:16=MEMD(R1++#8);	                     // load current DW from src2
	}	
	{
       R5:4 = VADDH(R21:20,R5:4):SAT;            // *src1 + tmpL16,	   
	   R13:12=MEMD(R0++#8);		                 // load current DW from src1
	   R19:18=VALIGNB(R17:16,R15:14,P0)          // align data from pre and2 cur DW of src2
	   R15:14=R17:16;		                     // update previous DW of src2
	}
	{
       R5:4 = VMPYH(R18,R8):<<1:SAT;             // ( *src2 * gainL16Q15 ) << 1	
	   R7:6 = VMPYH(R19,R9):<<1:SAT;             // ( *src2 * gainL16Q15 ) << 1	
	   MEMD(R2++#8) = R5:4;                      // storing to dest buffer
	}:endloop0	
	{
       R5:4 = VTRUNOWH(R7:6,R5:4);               // To get odd half words	
	   R21:20 = VALIGNB(R13:12,R11:10,P1)        // align data from pre and cur DW of src1
	}	
	{
       R5:4 = VADDH(R21:20,R5:4):SAT;            // *src1 + tmpL16,	   
	}
	{
	   MEMD(R2++#8) = R5:4;                      // storing to dest buffer
	   JUMP .lbuffer_fill_mix_asm_exit;          // exit
	}	
.lbuffer_fill_mix_q15one:
	{
	   R4=ADD(R4,#-1)                            // samples = (samples/4) -2 
	   R21:20=VALIGNB(R13:12,R11:10,P1)          // align data from pre and cur DW of src1
	   R11:10=R13:12;	                         // update previous DW of src1
	   R17:16=MEMD(R1++#8);	                     // load current DW from src2
	}
	{
	   LOOP0(.lbuffer_fill_mix_q15one_loop,R4);  // initialize loop
	}
.lbuffer_fill_mix_q15one_loop:
	{
       R5:4 = VADDH(R21:20,R19:18):SAT;          // *src1 + *src2,
	   R13:12=MEMD(R0++#8);		                 // load current DW from src1
	   R19:18=VALIGNB(R17:16,R15:14,P0)          // align data from pre and cur DW of src2
	   R15:14=R17:16;		                     // update previous DW of src2
	}
	{
	   R21:20 = VALIGNB(R13:12,R11:10,P1)        // align data from pre and cur DW of src1
	   R11:10=R13:12;	                         // update previous DW of src1
	   R17:16=MEMD(R1++#8);	                     // load current DW from src2	
	   MEMD(R2++#8) = R5:4;                      // storing to dest buffer
	}:endloop0	
	{
       R5:4 = VADDH(R21:20,R19:18):SAT;          // *src1 + *src2,	   
	   R13:12=MEMD(R0++#8);		                 // load current DW from src1
	   R19:18=VALIGNB(R17:16,R15:14,P0)          // align data from pre and cur DW of src2
	}
	{
	   R21:20 = VALIGNB(R13:12,R11:10,P1)        // align data from pre and cur DW of src1
	   MEMD(R2++#8) = R5:4;                      // storing to dest buffer
	}
	{
       R5:4 = VADDH(R21:20,R19:18):SAT;          // *src1 + *src2,   
	}
	{
	   MEMD(R2++#8) = R5:4;                      // storing to dest buffer
	   JUMP .lbuffer_fill_mix_asm_exit;          // exit
	}	
.lbuffer_fill_mix_q15minusone:
	{
	   R4=ADD(R4,#-1)                            // samples = (samples/4) -2 
	   R21:20=VALIGNB(R13:12,R11:10,P1)          // align data from pre and cur DW of src1
	   R11:10=R13:12;	                         // update previous DW of src1
	   R17:16=MEMD(R1++#8);	                     // load current DW from src2
	}
	{
	   LOOP0(.lbuffer_fill_mix_q15minusone_loop,R4); // initialize loop
	}
.lbuffer_fill_mix_q15minusone_loop:
	{
       R5:4 = VSUBH(R21:20,R19:18):SAT;          // *src1 - *src2,
	   R13:12=MEMD(R0++#8);		                 // load current DW from src1
	   R19:18=VALIGNB(R17:16,R15:14,P0)          // align data from pre and cur DW of src2
	   R15:14=R17:16;		                     // update previous DW of src2
	}
	{
	   R21:20 = VALIGNB(R13:12,R11:10,P1)        // align data from pre and cur DW of src1
	   R11:10=R13:12;	                         // update previous DW of src1
	   R17:16=MEMD(R1++#8);	                     // load current DW from src2	
	   MEMD(R2++#8) = R5:4;                      // storing to dest buffer
	}:endloop0	
	{
       R5:4 = VSUBH(R21:20,R19:18):SAT;          // *src1 - *src2,	   
	   R13:12=MEMD(R0++#8);		                 // load current DW from src1
	   R19:18=VALIGNB(R17:16,R15:14,P0)          // align data from pre and cur DW of src2
	}
	{
	   R21:20 = VALIGNB(R13:12,R11:10,P1)        // align data from pre and cur DW of src1
	   MEMD(R2++#8) = R5:4;                      // storing to dest buffer
	}
	{
       R5:4 = VSUBH(R21:20,R19:18):SAT;          // *src1 - *src2,   
	}
	{
	   MEMD(R2++#8) = R5:4;                      // storing to dest buffer
	}
.lbuffer_fill_mix_asm_exit:
#ifndef FEATURE_POPQUEUE_CONFLICT
    __restoreonexit_32                           // restore R16:R23 from stack and return
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
#endif
