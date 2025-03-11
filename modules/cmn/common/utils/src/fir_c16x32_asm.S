/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* =========================================================================
   FILE NAME: fir_c16xd32.S
   DESCRIPTION:
   	   Assembly function of fir filter for 32 bit input and 16 bit coefficient
  =========================================================================  */

/*******************************************************************************************************
void fir_c16xd32(int32 *memPtr, int16 *reverse_coeff, int nInputProcSize, int address, int32 *outPtr)
********************************************************************************************************/

	.text
    .p2align 2
    .p2align 4,,15
    .globl  fir_c16xd32
    .type   fir_c16xd32,@function
    .falign
 fir_c16xd32:
 	{
 		p0 = cmp.gt(r2,#0)
 		memd(r29+#-48) =  r17:16                 //Saving register in stack
        memd(r29+#-40) =  r19:18                 //Saving register in stack
        r2 = add(r2,#3)
    }
#ifndef FEATURE_POPQUEUE_CONFLICT
 	{
		r2 = asr(r2,#2)							 //(nInputProcSize+3)/4
        memd(r29+#-32) =  r21:20                 //Saving register in stack
        memd(r29+#-24) =  r23:22                 //Saving register in stack
        if(!p0) jumpr r31		  	             //if(nInputProcSize <= 0) return
    }
#else
 	{
		r2 = asr(r2,#2)							 //(nInputProcSize+3)/4
        memd(r29+#-32) =  r21:20                 //Saving register in stack
        memd(r29+#-24) =  r23:22                 //Saving register in stack
    
}
{
        if(!p0) jumpr r31		  	             //if(nInputProcSize <= 0) return
}
#endif
    {
       	r3 = asrrnd(r3,#1)						 //(address+1)/2
        memd(r29+#-16) =  r25:24                 //Saving register in stack
        memd(r29+#-8)  =  r27:26                 //Saving register in stack
        loop1(Outer_Loop,r2)					 //for (i = 0; i < nInputProcSize; i += 4)
    }
/********************************************************************
r0 : memPtr
r1 : reverse_coeff
r2 : nInputProcSize/4
r3 : address/2
r4 : outPtr

r5 : coef_temp
r6 : memPtr_temp
r7 : reverse_coeff_temp

r9:8	:	y64_1
r11:10	:	y64_2
r13:12	:	y64_3
r15:14	:	y64_4

r17:16	:	in10
r19:18	:	in21
r21:20	:	in32
r23:22	:	in43
r25:24	:	in54

r27:26	:	coef

********************************************************************/

.falign
Outer_Loop:
	{
		r17:16 = memd(r0)						//in10 = ((int64*)memPtr)[0]
		r21:20 = memd(r0+#8)					//in32 = ((int64*)memPtr)[1]
		r7 = r1									//reverse_coeff_temp = reverse_coeff
		r9:8 = xor(r9:8,r9:8)					//y64_1 = 0
	}
	{
		r11:10 = r9:8							//y64_2 = 0
		r13:12 = r9:8							//y64_3 = 0
		r19:18 = valignb(r21:20,r17:16,#4)		//in21 = Q6_P_valignb_PPI(in32,in10,4)
		r5 = memw(r7++#4)						//coef_temp = ((int32*)reverse_coeff)[0]
	}
	{
		r15:14 = r9:8							//y64_4 = 0
		r27:26 = vsxthw(r5)						//coef = Q6_P_vsxthw_R(coef_temp)
		r6 = add(r0,#16)						//memPtr_temp = (int64*)memPtr + 2
		loop0(Inner_Loop,r3)					//for (j = 0; j < (int) address; j += 2)
	}

	.falign
	Inner_Loop:
		{
			r9:8   += vrmpyweh(r17:16,r27:26)	//y64_1 = Q6_P_vrmpywehacc_PP(y64_1,in10,coef)
			r11:10 += vrmpyweh(r19:18,r27:26)	//y64_2 = Q6_P_vrmpywehacc_PP(y64_2,in21,coef)
			r25:24 = memd(r6++#8)				//in54 = ((int64*)memPtr)[(j/2) + 2]
			r17:16 = r21:20						//in10 = in32
		}
		{
			r23:22 = valignb(r25:24,r21:20,#4)	//in43 = Q6_P_valignb_PPI(in54,in32,4)
			r13:12 += vrmpyweh(r21:20,r27:26)	//y64_3 = Q6_P_vrmpywehacc_PP(y64_3,in32,coef)
			r21:20 = r25:24						//in32 = in54
			r5 = memw(r7++#4)					//coef_temp = ((int32*)reverse_coeff)[j/2 + 1]
		}
		{
			r19:18 = r23:22						//in21 = in43
			r15:14 += vrmpyweh(r23:22,r27:26)	//y64_4 = Q6_P_vrmpywehacc_PP(y64_4,in43,coef)
			r27:26 = vsxthw(r5)					//coef = Q6_P_vsxthw_R(coef_temp)
		}:endloop0								//end for (j = 0; j < (int) address; j += 2)

	{
		r0 = add(r0,#16)						//memPtr = memPtr + 4
		r9:8 = asr(r9:8,#15)					//s64_shl_s64(y64_1, -15)
		r11:10 = asr(r11:10,#15)				//s64_shl_s64(y64_2, -15)
	}
	{
		r13:12 = asr(r13:12,#15)				//s64_shl_s64(y64_3, -15)
		r15:14 = asr(r15:14,#15)				//s64_shl_s64(y64_4, -15)
	}
	{
		r8 = sat(r9:8)							//s32_saturate_s64(s64_shl_s64(y64_1, -15))
		r9 = sat(r11:10)						//s32_saturate_s64(s64_shl_s64(y64_2, -15))
	}
	{
		r10 = sat(r13:12)						//s32_saturate_s64(s64_shl_s64(y64_3, -15))
		r11 = sat(r15:14)						//s32_saturate_s64(s64_shl_s64(y64_4, -15))
		memd(r4++#8) = r9:8						//storing at output buffer
	}
	{
		memd(r4++#8) = r11:10					//storing at output buffer
	}:endloop1

    {
        r17:16 =  memd(r29+#-48)                 //Restoring Register
        r19:18 =  memd(r29+#-40)                 //Restoring Register
    }
    {
        r21:20 =  memd(r29+#-32)                 //Restoring Register
        r23:22 =  memd(r29+#-24)                 //Restoring Register
    }
#ifndef FEATURE_POPQUEUE_CONFLICT
    {
        r25:24 =  memd(r29+#-16)                 //Restoring Register
        r27:26 =  memd(r29+#-8)                  //Restoring Register
        jumpr r31
    }
#else
    {
        r25:24 =  memd(r29+#-16)                 //Restoring Register
        r27:26 =  memd(r29+#-8)                  //Restoring Register
    
}
{
        jumpr r31
}
#endif
 	.size   fir_c16xd32, .-fir_c16xd32
