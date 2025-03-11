#if ((defined __hexagon__) || (defined __qdsp6__))
/*==========================================================================*/
/* FUNCTION: divide_int32_qx                                                */
/*                                                                          */
/* DESCRIPTION: numerator/denominator (both in the same Q) and outputs      */
/*              quotient in Q0.                                             */
/*                                                                          */
/* INPUTS: numeratorL32: numerator in Qin.                                  */
/*         denominatorL32: denominator in Qin.                              */
/* OUTPUTS: quotient: quotient of operation in Q0.                          */
/*                                                                          */
/* IMPLEMENTATION NOTES:                                                    */
/*                                                                          */
/*        if(num >= 2^31*den)                                               */
/*           num = num - den;                                               */
/*           quotient = quotient<<1+1;                                      */
/*        else                                                              */
/*           quotient = quotient<<1;                                        */
/*      num = num<<1;                                                       */
/*                                                                          */
/*==========================================================================*/
	.text
	.p2align 4
	.globl	divide_int32_qx
	.type	divide_int32_qx,@function
divide_int32_qx:
	{
		R3 = R0        						// R3->numerator, R2->Q, R1->denominator
		P0 = cmp.eq(R0, #0)					// if(numerator==0) return 0;
		if (P0.new) jump:nt .L_DIVIDE_INT32_QX_RETURN
		R4 = abs(R0):sat        			// numeratorL32 = -numeratorL32
	}
	{
		R0 = #-1
		P1 = cmp.eq(R1, #0)					// if(denominator==0) return -1;
		if (P1.new) jump:nt .L_DIVIDE_INT32_QX_RETURN
		R5 = abs(R1):sat					// denominatorL32 = -denominatorL32
	}
	{
		R6 = add(clb(R4), #-1)   			// nShift = s16_norm_s32(numeratorL32)
		R7 = add(clb(R5), #-1)				// dShift = s16_norm_s32(denominatorL32)
	}
	{
		R9 = #0
		R8 = asl(R4, R6):sat				// shiftNumerL40 = s32_shl_s32_sat(numeratorL32, nShift)
		R2 += sub(R7, R6)					// count = s16_sub_s16_s16(count, nShift)
		R0 = #0
	}
	{
		R11 = #0
		R10 = asl(R5, R7):sat				// denominatorL32 = s32_shl_s32_sat(denominatorL32, dShift)
		R2 = add(R2, #1)					// count = s16_add_s16_s16(dShift,1)
		if (!cmp.gt(R2.new, #0)) jump:nt .L_DIVIDE_INT32_QX_RETURN
	}
	{
		loop0(.L_DIVIDE_INT32_QX_LOOP, R2) 	// for(i = 0; i<count; i++)
		R15 = #1
		//R13:12 = sub(R9:8, R11:10)        // temp = SN - SD
	}

.falign
.L_DIVIDE_INT32_QX_LOOP:
	{
		R13:12 = sub(R9:8, R11:10)			// tempL40 = s40_sub_s40_s40(shiftNumerL40, denominatorL32)
		R0 = asl(R0, R15):sat				// quotientL32 = s32_shl_s32_sat(quotientL32,1 )
	}
	{
		P0 = cmp.gt(R11:10, R9:8)			// if (tempL40 >=0)
		if(!P0.new) R9:8 = R13:12			// shiftNumerL40 = s40_shl_s40(tempL40,1);
		if(!P0.new) R6 = #1
		if(P0.new) R6 = #0
	}
	{
		R9:8 = asl(R9:8, #1)				// shiftNumerL40 = s40_shl_s40(shiftNumerL40,1)
		R0 = add(R0, R6):sat				// quotientL32 = s32_add_s32_s32_sat(quotientL32,1)
	}:endloop0

	{
		//R0 = asr(R0, R15):sat             // Q = Q >> 1
		R2 = xor(R1, R3)					// if (D^N > 0) then (return Q) else (return -Q)
		if (cmp.gt(R2.new, #-1)) jump:nt .L_DIVIDE_INT32_QX_RETURN
	}
	{
		R0 = neg(R0)
	}

.L_DIVIDE_INT32_QX_RETURN:
	{
		jumpr R31							// return quotientL32
	}
	.size	divide_int32_qx, .-divide_int32_qx
	
	     .p2align 2
      .globl div_s

div_s:
     { 
        R4=#0                             // var_out=0
        P1=CMP.EQ(R1,#0)
        IF (P1.new) JUMP:nt .Ldiv_s_exit   // if (L_Den==0),
        R9=CLB(R1)
     }
     {
        P0=CMP.EQ(R0,#0)
        IF (P0.new) JUMP:nt .Ldiv_s_exit   // if (L_num==0),
                                          // var_out=0
        P3=CMP.EQ(R0,R1) 
        R8=CLB(R0)
     }
     {
        R8=SUB(R8,R9)                     // difference of clb's
        R4=#32767                         // if (L_num=L_den), var_out=MAX_16
        IF P3 JUMP .Ldiv_s_exit            // exit if L_num=L_den
     }
     {
        P0=CMP.GT(R8,#1)                  // if ((difference of clb's) > 0)
        R10=SUB(#15,R8)                   // 15-(difference of clb's)
        R5=ASL(R0,R8)                     // L_num << =15-(difference of clb's)
        R4=#0
     }
     /* If shifting is done by exact difference of clb's there 
        is chance that num will become greater than denominator
        Hence we shift the initial value by diff of clb's - 1 */
.falign
.Ldo_div_operation:
     {
        R10=MUX(P0,R10,#14)     
        R11=MUX(P0,R8,#1)
        IF (!P0) R0=ADD(R0,R0)            // L_num << =1
        IF (P0) R0=R5                     // L_num << =15-(difference of clb's) 
                                          // var_out << =1
                                          // dont need to saturate, R4 is fixed
                                          // to 0.
     }   
     {
        LOOP0(.Ldiv_s_loop,R10) 
        P0=CMP.GT(R1,R0)                  // if ( L_denom > L_num )
        IF (!P0.new) R4=ADD(R4,#1)        // (var_out+1) - no neet to saturate
                                          // as max value is 7fff
        IF (!P0.new) R0=SUB(R0,R1)        // (L_num-L_denom) //donot need to saturate
     }
.falign
.Ldiv_s_loop:
 /* If (L_num > L_denom), var_out+=1, L_num = L_num-L_denom */
     {
        R0=ASL(R0,#1)                     // L_num << =1
        R4=ADD(R4,R4)                     // var_out << =1
     }
     {
        P0=CMP.GT(R1,R0)                  // if ( L_num > L_denom)
        IF (!p0.new) R4=ADD(R4,#1)        // (var_out+1)
        IF (!P0.new) R0=SUB(R0,R1)        // (L_num-L_denom) //donot need to saturate
     }:endloop0
.falign
.Ldiv_s_exit:
     {
        R0=R4
        JUMPR R31
     }

#endif