/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* =========================================================================
   FILE NAME: fir_c16xd32.c
   DESCRIPTION:
   	   Fir filter for 32 bit input and 16 bit coefficient
  =========================================================================  */



#include "audio_dsp.h"
#include "audio_basic_op_ext.h"

//#define FIR_C16XD32_C_UNROLLED
//#define FIR_C16XD32_INTRISNIC

#ifdef FIR_C16XD32_C_UNROLLED
#undef FIR_C16XD32_INTRISNIC
#endif

#ifdef FIR_C16XD32_C_UNROLLED
void fir_c16xd32(int32 *memPtr, int16 *reverse_coeff, int nInputProcSize, int address, int32 *outPtr)
{
    int i, j;
    for (i = 0; i < nInputProcSize; i += 4) {
        int64 y64_1 = 0, y64_2 = 0, y64_3 = 0, y64_4 = 0;

        for (j = 0; j < (int) address; j += 2) {
            // convolution, y = sum (c[k] * x[n-k]) , k = 0, ..., taps-1

            y64_1 = s64_mac_s32_s32(y64_1, memPtr[j + 0], reverse_coeff[j]);
            y64_1 = s64_mac_s32_s32(y64_1, memPtr[j + 1], reverse_coeff[j+1]);

            y64_2 = s64_mac_s32_s32(y64_2, memPtr[j + 1], reverse_coeff[j]);
            y64_2 = s64_mac_s32_s32(y64_2, memPtr[j + 2], reverse_coeff[j+1]);

            y64_3 = s64_mac_s32_s32(y64_3, memPtr[j + 2], reverse_coeff[j]);
            y64_3 = s64_mac_s32_s32(y64_3, memPtr[j + 3], reverse_coeff[j+1]);

            y64_4 = s64_mac_s32_s32(y64_4, memPtr[j + 3], reverse_coeff[j]);
            y64_4 = s64_mac_s32_s32(y64_4, memPtr[j + 4], reverse_coeff[j+1]);
        }
        memPtr = memPtr + 4;
        // shift and output sample
        *outPtr++ = s32_saturate_s64(s64_shl_s64(y64_1, -15));
        *outPtr++ = s32_saturate_s64(s64_shl_s64(y64_2, -15));
        *outPtr++ = s32_saturate_s64(s64_shl_s64(y64_3, -15));
        *outPtr++ = s32_saturate_s64(s64_shl_s64(y64_4, -15));

    }  // end of i loop
}
#endif

#ifdef FIR_C16XD32_INTRISNIC
void fir_c16xd32(int32 *memPtr, int16 *reverse_coeff, int nInputProcSize, int address, int32 *outPtr)
{
    int i, j;

    int64 coef;
    int64 in10;
    int64 in21;
    int64 in32;
    int64 in43;
    int64 in54;

    for (i = 0; i < nInputProcSize; i += 4)
    {
        int64 y64_1 = 0, y64_2 = 0, y64_3 = 0, y64_4 = 0;

        in10 = ((int64*)memPtr)[0];
        in32 = ((int64*)memPtr)[1];
        in21 = Q6_P_valignb_PPI(in32,in10,4);
    	coef = Q6_P_vsxthw_R(((int32*)reverse_coeff)[0]);

        for (j = 0; j < (int) address; j += 2)
        {
        	y64_1 = Q6_P_vrmpywehacc_PP(y64_1,in10,coef);
        	y64_2 = Q6_P_vrmpywehacc_PP(y64_2,in21,coef);
        	in54 = ((int64*)memPtr)[(j/2) + 2];
        	in10 = in32;

        	in43 = Q6_P_valignb_PPI(in54,in32,4);
        	y64_3 = Q6_P_vrmpywehacc_PP(y64_3,in32,coef);
        	in32 = in54;

        	in21 = in43;
        	y64_4 = Q6_P_vrmpywehacc_PP(y64_4,in43,coef);
        	coef = Q6_P_vsxthw_R(((int32*)reverse_coeff)[j/2 + 1]);
        }
        memPtr = memPtr + 4;
        // shift and output sample
        *outPtr++ = s32_saturate_s64(s64_shl_s64(y64_1, -15));
        *outPtr++ = s32_saturate_s64(s64_shl_s64(y64_2, -15));
        *outPtr++ = s32_saturate_s64(s64_shl_s64(y64_3, -15));
        *outPtr++ = s32_saturate_s64(s64_shl_s64(y64_4, -15));

    }  // end of i loop
}
#endif
