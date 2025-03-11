/*
 * Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "byte_conversion.h"
#include <stdlib.h>
#if defined CAPI_STANDALONE
#include "capi_util.h"
#elif defined DEC_CAPI_STANDALONE
#include "dec_capi_util.h"
#elif defined APPI_EXAMPLE_STANDALONE
#include "appi_util.h"
#else
#include <stringl.h>
#endif

static const uint32_t MAX_Q27 = ((1 << PCM_Q_FACTOR_27) - 1);
static const uint32_t MIN_Q27 = ( - (1 << PCM_Q_FACTOR_27));
static const uint32_t MAX_Q31 = 0x7FFFFFFF;
static const uint32_t MIN_Q31 = 0x80000000;

/**
 * This function is for up down conversion of bytes per sample 
 * for the data buf pointed to by ptr_src_buf. 
 *  
 * @param[in] ptr_src_buf, ptr to source buf to be converted, 
 *       assumes channel de-interleaved data
 * @param[in] ptr_out_buf, pointer to output buffer, asumes 
 *       de-interleaved data, should be sufficiently large to
 *       hold the output data.
 * @param[in] num_samples_per_ch, number of samples per channel
 * @param[in] num_channels, Number of channels
 * @param[in] chan_spacing_in, spacing between adjacent channels 
 *       for input buffers
 * @param[in] chan_spacing_out, spacing between adjacent 
 *       channels for output buffers
 * @param[in] conv_mode, up/down conversion mode 
 *  
 * NOTE : 16->32 bit conversion is NOT in place. 32->16 bit 
 * conversion can be both in-place or non-inplace 
 *  
 * For 16->32 bit conversion, output buffer should be 
 * sufficiently large to hold the up converted data. 
 * @return none
 */

void byte_up_down_convertor_deintlv(int8_t * ptr_src_buf, int8_t * ptr_out_buf, uint32_t num_samp_per_ch,
                                    uint32_t num_channels, uint32_t chan_spacing_in, uint32_t chan_spacing_out, 
                                    uint16_t conv_mode, uint16_t in_q_format, uint16_t out_q_format)
{
   int32_t   samp, nChan;
   int32_t   *buf_ptr_src1_32, *buf_ptr_src2_32, *buf_ptr_32;
   int16_t   *buf_ptr_16, *buf_ptr_src_16;
   int64_t   *buf_ptr_64;
   uint32_t  shift_factor = 0;

   /* q27-to-q15 OR q31-to-q15 conversion */
   if(BYTE_DOWN_CONV == conv_mode)
   {
      int64_t dw_MAX = 0;
      int64_t dw_MIN = 0;
      int32_t max_value = 0;
      int32_t min_value = 0;

      //q27 to q15
      if(PCM_Q_FACTOR_27 == in_q_format)
      {
         dw_MAX = ((int64_t)MAX_Q27 << 32) | (int64_t)MAX_Q27;
         dw_MIN = ((int64_t)MIN_Q27 << 32) | (int64_t)MIN_Q27;
         shift_factor = BYTE_UPDOWN_CONV_SHIFT_FACT;     //q27 - q15
         max_value = (int32_t)MAX_Q27;
         min_value = (int32_t)MIN_Q27;
      }
      //q31 to q15
      else if(PCM_Q_FACTOR_31 == in_q_format)
      {
         dw_MAX = ((int64_t)MAX_Q31 << 32) | (int64_t)MAX_Q31;
         dw_MIN = ((int64_t)MIN_Q31 << 32) | (int64_t)MIN_Q31;
         shift_factor = PCM_Q_FACTOR_31 - PCM_Q_FACTOR_15;  //q31 - q15
         max_value = (int32_t)MAX_Q31;
         min_value = (int32_t)MIN_Q31;
      }

#if ((defined __hexagon__) || (defined __qdsp6__))

      buf_ptr_src1_32 = (int32_t *)ptr_src_buf;
      buf_ptr_16 = (int16_t *)ptr_out_buf;

      for(nChan = 0; nChan < num_channels; nChan++)
      {
         buf_ptr_64 = (int64_t *)(buf_ptr_src1_32 + nChan*chan_spacing_in);      /* Input buffer  */
         buf_ptr_32 = (int32_t *)(buf_ptr_16 + nChan*chan_spacing_out);          /* Output buffer */

         /* Convert from (Q27 or Q31) to Q15 */

         /* If input buf addr is 8 byte aligned and out buf addr is 4 byte aligned
            then only perform the vector operation */
         if( (0 == ((uint32_t)buf_ptr_64 & 0x7)) && 
             (0 == ((uint32_t)buf_ptr_32 & 0x3)) )
         {
            for(samp = num_samp_per_ch; samp >=2; samp -= 2)
            {
               int64_t temp64;

               /* Saturate to 27 or 31 bits based on input q format */
               temp64 = Q6_P_vminw_PP(*buf_ptr_64++, dw_MAX);
               temp64 = Q6_P_vmaxw_PP(temp64, dw_MIN);

               /* Right shift each of the 32 bit words into 16 bits and 
               store the two lower halfwords into destination */
               *buf_ptr_32++ = Q6_R_vasrw_PR(temp64, shift_factor);
            }

            /* If the number of samples are odd */
            if(samp)
            {
               int32_t temp32;

               buf_ptr_src_16 = (int16_t *)buf_ptr_32;
               buf_ptr_src2_32 = (int32_t *)buf_ptr_64;

               /* Saturate to 27 or 31 bits based on the input q format */
               temp32 = Q6_R_max_RR(*buf_ptr_src2_32, (int32_t)min_value);
               temp32 = Q6_R_min_RR(temp32, (int32_t)max_value);

               /* Q27->Q15 or Q31->Q15 with rounding */
               *buf_ptr_src_16 = Q6_R_asr_RR(temp32, shift_factor);
            }
         }
         else /* if either of buf addr is not byte aligned as required for vectorization */
         {
            int32_t temp32;

            buf_ptr_src_16 = (int16_t *)buf_ptr_32;
            buf_ptr_src2_32 = (int32_t *)buf_ptr_64;

            if(PCM_Q_FACTOR_27 == in_q_format)
            {
               for(samp = 0; samp < num_samp_per_ch; samp++)
               {
                  /* Saturate to 27 bits */
                  temp32 = Q6_R_max_RR(*buf_ptr_src2_32++, (int32_t)min_value);
                  temp32 = Q6_R_min_RR(temp32, (int32_t)max_value);

                  /* Q27 to Q15 */
                  *buf_ptr_src_16++ = Q6_R_asr_RR(temp32, shift_factor);
               }
            }
            else if(PCM_Q_FACTOR_31 == in_q_format)
            {
               for(samp = 0; samp < num_samp_per_ch; samp++)
               {
                  /* Saturate to 31 bits */
                  temp32 = Q6_R_max_RR(*buf_ptr_src2_32++, (int32_t)min_value);
                  temp32 = Q6_R_min_RR(temp32, (int32_t)max_value);

                  /* Q31 to Q15 */
                  *buf_ptr_src_16++ = Q6_R_asr_RR(temp32, shift_factor);
               }
            }

         }
      }
#else
      /*----------- Non Q6 Version ---------------*/
      for(nChan = 0; nChan < num_channels; nChan++)
      {
         buf_ptr_32 = (buf_ptr_src1_32 + nChan*chan_spacing_in);            /* Input buffer  */
         buf_ptr_16 = (buf_ptr_16 + nChan*chan_spacing_out);                /* Output buffer */

         /* Convert from Q27 (or Q31) to Q15 , inplace conversion */
         for(samp = 0; samp < num_samp_per_ch; samp++)
         {
            int32_t temp32 = *buf_ptr_32++;

            /* Saturate to 27 or 31 bits */
            temp32 = (temp32 < (int32_t)min_value)? (int32_t)min_value: temp32;
            temp32 = (temp32 > (int32_t)max_value)? (int32_t)max_value: temp32;

            /*shift to Q15 */
            *buf_ptr_16++ = (int16_t)( temp32 >> shift_factor);
         }
      }
#endif /* __qdsp6__ */
        
   }

   /* q15-to-q27 OR q15-to-q31 conversion */
   else
   {
      //q15 to q27
      if(PCM_Q_FACTOR_27 == out_q_format)
      {
         shift_factor = BYTE_UPDOWN_CONV_SHIFT_FACT;     //q27 - q15
      }
      //q15 to q31
      else
      {
         shift_factor = PCM_Q_FACTOR_31 - PCM_Q_FACTOR_15;  //q31 - q15
      }
      buf_ptr_16 = (int16_t *)ptr_src_buf;
      buf_ptr_src1_32 = (int32_t *)ptr_out_buf;

#if ((defined __hexagon__) || (defined __qdsp6__))
      for(nChan = 0; nChan < num_channels; nChan++)
      {
         buf_ptr_32 = (int32_t *)(buf_ptr_16 + nChan*chan_spacing_in);            /* Input buffer  */
         buf_ptr_64 = (int64_t *)(buf_ptr_src1_32 + nChan*chan_spacing_out);      /* Output buffer */

         /* Convert from Q15 to Q28, conversion output in scratch buffer */

         /* If Output buf addr is 8 byte aligned and input buf addr is 4 byte aligned
            then only perform the vector operation */
         if( (0 == ((uint32_t)buf_ptr_64 & 0x7)) && 
               (0 == ((uint32_t)buf_ptr_32 & 0x3)) )
         {
            for(samp = num_samp_per_ch; samp >= 2; samp -= 2)
            {
               /* Sign extend two 16-bit words in to two 32-bit words */
               *buf_ptr_64 = Q6_P_vsxthw_R(*buf_ptr_32++);

               /* Shift left the result to convert it to Q27 or Q31 */
               *buf_ptr_64 = Q6_P_vaslw_PR(*buf_ptr_64, shift_factor);
               buf_ptr_64++;
            }

            /* If the number of samples per channel is odd */
            if(samp)
            {
               buf_ptr_src_16 = (int16_t *)buf_ptr_32;
               buf_ptr_src2_32 = (int32_t *)buf_ptr_64;
               *buf_ptr_src2_32 = (int32_t)(*buf_ptr_src_16 << shift_factor);
            }
         }
         else /* if either of buf addr is not byte aligned as required for vectorization */
         {
            buf_ptr_src_16 = (int16_t *)buf_ptr_32;
            buf_ptr_src2_32 = (int32_t *)buf_ptr_64;

            /* Q15 -> Q27(Q31) conversion */
            for(samp = 0; samp < num_samp_per_ch; samp++)
            {
               (*buf_ptr_src2_32++) = (int32_t)( (*buf_ptr_src_16++) << shift_factor);
            }
         }
      }
#else
      /*----------- Non Q6 Version ---------------*/
      for(nChan = 0; nChan < num_channels; nChan++)
      {
         buf_ptr_16 = (buf_ptr_16 + nChan*chan_spacing_in);            /* Input buffer  */
         buf_ptr_32 = (buf_ptr_src1_32 + nChan*chan_spacing_out);      /* Output buffer */

         /* Convert from Q15 to Q27(or Q31), conversion output in scratch buffer */
         for(samp = 0; samp < num_samp_per_ch; samp++)
         {
            *buf_ptr_32++ = ((int32_t)(*buf_ptr_16++)) << shift_factor;
         }
      }
#endif /* __qdsp6__ */
   }
}

/**
 * This function is for up down conversion of bytes per sample 
 * for the data buf pointed to by ptr_src_buf. 
 *  
 * @param[in] ptr_src_buf, ptr to source buf to be converted, 
 *       assumes channel interleaved data
 * @param[out] ptr_out_buf, pointer to output buffer, asumes 
 *       de-interleaved data, should be sufficiently large to
 *       hold the output data.
 * @param[in] total_num_samples, total number of samples 
 * @param[in] conv_mode, up/down conversion mode 
 *  
 * NOTE : 16->32 bit conversion is NOT in place. 32->16 bit 
 * conversion can be both in-place or non-inplace 
 *  
 * For 16->32 bit conversion, output buffer should be 
 * sufficiently large to hold the up converted data. 
 * @return none
 */

void byte_up_down_convertor_intlv(int8_t * ptr_src_buf, int8_t * ptr_out_buf, uint32_t total_num_samples, uint16_t conv_mode)
{
   int32_t   samp;
   int32_t   *buf_ptr_src2_32, *buf_ptr_32;
   int16_t   *buf_ptr_16, *buf_ptr_src_16;
   int64_t   *buf_ptr_64;

   int64_t dw_MAX_Q27 = ((int64_t)MAX_Q27 << 32) | (int64_t)MAX_Q27;
   int64_t dw_MIN_Q27 = ((int64_t)MIN_Q27 << 32) | (int64_t)MIN_Q27;

   /* 32-to-16 bit conversion */
   if(BYTE_DOWN_CONV == conv_mode)
   {

#if ((defined __hexagon__) || (defined __qdsp6__))

      buf_ptr_64 = (int64_t *)(ptr_src_buf);      /* Input buffer  */
      buf_ptr_32 = (int32_t *)(ptr_out_buf);     /* Output buffer */

      /* If input buf addr is 8 byte aligned and out buf addr is 4 byte aligned
         then only perform the vector operation
      */
      if( (0 == ((uint32_t)buf_ptr_64 & 0x7)) && 
          (0 == ((uint32_t)buf_ptr_32 & 0x3))
          )
      {
         for(samp = total_num_samples; samp >= 2 ; samp -= 2)
         {
            int64_t temp64;
            /* Convert from Q28 to Q15*/
            /*
            * Saturate to 28 bits*/
            temp64 = Q6_P_vminw_PP(*buf_ptr_64++, dw_MAX_Q27);
            temp64 = Q6_P_vmaxw_PP(temp64, dw_MIN_Q27);

            /* Right shift each of the 32 bit words into 16 bits and 
               store the two lower halfwords into destination
            */
            *buf_ptr_32++ = Q6_R_vasrw_PI(temp64, BYTE_UPDOWN_CONV_SHIFT_FACT);
         }
   
         /* If the number of samples are odd */
         if(samp)
         {
            int32_t temp32;

            buf_ptr_src_16 = (int16_t *)buf_ptr_32;
            buf_ptr_src2_32 = (int32_t *)buf_ptr_64;

            /*saturate to 28 bits*/
            temp32 = Q6_R_max_RR(*buf_ptr_src2_32, (int32_t)MIN_Q27);
            temp32 = Q6_R_min_RR(temp32, (int32_t)MAX_Q27);

            /* Q28 -> Q15 with rounding */
            *buf_ptr_src_16 = Q6_R_asr_RI(temp32, BYTE_UPDOWN_CONV_SHIFT_FACT);
         }
      }
      else /* if either of buf addr is not byte aligned as required for vectorization */
      {
         int32_t temp32;

         buf_ptr_src_16 = (int16_t *)buf_ptr_32;
         buf_ptr_src2_32 = (int32_t *)buf_ptr_64;

         for(samp = 0; samp < total_num_samples; samp++)
         {
            /*saturate to 28 bits*/
            temp32 = Q6_R_max_RR(*buf_ptr_src2_32++, (int32_t)MIN_Q27);
            temp32 = Q6_R_min_RR(temp32, (int32_t)MAX_Q27);
            /* Q28 to Q15 */
            /* First right shift followed by rounding operation */
            *buf_ptr_src_16++ = Q6_R_asr_RI(temp32, BYTE_UPDOWN_CONV_SHIFT_FACT);
         }
      }
      
#else
      /*----------- Non Q6 Version ---------------*/
      buf_ptr_16 = (int16_t *)ptr_out_buf;
      buf_ptr_32 = (int32_t *)ptr_src_buf;
   
       /* Q28 to Q15 */        
      for(samp = 0; samp < total_num_samples; samp++)
      {        
         int32_t temp32 = *buf_ptr_32++;
         /*saturate to 28 bits*/
         temp32 = (temp32 < (int32_t)MIN_Q27)? (int32_t)MIN_Q27: temp32;
         temp32 = (temp32 > (int32_t)MAX_Q27)? (int32_t)MAX_Q27: temp32;
         /*Right shift to Q15*/
         *buf_ptr_16++ = (int16_t)( temp32 >> BYTE_UPDOWN_CONV_SHIFT_FACT);
      }
#endif /* __qdsp6__ */
        
   }
   else // Q15 -> Q28, NO inplace conversion, 
   {
     
#if ((defined __hexagon__) || (defined __qdsp6__))
      
      buf_ptr_32 = (int32_t *)(ptr_src_buf);            /* Input buffer  */
      buf_ptr_64 = (int64_t *)(ptr_out_buf);            /* Output buffer */

      /* Convert from Q15 to Q28, conversion output in scratch buffer
      */

      /* If Output buf addr is 8 byte aligned and input buf addr is 4 byte aligned
         then only perform the vector operation
      */
      if( (0 == ((uint32_t)buf_ptr_64 & 0x7)) && 
          (0 == ((uint32_t)buf_ptr_32 & 0x3))
          )
      {
         for(samp = total_num_samples; samp >= 2; samp -= 2)
         {
            /* Sign extend two 16-bit words in to two 32-bit words */
            *buf_ptr_64 = Q6_P_vsxthw_R(*buf_ptr_32++);
               
            /* Shift left the result to convert it to Q28
            */
            *buf_ptr_64 = Q6_P_vaslw_PI(*buf_ptr_64, BYTE_UPDOWN_CONV_SHIFT_FACT);
            buf_ptr_64++;
         }

         /* If the number of samples per channel is odd */
         if(samp)
         {
            buf_ptr_src_16 = (int16_t *)buf_ptr_32;
            buf_ptr_src2_32 = (int32_t *)buf_ptr_64;
   
            *buf_ptr_src2_32 = (int32_t)(*buf_ptr_src_16 << BYTE_UPDOWN_CONV_SHIFT_FACT);
         }
      }
      else /* if either of buf addr is not byte aligned as required for vectorization */
      {
         buf_ptr_16 = (int16_t *)ptr_src_buf;
         buf_ptr_32 = (int32_t *)ptr_out_buf;

         /* Q15 -> Q28 conversion */
         for(samp = 0; samp < total_num_samples; samp++)
         {
            (*buf_ptr_32++) = (int32_t)( (*buf_ptr_16++) << BYTE_UPDOWN_CONV_SHIFT_FACT);
         }
      }
#else
      /*----------- Non Q6 Version ---------------*/
      
      buf_ptr_16 = (int16_t *)ptr_src_buf;
      buf_ptr_32 = (int32_t *)ptr_out_buf;

      /* Q15 -> Q28 conversion */
      for(samp = 0; samp < total_num_samples; samp++)
      {
         (*buf_ptr_32++) = (int32_t)( (*buf_ptr_16++) << BYTE_UPDOWN_CONV_SHIFT_FACT);
      }
      
#endif /* __qdsp6__ */
   }
}

#if __qdsp6__
#define USE_Q6_SPECIFIC_CODE
#endif

/**
 * This function is for conversion of bytes per sample
 * for the data buf pointed to by ptr_src_buf.
 *
 * data in ptr_src_buf is either 32 bits or 24 bits or 16 bits
 * when 32 Q formats are Q23, Q27, Q31. Q23, Q27 are not tested.
 * when 24 Q format is Q23
 * when 16 Q format is Q15
 *
 * Output is 16 bits and Q15.
 *
 * @param[in] ptr_src_buf, ptr to source buf to be converted,
 *       assumes channel interleaved data
 * @param[out] ptr_out_buf, pointer to output buffer, assumes
 *       interleaved data, should be sufficiently large to
 *       hold the output data.
 * @param[in] total_num_samples, total number of samples (includes all channels)
 * @param[in] in_word_size, word size in bits 32, 24, 16 of input samples
 * @param[in] in_Q_fact, Q factor of input.
 *
 * @return error status
 *
 */

ar_result_t byte_convertor_intlv_16_out(int8_t * ptr_src_buf, int8_t * ptr_out_buf, uint32_t total_num_samples,
      uint16_t in_word_size, uint16_t in_Q_fact)
{
   int32_t   samp;
   int32_t   *buf_ptr_src2_32, *buf_ptr_32;
   int16_t   *buf_ptr_16, *buf_ptr_src_16;
   int64_t   *buf_ptr_64;

   //error checks
   if ( ((24 == in_word_size) && (23 != in_Q_fact)) ||
         ((16 == in_word_size) && (15 != in_Q_fact)) ||
         ((32 == in_word_size) && (31 != in_Q_fact)) )
   {
      return AR_EUNSUPPORTED;
   }


   const uint32_t shift = (in_Q_fact - 15); // (Qn - Q15)


   if (32 == in_word_size)
   {
   #ifdef USE_Q6_SPECIFIC_CODE
      buf_ptr_64 = (int64_t *)(ptr_src_buf);      /* Input buffer  */
      buf_ptr_32 = (int32_t *)(ptr_out_buf);     /* Output buffer */

      /* If input buf addr is 8 byte aligned and out buf addr is 4 byte aligned
         then only perform the vector operation
      */
      if( (0 == ((uint32_t)buf_ptr_64 & 0x7)) &&
          (0 == ((uint32_t)buf_ptr_32 & 0x3))
          )
      {
         for(samp = total_num_samples; samp >= 2 ; samp -= 2)
         {
            int64_t temp64 = *buf_ptr_64++;
            /* Convert from Q31 to Q15*/

            /* Right shift each of the 32 bit words into 16 bits and
               store the two lower halfwords into destination
            */
            *buf_ptr_32++ = Q6_R_vasrw_PR(temp64, shift);
         }

         /* If the number of samples are odd, following loop will handle. */
         total_num_samples = samp;
      }
       /* if either of buf addr is not byte aligned as required for vectorization
        * or for the remaining samples in case it's odd*/
      {
         int32_t temp32;

         buf_ptr_src_16 = (int16_t *)buf_ptr_32;
         buf_ptr_src2_32 = (int32_t *)buf_ptr_64;

         for(samp = 0; samp < total_num_samples; samp++)
         {
            temp32 = *buf_ptr_src2_32++;
            /* Q28 to Q15 */
            /* First right shift followed by rounding operation */
            *buf_ptr_src_16++ = Q6_R_asr_RR(temp32, shift);
         }
      }

   #else
      /*----------- Non Q6 Version ---------------*/
      buf_ptr_16 = (int16_t *)ptr_out_buf;
      buf_ptr_32 = (int32_t *)ptr_src_buf;

       /* Qn to Q15 */
      //shift is 31-15 = 16 if in_Q_fact is 31.
      for(samp = 0; samp < total_num_samples; samp++)
      {
         int32_t temp32 = *buf_ptr_32++;
         /*Right shift to Q15*/
         *buf_ptr_16++ = (int16_t)( temp32 >> shift);
      }
   #endif /* USE_Q6_SPECIFIC_CODE */
   }
   else if (24 == in_word_size)
   {
      buf_ptr_16 = (int16_t *)ptr_out_buf;
      uint8_t *src = (uint8_t*)ptr_src_buf;
       /* Qn to Q15 */
      for(samp = 0; samp < total_num_samples; samp++)
      {
         int32_t num32;
         uint32_t tem32;

         num32 = 0;
         tem32 = *src++;
         num32 = num32 | (tem32);
         tem32 = *src++;
         num32 = num32 | (tem32<<8);
         tem32 = *src++;
         num32 = num32 | (tem32<<16);
         /*Right shift to Q15*/
         *buf_ptr_16++ = (int16_t)( num32 >> shift);
      }
   }
   else if(16 == in_word_size)
   {
      //no processing is required.
      if (ptr_out_buf != ptr_src_buf)
      {
         memsmove(ptr_out_buf, total_num_samples*sizeof(int16_t), ptr_src_buf, total_num_samples*sizeof(int16_t));
      }
   }
   else
   {
      return AR_EUNSUPPORTED;
   }
   return AR_EOK;
}

/**
 * This function is for conversion of bytes per sample
 * for the data buf pointed to by ptr_src_buf.
 *
 * Input can be Q15 in 16 bits, Q23 in 24 bit words or
 *   (Q23 in 32 bit words or Q27 in 32 bits or) Q31 in 32 bits.
 * Output is Q27 in 32 bit words.
 *
 * @param[in] ptr_src_buf, ptr to source buf to be converted,
 *       assumes channel interleaved data
 * @param[out] ptr_out_buf, pointer to output buffer, asumes
 *       interleaved data, should be sufficiently large to
 *       hold the output data.
 * @param[in] total_num_samples, total number of samples
 * @param[in] in_word_size, input word size: 16 or 24 bits or 32 bits
 * @param[in] in_Q_fact Q15 or Q23 or Q27 or Q31.
 *
 * NOTE : This up conversion is NOT in place in all cases. ptr_src_buf and
 * ptr_out_buf should point to different memories.
 *
 * For this conversion, output buffer should be
 * sufficiently large to hold the up converted data.
 *
 * @return error status
 */

ar_result_t byte_convertor_intlv_32_out(int8_t *ptr_src_buf, int8_t *ptr_out_buf, uint32_t total_num_samples,
      uint16_t in_word_size, uint16_t in_Q_fact)
{
   int32_t   samp;
   int32_t   *buf_ptr_32;
   int16_t   *buf_ptr_16;
   int64_t   *buf_ptr_64, *in_buf_ptr_64;

   //error checks
   if ( ( (16 == in_word_size) && (15 != in_Q_fact) ) ||
         ( (24 == in_word_size) && (23 != in_Q_fact) ) ||
         ( (32 == in_word_size)  &&
               ( (31 != in_Q_fact) )
          )  )
   {
      return AR_EUNSUPPORTED;
   }

   uint32_t shift ;

   if (16 == in_word_size)
   {
      // no in-place
      if (ptr_src_buf == ptr_out_buf)
      {
         return AR_EBADPARAM;
      }

      shift = (PCM_Q_FACTOR_27 - in_Q_fact); // (Q27 - Qn)

      #ifdef USE_Q6_SPECIFIC_CODE

      buf_ptr_32 = (int32_t *)(ptr_src_buf);            /* Input buffer  */
      buf_ptr_64 = (int64_t *)(ptr_out_buf);            /* Output buffer */

      /* Convert from Q15 to Q27, conversion output in scratch buffer
      */

      /* If Output buf addr is 8 byte aligned and input buf addr is 4 byte aligned
         then only perform the vector operation
      */
      if( (0 == ((uint32_t)buf_ptr_64 & 0x7)) &&
          (0 == ((uint32_t)buf_ptr_32 & 0x3))
          )
      {
         for(samp = total_num_samples; samp >= 2; samp -= 2)
         {
            /* Sign extend two 16-bit words in to two 32-bit words */
            *buf_ptr_64 = Q6_P_vsxthw_R(*buf_ptr_32++);

            /* Shift left the result to convert it to Q27
            */
            *buf_ptr_64 = Q6_P_vaslw_PR(*buf_ptr_64, shift);
            buf_ptr_64++;
         }

         /* If the number of samples per channel is odd */
         total_num_samples = samp;
         }
      /* if either of buf addr is not byte aligned as required for vectorization
       * or for odd samples */
      {
         buf_ptr_16 = (int16_t *)buf_ptr_32;
         buf_ptr_32 = (int32_t *)buf_ptr_64;

         /* Q15 -> Q27 conversion */
         for(samp = 0; samp < total_num_samples; samp++)
         {
            (*buf_ptr_32++) = (int32_t)( (*buf_ptr_16++) << shift);
         }
      }
      #else
      /*----------- Non Q6 Version ---------------*/

      buf_ptr_16 = (int16_t *)ptr_src_buf;
      buf_ptr_32 = (int32_t *)ptr_out_buf;

      /* Q15 -> Q27 conversion */
      for(samp = 0; samp < total_num_samples; samp++)
      {
         (*buf_ptr_32++) = (int32_t)( (*buf_ptr_16++) << shift);
      }

      #endif /* USE_Q6_SPECIFIC_CODE */
   }
   else if (24 == in_word_size)
   {
      shift = (PCM_Q_FACTOR_27 - in_Q_fact); // (Q27 - Qn)

      /*----------- Non Q6 Version ---------------*/

      buf_ptr_32 = (int32_t *)ptr_out_buf;
      uint8_t *src = (uint8_t*)ptr_src_buf;
      //POSAL_ASSERT(shift <= 8);

      /* Q23 -> Q27  conversion */
      for(samp = 0; samp < total_num_samples; samp++)
      {

         int32_t num32;
         uint32_t tem32;
         num32 = 0;
         tem32 = *src++;
         num32 = num32 | (tem32<<8);
         tem32 = *src++;
         num32 = num32 | (tem32<<16);
         tem32 = *src++;
         num32 = num32 | (tem32<<24);

         (*buf_ptr_32++) = (int32_t)( (num32) >> (shift));
      }
   }
   else if (32 == in_word_size)
   {
      if (31 == in_Q_fact)
      {
         shift = (in_Q_fact - PCM_Q_FACTOR_27); // (Qn - Q27)

         #ifdef USE_Q6_SPECIFIC_CODE
         in_buf_ptr_64 = (int64_t *)(ptr_src_buf);            /* Input buffer  */
         buf_ptr_64 = (int64_t *)(ptr_out_buf);            /* Output buffer */

         /* Convert from Q15 to Q27, conversion output in scratch buffer
         */

         /* If Output buf addr is 8 byte aligned and input buf addr is 8 byte aligned
            then only perform the vector operation
         */
         if( (0 == ((uint32_t)buf_ptr_64 & 0x7)) &&
             (0 == ((uint32_t)in_buf_ptr_64 & 0x7))
             )
         {
            for(samp = total_num_samples; samp >= 2; samp -= 2)
            {
               /* Shift left the result to convert it to Q27
               */
               *buf_ptr_64 = Q6_P_vasrw_PR(*in_buf_ptr_64++, shift);
               buf_ptr_64++;
            }

            /* If the number of samples per channel is odd */
            total_num_samples = samp;
         }
         /* if either of buf addr is not byte aligned as required for vectorization
          * or for odd samples */
         {
            int32_t *in_buf_ptr_32 = (int32_t *)in_buf_ptr_64;
            buf_ptr_32 = (int32_t *)buf_ptr_64;

            /* Q15 -> Q27 conversion */
            for(samp = 0; samp < total_num_samples; samp++)
            {
               (*buf_ptr_32++) = (int32_t)( (*in_buf_ptr_32++) >> shift);
            }
         }
         #else //USE_Q6_SPECIFIC_CODE
            /*----------- Non Q6 Version ---------------*/

         int32_t *in_buf_ptr_32 = (int32_t *)ptr_src_buf;
         buf_ptr_32 = (int32_t *)ptr_out_buf;

         /* Q31 -> Q27 conversion */
         for(samp = 0; samp < total_num_samples; samp++)
         {
            (*buf_ptr_32++) = (int32_t)( (*in_buf_ptr_32++) >> shift);
         }
         #endif //USE_Q6_SPECIFIC_CODE

      }
   }
   else
   {
      return AR_EUNSUPPORTED;
   }

   return AR_EOK;
}

/**
 * This function is for conversion of bytes per sample
 * for the data buf pointed to by ptr_src_buf.
 *
 * Input can be Q15 in 16 bits, Q23 in 24 bit words or
 *   (Q23 in 32 bit words or Q27 in 32 bits or) Q31 in 32 bits.
 * Output is Q31 in 32 bit words.
 *
 * @param[in] ptr_src_buf, ptr to source buf to be converted,
 *       assumes channel interleaved data
 * @param[out] ptr_out_buf, pointer to output buffer, asumes
 *       interleaved data, should be sufficiently large to
 *       hold the output data.
 * @param[in] total_num_samples, total number of samples
 * @param[in] in_word_size, input word size: 16 or 24 bits or 32 bits
 * @param[in] in_Q_fact Q15 or Q23 or Q27 or Q31.
 *
 * NOTE : This up conversion is NOT in place in all cases. ptr_src_buf and
 * ptr_out_buf should point to different memories.
 *
 * For this conversion, output buffer should be
 * sufficiently large to hold the up converted data.
 *
 * @return error status
 */

ar_result_t byte_convertor_intlv_32_out_Q31(int8_t *ptr_src_buf, int8_t *ptr_out_buf, uint32_t total_num_samples,
      uint16_t in_word_size, uint16_t in_Q_fact)
{
   int32_t   samp;
   int32_t   *buf_ptr_32;
   int16_t   *buf_ptr_16;
   int64_t   *buf_ptr_64, *in_buf_ptr_64;

   //error checks
   if ( ( (16 == in_word_size) && (15 != in_Q_fact) ) ||
         ( (24 == in_word_size) && (23 != in_Q_fact) ) ||
         ( (32 == in_word_size)  &&
               ( (31 != in_Q_fact) )
          )  )
   {
      return AR_EUNSUPPORTED;
   }

   uint32_t shift ;

   if (16 == in_word_size)
   {
      // no in-place
      if (ptr_src_buf == ptr_out_buf)
      {
         return AR_EBADPARAM;
      }

      shift = (PCM_Q_FACTOR_31 - in_Q_fact); // (Q31 - Qn)

      #ifdef USE_Q6_SPECIFIC_CODE

      buf_ptr_32 = (int32_t *)(ptr_src_buf);            /* Input buffer  */
      buf_ptr_64 = (int64_t *)(ptr_out_buf);            /* Output buffer */

      /* Convert from Q15 to Q31, conversion output in scratch buffer
      */

      /* If Output buf addr is 8 byte aligned and input buf addr is 4 byte aligned
         then only perform the vector operation
      */
      if( (0 == ((uint32_t)buf_ptr_64 & 0x7)) &&
          (0 == ((uint32_t)buf_ptr_32 & 0x3))
          )
      {
         for(samp = total_num_samples; samp >= 2; samp -= 2)
         {
            /* Sign extend two 16-bit words in to two 32-bit words */
            *buf_ptr_64 = Q6_P_vsxthw_R(*buf_ptr_32++);

            /* Shift left the result to convert it to Q27
            */
            *buf_ptr_64 = Q6_P_vaslw_PR(*buf_ptr_64, shift);
            buf_ptr_64++;
         }

         /* If the number of samples per channel is odd */
         total_num_samples = samp;
         }
      /* if either of buf addr is not byte aligned as required for vectorization
       * or for odd samples */
      {
         buf_ptr_16 = (int16_t *)buf_ptr_32;
         buf_ptr_32 = (int32_t *)buf_ptr_64;

         /* Q15 -> Q31 conversion */
         for(samp = 0; samp < total_num_samples; samp++)
         {
            (*buf_ptr_32++) = (int32_t)( (*buf_ptr_16++) << shift);
         }
      }
      #else
      /*----------- Non Q6 Version ---------------*/

      buf_ptr_16 = (int16_t *)ptr_src_buf;
      buf_ptr_32 = (int32_t *)ptr_out_buf;

      /* Q15 -> Q31 conversion */
      for(samp = 0; samp < total_num_samples; samp++)
      {
         (*buf_ptr_32++) = (int32_t)( (*buf_ptr_16++) << shift);
      }

      #endif /* USE_Q6_SPECIFIC_CODE */
   }
   else if (24 == in_word_size)
   {
      shift = (PCM_Q_FACTOR_31 - in_Q_fact); // (Q31 - Qn)

      /*----------- Non Q6 Version ---------------*/

      buf_ptr_32 = (int32_t *)ptr_out_buf;
      uint8_t *src = (uint8_t*)ptr_src_buf;
      //POSAL_ASSERT(shift <= 8);

      /* Q23 -> Q31  conversion */
      for(samp = 0; samp < total_num_samples; samp++)
      {

         int32_t num32;
         uint32_t tem32;
         num32 = 0;
         tem32 = *src++;
         num32 = num32 | (tem32<<8);
         tem32 = *src++;
         num32 = num32 | (tem32<<16);
         tem32 = *src++;
         num32 = num32 | (tem32<<24);

         (*buf_ptr_32++) = (int32_t)( (num32) >> (shift));
      }
   }
   else if (32 == in_word_size)
   {
      if (31 == in_Q_fact)
      {
         shift = (in_Q_fact - PCM_Q_FACTOR_31); // (Qn - Q31)

         #ifdef USE_Q6_SPECIFIC_CODE
         in_buf_ptr_64 = (int64_t *)(ptr_src_buf);            /* Input buffer  */
         buf_ptr_64 = (int64_t *)(ptr_out_buf);            /* Output buffer */

         /* Convert from Q15 to Q31, conversion output in scratch buffer
         */

         /* If Output buf addr is 8 byte aligned and input buf addr is 8 byte aligned
            then only perform the vector operation
         */
         if( (0 == ((uint32_t)buf_ptr_64 & 0x7)) &&
             (0 == ((uint32_t)in_buf_ptr_64 & 0x7))
             )
         {
            for(samp = total_num_samples; samp >= 2; samp -= 2)
            {
               /* Shift left the result to convert it to Q27
               */
               *buf_ptr_64 = Q6_P_vasrw_PR(*in_buf_ptr_64++, shift);
               buf_ptr_64++;
            }

            /* If the number of samples per channel is odd */
            total_num_samples = samp;
         }
         /* if either of buf addr is not byte aligned as required for vectorization
          * or for odd samples */
         {
            int32_t *in_buf_ptr_32 = (int32_t *)in_buf_ptr_64;
            buf_ptr_32 = (int32_t *)buf_ptr_64;

            /* Q15 -> Q31 conversion */
            for(samp = 0; samp < total_num_samples; samp++)
            {
               (*buf_ptr_32++) = (int32_t)( (*in_buf_ptr_32++) >> shift);
            }
         }
         #else //USE_Q6_SPECIFIC_CODE
            /*----------- Non Q6 Version ---------------*/

         int32_t *in_buf_ptr_32 = (int32_t *)ptr_src_buf;
         buf_ptr_32 = (int32_t *)ptr_out_buf;

         /* Q31 -> Q31 conversion */
         for(samp = 0; samp < total_num_samples; samp++)
         {
            (*buf_ptr_32++) = (int32_t)( (*in_buf_ptr_32++) >> shift);
         }
         #endif //USE_Q6_SPECIFIC_CODE

      }
   }
   else
   {
      return AR_EUNSUPPORTED;
   }

   return AR_EOK;
}

/**
 * This function is for conversion of bytes per sample
 * for the data buf pointed to by ptr_src_buf while interleaving
 * the given samples.
 *
 * data in ptr_src_buf is 16 bits Q15
 * output in ptr_out_buf can be 16, 24 or 32 bit words.
 * when 32 Q formats are (Q23, Q27,)  Q31 Q31 only supported
 * when 24 Q format is Q23
 * when 16 Q format is Q15
 *
 * input and output buffers cannot overlap.
 *
 * @param[in] ptr_src_buf, ptr to source buf to be converted,
 *       assumes channel interleaved data
 * @param[out] ptr_out_buf, pointer to output buffer, assumes
 *       interleaved data, should be sufficiently large to
 *       hold the output data.
 * @param[in] num_sampl_p_ch, number of samples per channel
 * @param[in] num_channels, number of channels in the input.
 * @param[in] out_word_size, word size in bits 32, 24, 16 of input samples
 * @param[in] out_Q_fact, Q factor of output Q23, Q27, Q31
 * @param[in] out_bits_p_sample, 16,24,32. When output word size is 32 &
 *            Q factor is Q31, it can be just 24 bits (last 8 bits zero).
 * @param[in] is_in_intlvd, whether the samples are interleaved. If not,
 * the channels are assumed to be separated by num_sampl_p_ch channels.
 * @param[in] channel_spacing_samples, channel spacing in samples
 *
 * @return error status
 *
 * input -> output
 * 16    ->    16
 * 16    ->    24
 * 16    ->    32
 *
 */

ar_result_t byte_convertor_with_intlv_16_in(int8_t * ptr_src_buf, int8_t * ptr_out_buf, uint32_t num_sampl_p_ch,
      uint32_t num_channels, uint16_t out_word_size, uint16_t out_Q_fact, uint16_t out_bits_p_sample,
      bool_t is_in_intlvd, uint32_t channel_spacing_samples )
{
   //error checks
   if ( ( (16 == out_word_size) && (15 != out_Q_fact) ) ||
         ( (24 == out_word_size) && (23 != out_Q_fact) ) ||
         ( (32 == out_word_size)  && ( (31 != out_Q_fact) ) )  )
   {
      return AR_EUNSUPPORTED;
   }

   // no in-place
   if (ptr_src_buf == ptr_out_buf)
   {
      return AR_EBADPARAM;
   }

   int16_t *src_ptr_16 = (int16_t*)ptr_src_buf;
   uint32_t shift;
   uint32_t i;
   uint32_t ch;

   if (is_in_intlvd)
   {
      uint32_t total_samples = num_sampl_p_ch*num_channels;
      if (16 == out_word_size)
      {
         memscpy(ptr_out_buf,total_samples*sizeof(int16_t), ptr_src_buf, total_samples*sizeof(int16_t));
      }
      else if (24 == out_word_size)
      {
         shift = out_Q_fact - 15;
         int32_t num;
         for (i = 0; i < total_samples; i++)
         {
            num = (int32_t) (*src_ptr_16++);
            num <<= shift;
            *ptr_out_buf++ = (int8_t) ((uint32_t)num & 0x000000FF) ;
            *ptr_out_buf++ = (int8_t) (((uint32_t)num & 0x0000FF00)>>8) ;
            *ptr_out_buf++ = (int8_t) (((uint32_t)num & 0x00FF0000)>>16) ;
         }
      }
      else if (32 == out_word_size)
      {
         shift = out_Q_fact - 15;
         int32_t *out_ptr_32 = (int32_t*) ptr_out_buf;
         uint32_t mask = 0xFFFFFFFF<<(out_word_size-out_bits_p_sample);

         for (i = 0; i < total_samples; i++)
         {
            *out_ptr_32++ = ( ((int32_t)(*src_ptr_16++)) << shift) & mask ;
         }
      }
   }
   else //input is deinterleaved
   {
      if (16 == out_word_size)
      {
         int16_t *out_ptr_16 = (int16_t*) ptr_out_buf;
         for (ch = 0; ch < num_channels; ch++)
         {
            for (i = 0; i < num_sampl_p_ch; i++)
            {
               *(out_ptr_16 + i*num_channels + ch) = (int16_t) ( *(src_ptr_16 + ch*channel_spacing_samples + i));
            }
         }
      }
      else if (24 == out_word_size)
      {
         int32_t num;
         shift = out_Q_fact - 15;
         uint8_t *ptr;
         for (ch = 0; ch < num_channels; ch++)
         {
            for (i = 0; i < num_sampl_p_ch; i++)
            {
               num = (int32_t) (*(src_ptr_16 + ch*channel_spacing_samples + i));
               num <<= shift;
               ptr = (uint8_t *)(ptr_out_buf + i*num_channels*(out_word_size/8) + ch*(out_word_size/8));
               *ptr++ = (uint8_t) ((uint32_t)num & 0x000000FF) ;
               *ptr++ = (uint8_t) ((((uint32_t)num & 0x0000FF00))>>8) ;
               *ptr++ = (uint8_t) ((((uint32_t)num & 0x00FF0000))>>16) ;
            }
         }
      }
      else if (32 == out_word_size)
      {
         shift = out_Q_fact - 15;
         int32_t *out_ptr_32 = (int32_t*) ptr_out_buf;

         uint32_t mask = 0xFFFFFFFF<<(out_word_size-out_bits_p_sample);

         for (ch = 0; ch < num_channels; ch++)
         {
            for (i = 0; i < num_sampl_p_ch; i++)
            {
               *(out_ptr_32 + i*num_channels + ch) = ( ((int32_t)(*(src_ptr_16 + ch*channel_spacing_samples + i))) << shift) & mask;
            }
         }
      }
   }

   return AR_EOK;
}

/**
 * This function is for conversion of bytes per sample
 * for the data buf pointed to by ptr_src_buf while interleaving
 * the given samples.
 *
 * data in ptr_src_buf is 32 bits Q27 (ELITE_32BIT_PCM_Q_FORMAT)
 * output in ptr_out_buf can be 16, 24 or 32 bit words.
 * when 32 Q formats are (Q23, Q27,) Q31. Q31 only supported
 * when 24 Q format is Q23
 * when 16 Q format is Q15
 *
 * input and output buffers cannot overlap.
 *
 * @param[in] ptr_src_buf, ptr to source buf to be converted,
 *       assumes channel interleaved data
 * @param[out] ptr_out_buf, pointer to output buffer, assumes
 *       interleaved data, should be sufficiently large to
 *       hold the output data.
 * @param[in] num_sampl_p_ch, number of samples per channel
 * @param[in] num_channels, number of channels in the input.
 * @param[in] out_word_size, word size in bits 32, 24, 16 of input samples
 * @param[in] out_Q_fact, Q factor of output Q15, Q23, Q27, Q31
 * @param[in] out_bits_p_sample, 16,24,32. When output word size is 32 &
 *            Q factor is Q31, it can be just 24 bits (last 8 bits zero).
 * @param[in] is_in_intlvd, whether the samples are interleaved. If not,
 * the channels are assumed to be separated by num_sampl_p_ch channels.
 * @param[in] channel_spacing_samples, channel spacing in samples
 *
 * @return error status
 *
 * input -> output
 * 32    ->    16
 * 32    ->    24
 * 32    ->    32
 *
 */

ar_result_t byte_convertor_with_intlv_32_in(int8_t * ptr_src_buf, int8_t * ptr_out_buf, uint32_t num_sampl_p_ch,
      uint32_t num_channels, uint16_t out_word_size, uint16_t out_Q_fact,
      uint16_t out_bits_p_sample, bool_t is_in_intlvd, uint32_t channel_spacing_samples)
{
   //error checks
   if ( ( (16 == out_word_size) && (15 != out_Q_fact) ) ||
         ( (24 == out_word_size) && (23 != out_Q_fact) ) ||
         ( (32 == out_word_size)  &&
               ( (23 != out_Q_fact) &&
                     (PCM_Q_FACTOR_27 != out_Q_fact) &&
                     (31 != out_Q_fact) )
          )  )
   {
      return AR_EUNSUPPORTED;
   }

   // no in-place
   if (ptr_src_buf == ptr_out_buf)
   {
      return AR_EBADPARAM;
   }

   int32_t *src_ptr_32 = (int32_t*)ptr_src_buf;
   uint32_t shift;
   uint32_t i;
   uint32_t ch;
   int32_t temp32, samp;
   int32_t min = ( - (1 << PCM_Q_FACTOR_27));
   int32_t max = ((1 << PCM_Q_FACTOR_27) - 1);

   int64_t dw_max = ((int64_t)max << 32) | (int64_t)max;
   int64_t dw_min = ((int64_t)min << 32) | (int64_t)min;

   if (is_in_intlvd)
   {
      uint32_t total_samples = num_sampl_p_ch*num_channels;
      if (16 == out_word_size)
      {
         shift = PCM_Q_FACTOR_27 - out_Q_fact;

#ifdef USE_Q6_SPECIFIC_CODE
         int64_t *buf_ptr_64 = (int64_t *)(ptr_src_buf);      /* Input buffer  */
         int32_t *buf_ptr_32 = (int32_t *)(ptr_out_buf);     /* Output buffer */

         /* If input buf addr is 8 byte aligned and out buf addr is 4 byte aligned
            then only perform the vector operation
         */
         if( (0 == ((uint32_t)buf_ptr_64 & 0x7)) &&
             (0 == ((uint32_t)buf_ptr_32 & 0x3))
             )
         {
            for(samp = total_samples; samp >= 2 ; samp -= 2)
            {
               int64_t temp64;
               /* Convert from Q27 to Q15*/
               /*
               * Saturate to 27 bits*/
               temp64 = Q6_P_vminw_PP(*buf_ptr_64++, dw_max);
               temp64 = Q6_P_vmaxw_PP(temp64, dw_min);

               /* Right shift each of the 32 bit words into 16 bits and
                  store the two lower halfwords into destination
               */
               *buf_ptr_32++ = Q6_R_vasrw_PR(temp64, shift);
            }

            /* If the number of samples are odd, following loop will handle. */
            total_samples = samp;
         }
          /* if either of buf addr is not byte aligned as required for vectorization
           * or for the remaining samples in case it's odd*/
         {
            int32_t temp32;

            int16_t *buf_ptr_src_16 = (int16_t *)buf_ptr_32;
            int32_t *buf_ptr_src2_32 = (int32_t *)buf_ptr_64;

            for(samp = 0; samp < total_samples; samp++)
            {
               temp32 = *buf_ptr_src2_32++;
               /*saturate to 27 bits*/
               temp32 = Q6_R_max_RR(*buf_ptr_src2_32, (int32_t)min);
               temp32 = Q6_R_min_RR(temp32, (int32_t)max);

               /* Q27 to Q15 */
               /* First right shift followed by rounding operation */
               *buf_ptr_src_16++ = Q6_R_asr_RR(temp32, shift);
            }
         }

#else
         /*----------- Non Q6 Version ---------------*/
         int16_t *out_ptr_16 = (int16_t*) ptr_out_buf;
         for (i = 0; i < total_samples; i++)
         {
            temp32 = *src_ptr_32++;
            /*saturate to Q factor */
            temp32 = (temp32 < min)? min: temp32;
            temp32 = (temp32 > max)? max: temp32;

            *out_ptr_16++ = (int16_t) ((temp32) >> shift);
         }
#endif //USE_Q6_SPECIFIC_CODE
      }
      else if (24 == out_word_size)
      {
         shift = PCM_Q_FACTOR_27 - out_Q_fact;
         for (i = 0; i < total_samples; i++)
         {
            temp32 = *src_ptr_32++;
            /*saturate to Q factor */
            temp32 = (temp32 < min)? min: temp32;
            temp32 = (temp32 > max)? max: temp32;

            temp32 >>= shift;

            *ptr_out_buf++ = (uint8_t) ( (uint32_t)temp32 & 0x000000FF) ;
            *ptr_out_buf++ = (uint8_t) (( (uint32_t)temp32 & 0x0000FF00)>>8) ;
            *ptr_out_buf++ = (uint8_t) (( (uint32_t)temp32 & 0x00FF0000)>>16) ;
         }
      }
      else if (32 == out_word_size)
      {
         int32_t *out_ptr_32 = (int32_t*) ptr_out_buf;

         shift = out_Q_fact - PCM_Q_FACTOR_27;

         uint32_t mask = 0xFFFFFFFF<<(out_word_size-out_bits_p_sample);

         for (i = 0; i < total_samples; i++)
         {
            *out_ptr_32++ = ((*src_ptr_32++)<<shift) & mask;
         }
      }
   }
   else //input is deinterleaved
   {
      if (16 == out_word_size)
      {
         shift = PCM_Q_FACTOR_27 - out_Q_fact;
         int16_t *out_ptr_16 = (int16_t*) ptr_out_buf;
         for (ch = 0; ch < num_channels; ch++)
         {
            for (i = 0; i < num_sampl_p_ch; i++)
            {
               temp32 = (int32_t)(*(src_ptr_32 + ch*channel_spacing_samples + i));
               /*saturate to Q factor */
               temp32 = (temp32 < min)? min: temp32;
               temp32 = (temp32 > max)? max: temp32;

               *(out_ptr_16 + i*num_channels + ch) = (int16_t) ( (temp32) >> shift);
            }
         }
      }
      else if (24 == out_word_size)
      {
         shift = PCM_Q_FACTOR_27 - out_Q_fact;
         uint8_t *ptr;
         for (ch = 0; ch < num_channels; ch++)
         {
            for (i = 0; i < num_sampl_p_ch; i++)
            {
               temp32 = *(src_ptr_32 + ch*channel_spacing_samples + i);
               /*saturate to Q factor */
               temp32 = (temp32 < min)? min: temp32;
               temp32 = (temp32 > max)? max: temp32;

               temp32 >>= shift;

               ptr = (uint8_t *)(ptr_out_buf + i*num_channels*out_word_size/8 + ch*out_word_size/8);

               *ptr++ = (uint8_t) ( (uint32_t)temp32 & 0x000000FF) ;
               *ptr++ = (uint8_t) ( ((uint32_t)temp32 & 0x0000FF00)>>8) ;
               *ptr++ = (uint8_t) ( ((uint32_t)temp32 & 0x00FF0000)>>16) ;
            }
         }
      }
      else if (32 == out_word_size)
      {
         int32_t *out_ptr_32 = (int32_t*) ptr_out_buf;

         shift = out_Q_fact - PCM_Q_FACTOR_27;

         uint32_t mask = 0xFFFFFFFF<<(out_word_size-out_bits_p_sample);

         for (ch = 0; ch < num_channels; ch++)
         {
            for (i = 0; i < num_sampl_p_ch; i++)
            {
               *(out_ptr_32 + i*num_channels + ch) = ((int32_t)((*(src_ptr_32 + ch*channel_spacing_samples + i)) << shift)) & mask;
            }
         }
      }
   }
   return AR_EOK;
}

/**
 * This function is for conversion of bytes per sample
 * for the data buf pointed to by ptr_src_buf while interleaving
 * the given samples.
 *
 * data in ptr_src_buf is 32 bits Q31 (PCM_32BIT_Q_FORMAT)
 * output in ptr_out_buf can be 16, 24 or 32 bit words.
 * when 32 Q formats are (Q23, Q27,) Q31. Q31 only supported
 * when 24 Q format is Q23
 * when 16 Q format is Q15
 *
 * input and output buffers cannot overlap.
 *
 * @param[in] ptr_src_buf, ptr to source buf to be converted,
 *       assumes channel interleaved data
 * @param[out] ptr_out_buf, pointer to output buffer, assumes
 *       interleaved data, should be sufficiently large to
 *       hold the output data.
 * @param[in] num_sampl_p_ch, number of samples per channel
 * @param[in] num_channels, number of channels in the input.
 * @param[in] out_word_size, word size in bits 32, 24, 16 of input samples
 * @param[in] out_Q_fact, Q factor of output Q15, Q23, Q27, Q31
 * @param[in] out_bits_p_sample, 16,24,32. When output word size is 32 &
 *            Q factor is Q31, it can be just 24 bits (last 8 bits zero).
 * @param[in] is_in_intlvd, whether the samples are interleaved. If not,
 * the channels are assumed to be separated by num_sampl_p_ch channels.
 * @param[in] channel_spacing_samples, channel spacing in samples
 *
 * @return error status
 *
 * input -> output
 * 32    ->    16
 * 32    ->    24
 * 32    ->    32
 *
 */

ar_result_t byte_convertor_with_intlv_32_in_Q31(int8_t * ptr_src_buf, int8_t * ptr_out_buf, uint32_t num_sampl_p_ch,
      uint32_t num_channels, uint16_t out_word_size, uint16_t out_Q_fact,
      uint16_t out_bits_p_sample, bool_t is_in_intlvd, uint32_t channel_spacing_samples)
{
   //error checks
   if ( ( (16 == out_word_size) && (15 != out_Q_fact) ) ||
         ( (24 == out_word_size) && (23 != out_Q_fact) ) ||
         ( (32 == out_word_size)  &&
               ( (23 != out_Q_fact) &&
                     (PCM_Q_FACTOR_27 != out_Q_fact) &&
                     (PCM_Q_FACTOR_31 != out_Q_fact) )
          )  )
   {
      return AR_EUNSUPPORTED;
   }

   // no in-place
   if (ptr_src_buf == ptr_out_buf)
   {
      return AR_EBADPARAM;
   }

   int32_t *src_ptr_32 = (int32_t*)ptr_src_buf;
   uint32_t shift;
   uint32_t i;
   uint32_t ch;
   int32_t temp32, samp;
   int32_t min = MIN_32;
   int32_t max = MAX_32;

   int64_t dw_max = ((int64_t)max << 32) | (int64_t)max;
   int64_t dw_min = ((int64_t)min << 32) | (int64_t)min;

   if (is_in_intlvd)
   {
      uint32_t total_samples = num_sampl_p_ch*num_channels;
      if (16 == out_word_size)
      {
         shift = PCM_Q_FACTOR_31 - out_Q_fact;

#ifdef USE_Q6_SPECIFIC_CODE
         int64_t *buf_ptr_64 = (int64_t *)(ptr_src_buf);      /* Input buffer  */
         int32_t *buf_ptr_32 = (int32_t *)(ptr_out_buf);     /* Output buffer */

         /* If input buf addr is 8 byte aligned and out buf addr is 4 byte aligned
            then only perform the vector operation
         */
         if( (0 == ((uint32_t)buf_ptr_64 & 0x7)) &&
             (0 == ((uint32_t)buf_ptr_32 & 0x3))
             )
         {
            for(samp = total_samples; samp >= 2 ; samp -= 2)
            {
               int64_t temp64;
               /* Convert from Q31 to Q15*/
               /*
               * Saturate to 31 bits*/
               temp64 = Q6_P_vminw_PP(*buf_ptr_64++, dw_max);
               temp64 = Q6_P_vmaxw_PP(temp64, dw_min);

               /* Right shift each of the 32 bit words into 16 bits and
                  store the two lower halfwords into destination
               */
               *buf_ptr_32++ = Q6_R_vasrw_PR(temp64, shift);
            }

            /* If the number of samples are odd, following loop will handle. */
            total_samples = samp;
         }
          /* if either of buf addr is not byte aligned as required for vectorization
           * or for the remaining samples in case it's odd*/
         {
            int32_t temp32;

            int16_t *buf_ptr_src_16 = (int16_t *)buf_ptr_32;
            int32_t *buf_ptr_src2_32 = (int32_t *)buf_ptr_64;

            for(samp = 0; samp < total_samples; samp++)
            {
               temp32 = *buf_ptr_src2_32++;
               /*saturate to 31 bits*/
               temp32 = Q6_R_max_RR(*buf_ptr_src2_32, (int32_t)min);
               temp32 = Q6_R_min_RR(temp32, (int32_t)max);

               /* Q31 to Q15 */
               /* First right shift followed by rounding operation */
               *buf_ptr_src_16++ = Q6_R_asr_RR(temp32, shift);
            }
         }

#else
         /*----------- Non Q6 Version ---------------*/
         int16_t *out_ptr_16 = (int16_t*) ptr_out_buf;
         for (i = 0; i < total_samples; i++)
         {
            temp32 = *src_ptr_32++;
            /*saturate to Q factor */
            temp32 = (temp32 < min)? min: temp32;
            temp32 = (temp32 > max)? max: temp32;

            *out_ptr_16++ = (int16_t) ((temp32) >> shift);
         }
#endif //USE_Q6_SPECIFIC_CODE
      }
      else if (24 == out_word_size)
      {
         shift = PCM_Q_FACTOR_31 - out_Q_fact;
         for (i = 0; i < total_samples; i++)
         {
            temp32 = *src_ptr_32++;
            /*saturate to Q factor */
            temp32 = (temp32 < min)? min: temp32;
            temp32 = (temp32 > max)? max: temp32;

            temp32 >>= shift;

            *ptr_out_buf++ = (uint8_t) ( (uint32_t)temp32 & 0x000000FF) ;
            *ptr_out_buf++ = (uint8_t) (( (uint32_t)temp32 & 0x0000FF00)>>8) ;
            *ptr_out_buf++ = (uint8_t) (( (uint32_t)temp32 & 0x00FF0000)>>16) ;
         }
      }
      else if (32 == out_word_size)
      {
         int32_t *out_ptr_32 = (int32_t*) ptr_out_buf;

         shift = out_Q_fact - PCM_Q_FACTOR_31;

         uint32_t mask = 0xFFFFFFFF<<(out_word_size-out_bits_p_sample);

         for (i = 0; i < total_samples; i++)
         {
            *out_ptr_32++ = ((*src_ptr_32++)<<shift) & mask;
         }
      }
   }
   else //input is deinterleaved
   {
      if (16 == out_word_size)
      {
         shift = PCM_Q_FACTOR_31 - out_Q_fact;
         int16_t *out_ptr_16 = (int16_t*) ptr_out_buf;
         for (ch = 0; ch < num_channels; ch++)
         {
            for (i = 0; i < num_sampl_p_ch; i++)
            {
               temp32 = (int32_t)(*(src_ptr_32 + ch*channel_spacing_samples + i));
               /*saturate to Q factor */
               temp32 = (temp32 < min)? min: temp32;
               temp32 = (temp32 > max)? max: temp32;

               *(out_ptr_16 + i*num_channels + ch) = (int16_t) ( (temp32) >> shift);
            }
         }
      }
      else if (24 == out_word_size)
      {
         shift = PCM_Q_FACTOR_31 - out_Q_fact;
         uint8_t *ptr;
         for (ch = 0; ch < num_channels; ch++)
         {
            for (i = 0; i < num_sampl_p_ch; i++)
            {
               temp32 = *(src_ptr_32 + ch*channel_spacing_samples + i);
               /*saturate to Q factor */
               temp32 = (temp32 < min)? min: temp32;
               temp32 = (temp32 > max)? max: temp32;

               temp32 >>= shift;

               ptr = (uint8_t *)(ptr_out_buf + i*num_channels*out_word_size/8 + ch*out_word_size/8);

               *ptr++ = (uint8_t) ( (uint32_t)temp32 & 0x000000FF) ;
               *ptr++ = (uint8_t) ( ((uint32_t)temp32 & 0x0000FF00)>>8) ;
               *ptr++ = (uint8_t) ( ((uint32_t)temp32 & 0x00FF0000)>>16) ;
            }
         }
      }
      else if (32 == out_word_size)
      {
         int32_t *out_ptr_32 = (int32_t*) ptr_out_buf;

         shift = out_Q_fact - PCM_Q_FACTOR_31;

         uint32_t mask = 0xFFFFFFFF<<(out_word_size-out_bits_p_sample);

         for (ch = 0; ch < num_channels; ch++)
         {
            for (i = 0; i < num_sampl_p_ch; i++)
            {
               *(out_ptr_32 + i*num_channels + ch) = ((int32_t)((*(src_ptr_32 + ch*channel_spacing_samples + i)) << shift)) & mask;
            }
         }
      }
   }
   return AR_EOK;
}


/**
 * This function is for conversion of the byte order in which the input
 * samples are stored (big -> little and vice-versa)
 *
 * @param[in, out] ptr_src_buf, ptr to source buf to be converted,
 *       assumes channel interleaved data
 * @param[in] total_num_samples, number of samples to be converted
 * @param[in] in_word_size, size in bits in which samples are stored (16, 24, 32)
 *
 * @return error status
 *
 */

ar_result_t endianness_convertor_with_intlv_in(int8_t *ptr_src_buf, uint32_t total_num_samples, uint16_t in_word_size)
{
    if((in_word_size != 16) && (in_word_size != 24) && (in_word_size != 32))
    {
        return AR_EUNSUPPORTED;
    }

    uint32_t i;

    if(in_word_size == 16)
    {
        int16_t* ptr_16 = (int16_t *)ptr_src_buf;
        uint16_t val;
        for(i=0; i < total_num_samples; i++)
        {
            val = *ptr_16;
            *ptr_16++ = (int16_t) (((val & 0xFF00) >> 8) | ( (val & 0x00FF) << 8));
        }
    }
    else if(in_word_size == 24)
    {
        int8_t* dest_ptr = ptr_src_buf;
        uint8_t val1, val2, val3;

        for(i=0; i< total_num_samples; i++)
        {
            val1 = *ptr_src_buf++;
            val2 = *ptr_src_buf++;
            val3 = *ptr_src_buf++;
            *dest_ptr++ = val3;
            *dest_ptr++ = val2;
            *dest_ptr++ = val1;
        }
    }
    else if (in_word_size == 32)
    {
        int32_t *dest_ptr = (int32_t *)ptr_src_buf;
        uint32_t val;
        for(i = 0;i< total_num_samples;i++)
        {
            val = *dest_ptr;

#if ((defined __hexagon__) || (defined __qdsp6__))
            *dest_ptr++ = Q6_R_swiz_R(val);
#else
            *dest_ptr++ =  (int32_t)(( val & 0x000000FF)<<24  | (val & 0x0000FF00) << 8 | (val & 0x00FF0000) >> 8 | (val & 0xFF000000) >> 24);
#endif
        }
    }

    return AR_EOK;
}



