/*==============================================================================
  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
  SPDX-License-Identifier: BSD-3-Clause-Clear
  ==============================================================================*/

/*============================================================================
  FILE:          main.c

  OVERVIEW:      Entry point for PCM_converter unit test

  DEPENDENCIES:  None

  ============================================================================*/
#include <iostream.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <hexagon_sim_timer.h>

#include "wavefile.h"
#include "pcm_converter.h"
using namespace std;
uint8_t posal_debugmsg_lowest_prio = 4;

/* -----------------------------------------------------------------------
** Global Data
** ----------------------------------------------------------------------- */
extern WavHeader wh;
extern WavHeader wh_write;
void get_interleaved_info(pc_main_mf_t *media_fmt_ptr, int in_value)
{
   switch (in_value)
   {
   case 0:
      media_fmt_ptr->interleaving = CAPI_DEINTERLEAVED_PACKED;
      break;
   case 1:
      media_fmt_ptr->interleaving = CAPI_INTERLEAVED;
      break;
   default:
      media_fmt_ptr->interleaving = CAPI_INVALID_INTERLEAVING;
      break;
   }
}

void get_endian_info(pc_main_mf_t *media_fmt_ptr, int in_value)
{
   switch (in_value)
   {
   case 0:
      media_fmt_ptr->endianness = PCM_CNV_LITTLE_ENDIAN;
      break;
   case 1:
      media_fmt_ptr->endianness = PCM_CNV_BIG_ENDIAN;
      break;
   default:
      media_fmt_ptr->endianness = PCM_CNV_UNKNOWN_ENDIAN;
      break;
   }
}

void print_media_format(pc_main_mf_t *media_fmt_ptr)
{
   cout << endl;
   cout << "Media format is" << endl;
   cout << "Bit width :" << media_fmt_ptr->bit_width << endl;
   cout << "Word size :" << media_fmt_ptr->word_size << endl;
   cout << "Q format :" << media_fmt_ptr->q_factor << endl;
   cout << "Interleaving :" << media_fmt_ptr->interleaving << endl;
   cout << "Endianness :" << media_fmt_ptr->endianness << endl << endl;
}

int main(int argc, char *argv[])
{
   ar_result_t         result               = AR_EOK;
   unsigned long long  cycles               = 0;
   int                 sample_count         = 0;
   int                 bytes_to_read        = 0;
   int                 bytes_to_write       = 0;
   int                 bytes_read           = 0;
   int                 bytes_written        = 0;
   int                 frame_size_bytes     = 0;
   int                 frame_size_bytes_out = 0;
   int                 frame_size_samples   = 0;
   pc_main_mf_t inp_media_fmt        = { 0 };
   pc_main_mf_t out_media_fmt        = { 0 };
   int8_t *            input_buf_ptr        = NULL;
   int8_t *            output_buf_ptr       = NULL;
   int8_t *            scratch_buf_ptr      = NULL;
   pc_lib_t       me;
   pc_lib_t *     me_ptr = &me;

   // Check the number of arguments
   if (argc < 11)
   {
      fprintf(stderr, "Qualcomm's new PCM converter:\n");
      fprintf(stderr,
              "Usage: tst_pcm_cnv <input file>  <output file>  <input qFormat> <input interleaved 1/0> <input "
              "endieness 1/0> <output bitWidth> <output wordSize> <output qFormat> <output interleaved 1/0> "
              "<output endieness 1/0> \n\n");
      fprintf(stderr, "Input Multi-channel Audio wave file. \n");
      fprintf(stderr, "Output Multi-channel Audio output from PCM converter. \n");
      fprintf(stderr, "Q_format of input file - 15, 23, 27 or 31 \n");
      fprintf(stderr, "Whether input file is to be interleaved - set 1 to interleave\n");
      fprintf(stderr, "Endianness of input file - 0 or 1, 0 for little\n");
      fprintf(stderr, "Bitwidth of output file - 16, 24 or 32 \n");
      fprintf(stderr, "Word size of output file - 16, 24 or 32 \n");
      fprintf(stderr, "Q_format of output file - 15, 23, 27 or 31 \n");
      fprintf(stderr, "Whether output file is to be interleaved - set 1 to interleave\n");
      fprintf(stderr, "Endianness of output file - 0 or 1, 0 for little\n");
      fprintf(stderr, "The input multi-channel audio file can be in interleaved/deinterleaved format. \n");
      return -1;
   }

   if (0 != WaveFileReadInit(argv[1], 1))
      return -1;

   inp_media_fmt.bit_width = wh.bitsPerSample;
   inp_media_fmt.word_size = wh.bytesPerSample * 8 / wh.numChannels;
   inp_media_fmt.q_factor  = atoi(argv[3]);
   get_interleaved_info(&inp_media_fmt, atoi(argv[4]));
   get_endian_info(&inp_media_fmt, atoi(argv[5]));

   out_media_fmt.bit_width = atoi(argv[6]);
   out_media_fmt.word_size = atoi(argv[7]);
   out_media_fmt.q_factor  = atoi(argv[8]);
   get_interleaved_info(&out_media_fmt, atoi(argv[9]));
   get_endian_info(&out_media_fmt, atoi(argv[10]));

   memcpy(&wh_write, &wh, sizeof(WavHeader));
   wh_write.bitsPerSample  = out_media_fmt.bit_width;
   wh_write.bytesPerSample = out_media_fmt.word_size * wh_write.numChannels / 8;
   wh_write.bytesPerSecond = wh_write.bytesPerSample * wh_write.sampleRate;
   wh_write.dataLength     = wh.dataLength * wh_write.bytesPerSample / wh.bytesPerSample;
   print_media_format(&inp_media_fmt);
   print_media_format(&out_media_fmt);

   if (0 != WaveFileWriteInit(argv[2]))
   {
      WaveFileIOEnd();
      return -1;
   }

   frame_size_samples   = (wh.sampleRate / wh.sampleRate * 2) * wh.numChannels; // 2 samples per channel
   frame_size_bytes     = frame_size_samples * inp_media_fmt.word_size >> 3;
   frame_size_bytes_out = frame_size_samples * out_media_fmt.word_size >> 3;

   
   input_buf_ptr   = (int8_t *)calloc(frame_size_bytes, sizeof(int8_t));
   output_buf_ptr  = (int8_t *)calloc(frame_size_bytes_out, sizeof(int8_t));
   scratch_buf_ptr = (int8_t *)calloc(frame_size_bytes_out, sizeof(int8_t));

   sample_count = wh.dataLength / wh.bytesPerSample;

   me.input_media_fmt = inp_media_fmt;
   me.output_media_fmt = out_media_fmt;
   me.inp_media_fmt_combo = pc_classify_mf(&inp_media_fmt);
   me.out_media_fmt_combo = pc_classify_mf(&out_media_fmt);

   cout << "frame_size_samples :" << frame_size_samples << endl;
   cout << "frame_size_bytes :" << frame_size_bytes << endl;
   cout << "frame_size_bytes_out :" << frame_size_bytes_out << endl;
   do
   {
      if (frame_size_bytes > wh.dataLength)
      {
         bytes_to_read  = wh.dataLength;
         bytes_to_write = wh_write.dataLength;
      }
      else
      {
         bytes_to_read  = frame_size_bytes;
         bytes_to_write = frame_size_bytes_out;
      }

      // Read data from the input wave file
      if (WaveFileRead((char *)input_buf_ptr, bytes_to_read) < 0)
      {
         break;
      }
      bytes_read += bytes_to_read;
      sample_count -= bytes_to_read / wh.bytesPerSample;

      capi_buf_t inp_buffer       = { 0 };
      capi_buf_t out_buffer       = { 0 };
      capi_buf_t scratch_buffer   = { 0 };
      inp_buffer.data_ptr            = (int8_t *)input_buf_ptr;
      inp_buffer.actual_data_len     = bytes_to_read;
      inp_buffer.max_data_len        = frame_size_bytes * sizeof(int8_t);
      out_buffer.data_ptr            = (int8_t *)output_buf_ptr;
      out_buffer.actual_data_len     = 0;
      out_buffer.max_data_len        = frame_size_bytes_out * sizeof(int8_t);
      scratch_buffer.data_ptr        = (int8_t *)scratch_buf_ptr;
      scratch_buffer.actual_data_len = 0;
      scratch_buffer.max_data_len    = frame_size_bytes_out * sizeof(int8_t);
      pc_ch_info_t temp         = { 0 };
      temp.num_samp_per_ch           = bytes_to_read / ((inp_media_fmt.word_size >> 3) * wh.numChannels);
      temp.num_channels              = wh.numChannels;
      temp.chan_spacing_in           = bytes_to_read / wh.numChannels;
      temp.chan_spacing_out          = bytes_to_write / wh_write.numChannels;

      static int flag = 0;
      if (0 == flag)
      {
         cout << "num_samp_per_ch :" << temp.num_samp_per_ch << endl;
         cout << "num_channels :" << temp.num_channels << endl;
         cout << "chan_spacing_in :" << temp.chan_spacing_in << endl;
         cout << "chan_spacing_out :" << temp.chan_spacing_out << endl;
         flag = 1;
      }
      result = pc_process(me_ptr,
                                     &inp_buffer,
                                     &out_buffer,
                                     &scratch_buffer);
      if (AR_EOK != result)
      {
         AR_MSG(DBG_ERROR_PRIO, "Failed to PCM convert, lol");
         break;
      }
      //printf("%x %x %x\n", output_buf_ptr, input_buf_ptr, scratch_buf_ptr);
      WaveFileWrite((char *)output_buf_ptr, bytes_to_write);

      bytes_written += bytes_to_write;

   } while (sample_count > 0); // End main processing loop

   printf("cycles %llu\t", cycles);
   /* printf("MIPS %f\n", */
   /*        (((float)cycles * fparams[SAMPLE_RATE] * (float)numChannels)) / ((float)samplesWritten * 1000000)); */

   free(input_buf_ptr);
   free(output_buf_ptr);
   free(scratch_buf_ptr);
   printf("Bytes read from input file: %d\n", bytes_read);
   printf("Bytes written to output file: %d\n", bytes_written);

   return 0;
}
