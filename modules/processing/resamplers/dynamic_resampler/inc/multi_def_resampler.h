/*=========================================================================
Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
SPDX-License-Identifier: BSD-3-Clause-Clear
========================================================================= */

#ifndef MULTI_DEF_RESAMPLER_H
#define MULTI_DEF_RESAMPLER_H

#define resamp_output_latency resamp_output_latency_dynamic_resampler
#define resamp_input_latency resamp_input_latency_dynamic_resampler
#define resamp_size resamp_size_dynamic_resampler
#define select_filter select_filter_dynamic_resampler
#define init_upsamp init_upsamp_dynamic_resampler
#define straight_copy straight_copy_dynamic_resampler
#define gen_rs_filt_coefs_lin32 gen_rs_filt_coefs_lin32_dynamic_resampler
#define rsphase_update_frac rsphase_update_frac_dynamic_resampler
#define conv_32 conv_32_dynamic_resampler
#define init_dnsamp init_dnsamp_dynamic_resampler
#define conv_16X32 conv_16X32_dynamic_resampler
#define int_div_16 int_div_16_dynamic_resampler
#define int_div_32 int_div_32_dynamic_resampler
#define int_div_64 int_div_64_dynamic_resampler
#define frac_div_32 frac_div_32_dynamic_resampler
#define halfQ15 halfQ15_dynamic_resampler
#define oneQ15 oneQ15_dynamic_resampler
#define resampFiltQFac resampFiltQFac_dynamic_resampler
#define resampTotalPhGrp resampTotalPhGrp_dynamic_resampler
#define resampFiltLenGrp resampFiltLenGrp_dynamic_resampler


#endif /* MULTI_DEF_RESAMPLER_H */
