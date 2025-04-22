#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

#Include directories
#Add the source files
include_directories(
                    ../cmn/inc
                    ../core/src
                    ../core/src/generic
                    ../ext/bt_codec_fwk_ext/inc
                    ../ext/err_check/inc
                    ../ext/wr_sh_mem_ep/inc
                    ../ext/sync_fwk_ext/inc
                    ../ext/rd_sh_mem_ep/inc
                    ../ext/pure_st/inc
                    ../ext/placeholder/inc
                    ../ext/pass_thru_cntr/inc
                    ../ext/peer_cntr/inc
                    ../ext/offload/inc
                    ../ext/metadata/inc
                   )

#Add the sub directories
add_subdirectory(../core/build core)
add_subdirectory(../ext/build ext)
