#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

include_directories (
		../inc
		../../../../../amdb/ext/src #for amdb queue (AMDB queue has dependency on condvar, nmutex - which are not implemented on linux)
	)
#Add the sub directories
file(GLOB capi_library_thread_pool_src 
				"../src/*.cpp" 
				"../src/*.c"
				)

#Add the source files
set (lib_srcs_list
	${capi_library_thread_pool_src}
    )
	
#Call spf_build_static_library to generate the static library
spf_build_static_library(capi_library_thread_pool
                         "${lib_incs_list}"
                         "${lib_srcs_list}"
                         "${lib_defs_list}"
                         "${lib_flgs_list}"
                         "${lib_link_libs_list}"
                        )

