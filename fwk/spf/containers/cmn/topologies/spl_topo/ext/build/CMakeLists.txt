#[[
   @file CMakeLists.txt

   @brief

   @copyright
   Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
   SPDX-License-Identifier: BSD-3-Clause-Clear

]]
cmake_minimum_required(VERSION 3.10)

#Add the sub directories
add_subdirectory(../dm_fwk_ext/build dm_fwk_ext)
add_subdirectory(../mimo_proc_state_intf_ext/build mimo_proc_state_intf_ext)
add_subdirectory(../sync_fwk_ext/build sync_fwk_ext)
add_subdirectory(../trigger_policy_fwk_ext/build trigger_policy_fwk_ext)
