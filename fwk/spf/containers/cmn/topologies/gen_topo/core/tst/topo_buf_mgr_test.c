/**
 * \file topo_buf_mgr_test.c
 *  
 * \brief
 *  
 *     Topology buffer manager test file
 *  
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "ar_defs.h"
#include "posal.h"
#include "spf_utils.h"
#include "ar_msg.h"
#include "ar_ids.h"
#include "topo_buf_mgr.h"

#ifdef ENABLE_BUF_MANAGER_TEST

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

static ar_result_t test_1()
{
   ar_result_t result = AR_EOK;
   void *      buf_mgr_ptr;
   int8_t *    buf1_ptr, *buf2_ptr;
   uint32_t    buf_size = 0;

   result = topo_buf_manager_create(&buf_mgr_ptr, POSAL_HEAP_DEFAULT, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 1: topo_buf_manager_create buf_mgr_ptr: 0x%lx, result: %u", buf_mgr_ptr, result);

   buf_size = 20;
   result   = topo_buf_manager_get_buf(buf_mgr_ptr, &buf1_ptr, buf_size, 0);

   AR_MSG(DBG_HIGH_PRIO,
          "buf_mgr_test 1: topo_buf_manager_get_buf buf_size: %u buf_ptr: 0x%lx, result: %u",
          buf_size,
          buf1_ptr,
          result);

   buf_size = 30;
   result   = topo_buf_manager_get_buf(buf_mgr_ptr, &buf2_ptr, buf_size, 0);

   AR_MSG(DBG_HIGH_PRIO,
          "buf_mgr_test 1: topo_buf_manager_get_buf buf_size: %u buf_ptr: 0x%lx, result: %u",
          buf_size,
          buf2_ptr,
          result);

   topo_buf_manager_return_buf(buf_mgr_ptr, buf1_ptr, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 1: topo_buf_manager_return_buf returned buf1_ptr: 0x%lx", buf1_ptr);

   topo_buf_manager_return_buf(buf_mgr_ptr, buf2_ptr, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 1: topo_buf_manager_return_buf returned buf2_ptr: 0x%lx", buf2_ptr);

   topo_buf_manager_destroy(buf_mgr_ptr, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 1: topo_buf_manager_destroy destroyed buf_mgr_ptr: 0x%lx", buf_mgr_ptr);

   return result;
}

static ar_result_t test_2()
{
   ar_result_t result = AR_EOK;
   void *      buf_mgr_ptr;
   int8_t *    buf1_ptr, *buf2_ptr;
   uint32_t    buf_size = 0;

   result = topo_buf_manager_create(&buf_mgr_ptr, POSAL_HEAP_DEFAULT, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 1: topo_buf_manager_create buf_mgr_ptr: 0x%lx, result: %u", buf_mgr_ptr, result);

   buf_size = 20;
   result   = topo_buf_manager_get_buf(buf_mgr_ptr, &buf1_ptr, buf_size, 0);

   AR_MSG(DBG_HIGH_PRIO,
          "buf_mgr_test 2: topo_buf_manager_get_buf buf_size: %u buf_ptr: 0x%lx, result: %u",
          buf_size,
          buf1_ptr,
          result);

   buf_size = 30;
   result   = topo_buf_manager_get_buf(buf_mgr_ptr, &buf2_ptr, buf_size, 0);

   AR_MSG(DBG_HIGH_PRIO,
          "buf_mgr_test 2: topo_buf_manager_get_buf buf_size: %u buf_ptr: 0x%lx, result: %u",
          buf_size,
          buf2_ptr,
          result);

   topo_buf_manager_return_buf(buf_mgr_ptr, buf1_ptr, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 2: topo_buf_manager_return_buf returned buf_ptr: 0x%lx", buf1_ptr);

   topo_buf_manager_return_buf(buf_mgr_ptr, buf2_ptr, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 2: topo_buf_manager_return_buf  returned buf_ptr: 0x%lx", buf2_ptr);

   buf_size = 40;
   result   = topo_buf_manager_get_buf(buf_mgr_ptr, &buf1_ptr, buf_size, 0);

   AR_MSG(DBG_HIGH_PRIO,
          "buf_mgr_test 2: topo_buf_manager_get_buf buf_size: %u buf_ptr: 0x%lx, result: %u",
          buf_size,
          buf1_ptr,
          result);

   buf_size = 30;
   result   = topo_buf_manager_get_buf(buf_mgr_ptr, &buf2_ptr, buf_size, 0);

   AR_MSG(DBG_HIGH_PRIO,
          "buf_mgr_test 2: topo_buf_manager_get_buf buf_size: %u buf_ptr: 0x%lx, result: %u",
          buf_size,
          buf2_ptr,
          result);

   topo_buf_manager_return_buf(buf_mgr_ptr, buf1_ptr, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 2: topo_buf_manager_return_buf returned buf_ptr: 0x%lx", buf1_ptr);

   topo_buf_manager_return_buf(buf_mgr_ptr, buf2_ptr, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 2: topo_buf_manager_return_buf  returned buf_ptr: 0x%lx", buf2_ptr);

   topo_buf_manager_destroy(buf_mgr_ptr, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 2: topo_buf_manager_destroy destroyed buf_mgr_ptr: 0x%lx", buf_mgr_ptr);

   return result;
}

static ar_result_t test_3()
{
   ar_result_t result = AR_EOK;
   void *      buf_mgr_ptr;
   int8_t *    buf1_ptr, *buf2_ptr;
   uint32_t    buf_size = 0;

   result = topo_buf_manager_create(&buf_mgr_ptr, POSAL_HEAP_DEFAULT, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 3: topo_buf_manager_create buf_mgr_ptr: 0x%lx, result: %u", buf_mgr_ptr, result);

   buf_size = 20;
   result   = topo_buf_manager_get_buf(buf_mgr_ptr, &buf1_ptr, buf_size, 0);

   AR_MSG(DBG_HIGH_PRIO,
          "buf_mgr_test 3: topo_buf_manager_get_buf buf_size: %u buf_ptr: 0x%lx, result: %u",
          buf_size,
          buf1_ptr,
          result);

   topo_buf_manager_return_buf(buf_mgr_ptr, buf1_ptr, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 3: topo_buf_manager_return_buf returned buf1_ptr: 0x%lx", buf1_ptr);

   for (uint32_t i = 0; i < 25; i++)
   {
      buf_size = 30;
      result   = topo_buf_manager_get_buf(buf_mgr_ptr, &buf2_ptr, buf_size, 0);

      AR_MSG(DBG_HIGH_PRIO,
             "buf_mgr_test 3: topo_buf_manager_get_buf buf_size: %u buf_ptr: 0x%lx, result: %u",
             buf_size,
             buf2_ptr,
             result);

      topo_buf_manager_return_buf(buf_mgr_ptr, buf2_ptr, 0);

      AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 3: topo_buf_manager_return_buf returned buf2_ptr: 0x%lx", buf2_ptr);
   }

   topo_buf_manager_destroy(buf_mgr_ptr, 0);

   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test 3: topo_buf_manager_destroy destroyed buf_mgr_ptr: 0x%lx", buf_mgr_ptr);

   return result;
}

ar_result_t buf_mgr_test()
{
   ar_result_t result = AR_EOK, local_result = AR_EOK;

   local_result = test_1();
   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test: test 1 result: %d", local_result);
   result |= local_result;

   local_result = test_2();
   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test: test 2 result: %d", local_result);
   result |= local_result;

   local_result = test_3();
   AR_MSG(DBG_HIGH_PRIO, "buf_mgr_test: test 3 result: %d", local_result);
   result |= local_result;

   return result;
}

#ifdef __cplusplus
}
#endif //__cplusplus
#endif // ENABLE_BUF_MANAGER_TEST
