/**
 * \file posal_thread_profiling.c
 * \brief
 *     This file contains definitions for thread profiling utilities.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */

#include "posal.h"
#include "posal_thread_profiling.h"
#include "posal_thread_profiling_i.h"

/*--------------------------------------------------------------*/
/* Macro definitions                                            */
/* -------------------------------------------------------------*/

#ifdef POSAL_ENABLE_THREAD_PROFILING
/** Maximum length of the thread name, including the NULL terminator.
*/
#define   POSAL_THREAD_NAME_MAX_LENGTH (16)

/** Maximum number of hardware threads available on the platform. */
#define   POSAL_MAX_HW_THREADS           4

/** Maximum number of software threads available on the platform. */
#ifdef MAX_THREADS
#define   POSAL_MAX_SW_THREADS          MAX_THREADS
#else /* MAX_THREADS */
#define   POSAL_MAX_SW_THREADS          (128)
#endif /* MAX_THREADS */

#if defined(SIM)
#define POSAL_ENABLE_STACK_FILLING
#endif

#ifndef POSAL_THREAD_STACK_FILL_WORD
#define POSAL_THREAD_STACK_FILL_WORD             0xF8F8F8F8L
#endif

/** Thread stack fill spacing. */
#ifndef POSAL_THREAD_STACK_FILL_SPACING
#define POSAL_THREAD_STACK_FILL_SPACING          128
#endif

/** Node of a thread in the array of threads in the global state structure. */
typedef struct posal_thread_profile_list_t
{
   uint32_t  	  	  tid;      /**< Thread ID. */
   int                exit_status;   /**< Thread exit status. */
   char*              stack_ptr;        /**< Thread stack buffer. */
   int                stack_size;    /**< Thread stack size. */
   char               name[POSAL_THREAD_NAME_MAX_LENGTH];
                                     /**< Thread name. */
} posal_thread_profile_list_t;

/*Global context buffer */
posal_thread_profile_list_t      *global_thread_context_buffer[POSAL_MAX_SW_THREADS];

/*--------------------------------------------------------------*/
/* Function definitions                                            */
/* -------------------------------------------------------------*/

static uint32_t posal_thread_stack_use_estimate(void* sp, uint32_t size)
{
   uint32_t *pStackChecker =
         (uint32_t*) (sp);
   int nFreeStack = 0;
   for (nFreeStack = 0;
         nFreeStack < size;
         nFreeStack += POSAL_THREAD_STACK_FILL_SPACING * 4, pStackChecker +=
               POSAL_THREAD_STACK_FILL_SPACING)
   {
      if (POSAL_THREAD_STACK_FILL_WORD != *pStackChecker)
         break;
   }
   return nFreeStack;

}

static void posal_thread_stack_prof_init(void* sp, uint32_t size)
{
   uint32_t *stack_filler =
         (uint32_t*) (sp);
   for ( ; stack_filler < (uint32_t*) (((uint8_t*)sp) + size); stack_filler += POSAL_THREAD_STACK_FILL_SPACING)
   {
      *stack_filler = POSAL_THREAD_STACK_FILL_WORD;
   }

   uint32_t num_bytes = (uint32_t)((uintptr_t)sp) + size - ((uintptr_t)stack_filler);
   AR_MSG(DBG_LOW_PRIO, "Posal thread profiling: Stack profiler: filled %d of %d bytes", num_bytes, size);
}



ar_result_t posal_thread_profile_at_launch(   void **thread_list_ptr_ptr,
										char* stack_ptr,
										uint32_t stack_size,
										char *threadname,
										POSAL_HEAP_ID heap_id)
{
	int count;

	posal_thread_profile_list_t *pThreadNode =
			(posal_thread_profile_list_t *)posal_memory_malloc(sizeof(posal_thread_profile_list_t),  heap_id);
	if(NULL == pThreadNode)
	{
		return AR_ENOMEMORY;
	}

	// scan all the structures to find which one is free
	posal_mutex_lock(posal_globalstate.mutex);
	for (count=0;count<POSAL_MAX_SW_THREADS;count++)
	{
		if(NULL == global_thread_context_buffer[count])
		{
			/*Just load some address to mark being used*/
			global_thread_context_buffer[count] = (posal_thread_profile_list_t *)pThreadNode;
			break;
		}
	}
	posal_mutex_unlock(posal_globalstate.mutex);

	/*Already max sw threads are running cannot initialize threads further*/
	if(POSAL_MAX_SW_THREADS == count)
	{
		AR_MSG(DBG_ERROR_PRIO, "Allocating more than supported threads !");
		posal_memory_free(pThreadNode);
		return AR_EFAILED;
	}

	/*Return thread object pointer */
	*thread_list_ptr_ptr = (void *)pThreadNode;

	/* Update the context buffer with the thread context object */
	posal_mutex_lock(posal_globalstate.mutex);
	global_thread_context_buffer[count] = pThreadNode;
	posal_mutex_unlock(posal_globalstate.mutex);

	pThreadNode->stack_ptr = stack_ptr;
	pThreadNode->stack_size = stack_size;
	pThreadNode->exit_status = 0xdeadbabe;

	posal_strlcpy(&(pThreadNode->name[0]), threadname, POSAL_THREAD_NAME_MAX_LENGTH);
	posal_thread_stack_prof_init(stack_ptr, stack_size);



	return AR_EOK;
}

ar_result_t posal_thread_profile_at_exit(void *thread_profile_obj_ptr, uint32_t tid, int32_t exit_status )
{
	if(NULL == thread_profile_obj_ptr)
	{
		return AR_EBADPARAM;
	}
	posal_thread_profile_list_t *pThreadNode = (posal_thread_profile_list_t*)thread_profile_obj_ptr;

	pThreadNode->tid = tid;
	pThreadNode->exit_status = exit_status;

	return AR_EOK;
}

void posal_thread_profile_set_tid(   void **thread_list_ptr_ptr,
										uint32_t tid)
{
	posal_thread_profile_list_t *pThreadNode = (posal_thread_profile_list_t *)	*thread_list_ptr_ptr;
	pThreadNode->tid = tid;
}

ar_result_t  posal_thread_profile_at_join(void *thread_profile_obj_ptr, int *pStatus)
{
	if(NULL == thread_profile_obj_ptr)
	{
		return AR_EBADPARAM;
	}
	posal_thread_profile_list_t *pThreadNode = (posal_thread_profile_list_t*)thread_profile_obj_ptr;

	/* Find exit status, and remove node from global state structure */
	int count =0;
	for(count=0;count<POSAL_MAX_SW_THREADS;count++)
	{
		if (global_thread_context_buffer[count] == pThreadNode)
		{
			*pStatus = global_thread_context_buffer[count]->exit_status;
			posal_thread_stack_use_estimate(global_thread_context_buffer[count]->stack_ptr, global_thread_context_buffer[count]->stack_size);

			/* free global structure memory */
			posal_memory_free(global_thread_context_buffer[count]);
			global_thread_context_buffer[count] = NULL;

			return AR_EOK;
		}
	}

	AR_MSG( DBG_ERROR_PRIO,
			"THRD JOIN: Thread %d is not found in global state structure", pThreadNode->tid);

	return AR_EOK;
}

// This is needed for IRM even on target so it's enabled regardless of POSAL_ENABLE_THREAD_PROFILING
ar_result_t posal_thread_profiling_get_stack_info(uint32_t tid, uint32_t* current_stack_usage_ptr, uint32_t* stack_size_ptr)
{
	int count =0;
	uint32_t free_stack;
	for(count=0;count<POSAL_MAX_SW_THREADS;count++)
	{
		if (NULL != global_thread_context_buffer[count] && (uint32_t)global_thread_context_buffer[count]->tid == tid)
		{
			free_stack = posal_thread_stack_use_estimate(global_thread_context_buffer[count]->stack_ptr, global_thread_context_buffer[count]->stack_size);
			*stack_size_ptr = global_thread_context_buffer[count]->stack_size;
			*current_stack_usage_ptr = (*stack_size_ptr) - free_stack;
			return AR_EOK;
		}
	}

	AR_MSG( DBG_ERROR_PRIO, "THRD STACK PROFILE: Thread 0x%x is not found in global state structure", tid);

	return AR_EFAILED;

}

#endif /* POSAL_ENABLE_THREAD_PROFILING */
