/**
 * \file posal_timer.h
 * \brief
 *  	 This file contains utilities for timers. One-shot, period and sleep timers are provided.
 *
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_TIMER_H
#define POSAL_TIMER_H

/* =======================================================================
INCLUDE FILES FOR MODULE
========================================================================== */
#include "ar_error_codes.h"
#include "posal_channel.h"
#include "posal_signal.h"

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/** @addtogroup posal_timer
@{ */

/****************************************************************************
 ** Timers
 *****************************************************************************/
/** Valid timer types. */
typedef enum {
	POSAL_TIMER_ONESHOT_DURATION=0, /**< Duration-based one-shot timer. */
	POSAL_TIMER_ONESHOT_ABSOLUTE,         /**< Non-Defferable Absolute one-shot timer. */
	POSAL_TIMER_PERIODIC,            /**< Periodic timer. */
	POSAL_TIMER_ONESHOT_ABSOLUTE_DEFERRABLE  /**< Defferable Absolute one-shot timer. */
} posal_timer_duration_t;

/** Valid timer clock sources. */
typedef enum {
	POSAL_TIMER_USER      /**< General purpose timer for users/clients. */
} posal_timer_src_t;

/** Valid timer client notification types */
typedef enum {
	POSAL_TIMER_NOTIFY_OBJ_TYPE_SIGNAL=0,        /**< Notify timer client through signal posal_signal_t */
	POSAL_TIMER_NOTIFY_OBJ_TYPE_CB_FUNC,         /**< Notify timer client through callback function posal_timer_callback_info_t */
  MAX_SUPPORTED_POSAL_TIMER_NOTIFY_OBJ_TYPES   /**< Max supported timer notification object types>*/
} posal_timer_client_notification_type_t;

typedef void *posal_timer_t;

/* Callback function signature */
typedef void (*posal_timer_callback_func_t)(void *callback_context_ptr);

typedef struct
{
    posal_timer_callback_func_t  cb_func_ptr;
    void*                        cb_context_ptr;
}posal_timer_callback_info_t;

/** posal timer structure. */
typedef struct {
  timer_t*         timer_obj;
  /**< Pointer to timer object. */

  uint64_t          timer_start_time;

  uint32_t          uTimerType;
  /**< Timer type; see #posal_timer_duration_t. */

  uint32_t            timer_sigmask;
  /**< Signal mask of the timer signal to be set when the timer expires. */

  posal_channel_t*      pChannel;
  /**< Pointer to the channel that waits for the timer signal. */

  bool_t            istimerCreated;
  /**< Specifies whether the timer is created.
         @values
         - TRUE -- Created
         - FALSE -- Not created @tablebulletend */

  uint64_t        duration;
  /**< Duration (in microseconds) of the timer.

          - For periodic timers, the duration value is the period of the
            timer.
          - For absolute timers, the duration value is the value of the
            absolute expiry time minus the time at start of the timer.
            @tablebulletend*/

  bool_t          is_deferrable_group;
  /**< Specifies whether the timer is deferrable.
         @values
         - TRUE -- Deferrable
         - FALSE -- Non-deferrable @tablebulletend */
  POSAL_HEAP_ID        heap_id;
  /**< heap id type whether low power or default. */

  void *uTimerInfo;
   /**< Specifies if this timer has any utimer attributes */
}posal_timer_info_t;

typedef struct {
  posal_signal_t*  pSignal;
     /**< Pointer to the signal to be set */

 }posal_utimer_info_t;
/* -----------------------------------------------------------------------
 **  Function Definitions.
 ** ----------------------------------------------------------------------- */
/**
  Creates a timer in the default non-deferrable timer group.

  @datatypes
  posal_timer_t \n
  #posal_timer_duration_t \n
  #posal_timer_src_t \n
  posal_signal_t

  @param[in] pTimer      Pointer to the POSAL timer object.
  @param[in] timerType   One of the following:
                          - #POSAL_TIMER_ONESHOT_DURATION
                          - #POSAL_TIMER_PERIODIC
                          - #POSAL_TIMER_ONESHOT_ABSOLUTE
                            @tablebulletend
  @param[in] clockSource Clock source is #POSAL_TIMER_USER.
  @param[in] pSignal     Pointer to the signal to be generated when the timer
                         expires.
  @param[in] heap_id     heap id needed for malloc.
  @detdesc
  The caller must allocate the memory for the timer structure and pass the
  pointer to this function.
  @par
  After calling this function, call the appropriate start functions based on
  the type of timer.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  This function must be called before arming the timer. @newpage
 */
int32_t posal_timer_create(posal_timer_t          *pp_timer,
                           posal_timer_duration_t timerType,
                           posal_timer_src_t      clockSource,
                           posal_signal_t         p_signal,
                           POSAL_HEAP_ID          heap_id);

/**
  Creates a timer in the default non-deferrable timer group.

  @datatypes
  posal_timer_t \n
  #posal_timer_duration_t \n
  #posal_timer_src_t \n
  posal_timer_client_notification_type_t \n
  posal_signal_t

  @param[in] pTimer      Pointer to the POSAL timer object.
  @param[in] timerType   One of the following:
                          - #POSAL_TIMER_ONESHOT_DURATION
                          - #POSAL_TIMER_PERIODIC
                          - #POSAL_TIMER_ONESHOT_ABSOLUTE
                            @tablebulletend
  @param[in] clockSource Clock source is #POSAL_TIMER_USER.
  @param[in] notification_type    Defines the type of the object passed by the client as client info.
  @param[in] client_info_ptr      Context Pointer for the client can be signal or callback info. depends on "notification_type"
  @param[in] heap_id     heap id needed for malloc.
  @detdesc
  The caller must allocate the memory for the timer structure and pass the
  pointer to this function.
  @par
  After calling this function, call the appropriate start functions based on
  the type of timer.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  This function must be called before arming the timer. Must use posal_timer_destroy_v2() to destroy this timer. @newpage
 */
int32_t posal_timer_create_v2(posal_timer_t          *pp_timer,
                              posal_timer_duration_t timerType,
                              posal_timer_src_t      clockSource,
                              posal_timer_client_notification_type_t  notification_type,
                              void*                  client_info_ptr,
                              POSAL_HEAP_ID          heap_id);


/**
  Deletes an existing timer.

  @datatypes
  posal_timer_t

  @param[in] pTimer   Pointer to the POSAL timer object.

  @return
  Indication of success (0) or failure (nonzero).

  @dependencies
  The timer object must be created using posal_timer_create().
 */
ar_result_t posal_timer_destroy(posal_timer_t *pp_timer);

/**
  Deletes an existing V2 timer.

  @datatypes
  posal_timer_t

  @param[in] pTimer   Pointer to the POSAL timer object.

  @return
  Indication of success (0) or failure (nonzero).

  @dependencies
  The timer object must be created using posal_timer_create().
 */
ar_result_t posal_timer_destroy_v2(posal_timer_t *pp_timer);

/**
  Gets the duration of the specified timer.

  @datatypes
  posal_timer_t

  @param[in] pTimer   Pointer to the POSAL timer object.

  @return
  Duration of the timer, in microseconds.

  @dependencies
  The timer object must be created using posal_timer_create().
  @newpage
 */
uint64_t posal_timer_get_duration(posal_timer_t p_timer);

/**
  Creates a synchronous sleep timer. Control returns to the callee
  after the timer expires.

  @param[in] llMicrosec  Duration the timer will sleep.

  @return
  Always returns 0. The return type is int for backwards compatibility.

  @dependencies
  None.
 */
int32_t posal_timer_sleep(int64_t llMicrosec);

/**
  Gets the wall clock time.

  @return
  Returns the wall clock time in microseconds.

  @dependencies
  None.
 */
uint64_t posal_timer_get_time(void);

/**
  Gets the wall clock time in milliseconds.

  @return
  Returns the wall clock time in milliseconds.

  @dependencies
  None. @newpage
 */
uint64_t posal_timer_get_time_in_msec(void);


/**
  Restarts the absolute one-shot timer.

  @datatypes
  posal_timer_t

  @param[in] pTimer  Pointer to the POSAL timer object.
  @param[in] time    Absolute time of the timer, in microseconds.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  The timer must be created using posal_timer_create().
 */
int32_t posal_timer_oneshot_start_absolute(posal_timer_t p_timer, int64_t time);


/**
  Restarts the duration-based one-shot timer.

  @datatypes
  posal_timer_t

  @param[in] pTimer    Pointer to the POSAL timer object.
  @param[in] duration  Duration of the timer, in microseconds.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  The timer must be created using posal_timer_create(). @newpage
 */
int32_t posal_timer_oneshot_start_duration(posal_timer_t p_timer, int64_t duration);


/**
  Starts the periodic timer.

  @datatypes
  posal_timer_t

  @param[in] pTimer    Pointer to the POSAL timer object.
  @param[in] duration  Duration of the timer, in microseconds.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  The timer must be created using posal_timer_create().
 */
int32_t posal_timer_periodic_start(posal_timer_t p_timer, int64_t duration);

/**
  Starts the periodic timer after the specified duration.

  @datatypes
  posal_timer_t

  @param[in] pTimer    Pointer to the POSAL timer object.
  @param[in] duration  Duration of the timer, in microseconds.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  The timer must be created using posal_timer_create().
 */
int32_t posal_timer_periodic_start_with_offset(posal_timer_t p_timer, int64_t periodic_duration, int64_t start_offset);

/**
  Stops the timer.

  @datatypes
  posal_timer_t

  @param[in] pTimer   Pointer to the POSAL timer object.

  @return
  An indication of success (0) or failure (nonzero).

  @dependencies
  The timer must be created using posal_timer_create(). @newpage
 */
int32_t posal_timer_stop(posal_timer_t p_timer);

void posal_utimer_expiry_cb(void *data);

/**
  Gets the remaining duration of the timer.

  @datatypes
  posal_timer_t

  @param[in] pTimer   Pointer to the POSAL timer object.

  @return
  Remaining timer duration in usec.
 */
uint64_t posal_timer_get_remaining_duration(posal_timer_t p_obj);

/**
  Utility function to convert tick to timestamp as per Q-timer with 19.2MHz.

  @param[in] tick_count   tick_count Tick Count by DMA..

  @return
  Time stamp in Micro-Seconds.

  @dependencies
  None. @newpage
 */
uint64_t posal_convert_tick_to_time(uint64_t tick_count);

/**
   Gets the HW tick.

   @return
   HW tick.

   @dependencies
   None. @newpage
*/
uint64_t posal_timer_get_hw_ticks(void);


/**
  Utility function to convert timestamp to tick.

  @param[in] Time stamp in Micro-Seconds.

  @return
  tick_count   tick_count Tick Count.

  @dependencies
  None. @newpage
 */
uint64_t posal_timer_time_to_tick(uint64_t time_us);

/** @} */ /* end_addtogroup posal_timer */

#ifdef __cplusplus
}
#endif //__cplusplus
#endif // #ifndef POSAL_TIMER_H
