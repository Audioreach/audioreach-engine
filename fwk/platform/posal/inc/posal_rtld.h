
/**
 * \file posal_rtld.h
 * \brief 
 *  	 This file contains the Run-Time Linking (rtld) utilities. 
 * 
 * \copyright
 *  Copyright (c) Qualcomm Innovation Center, Inc. All Rights Reserved.
 *  SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef POSAL_RTLD_H
#define POSAL_RTLD_H

/*
 * Values for dlopen `flags'
 */
 #define POSAL_RTLD_LAZY 1
 #define POSAL_RTLD_NOW 2
 

 /** ideally dlfcn.h must define these macros. but in some targets they are not defined. this is a work-around.*/
#ifndef RTLD_DI_LOAD_ADDR
#define RTLD_DI_LOAD_ADDR 3
#endif

#ifndef RTLD_DI_LOAD_SIZE
#define RTLD_DI_LOAD_SIZE 4
#endif

/*
 * User interface to the run-time linker
 */

/**
  Opens the specified dynamic lib

  @datatypes
  const char*, int

  @param[in]    name    Name of the file to be opened
  @param[in]    flags   Flags for to indicate how the file should be opened. Possible values are 
    POSAL_RTLD_LAZY and POSAL_RTLD_NOW

  @return
  Returns a pointer to the opened dynamic library, or 0 on failure

  @dependencies
  None. @newpage
*/
void* posal_dlopen(const char* name, int flags);

/**
  Opens the specified dynamic lib located in the given buffer

  @datatypes
  const char*, const char*, int

  @param[in]    name    Name of the file to be opened
  @param[in]    buf     Buffer of the dynamic library
  @param[in]    flags   Flags for to indicate how the file should be opened. Possible values are 
    POSAL_RTLD_LAZY and POSAL_RTLD_NOW

  @return
  Returns a pointer to the opened dynamic library, or 0 on failure

  @dependencies
  None. @newpage
*/
void* posal_dlopenbuf(const char* name, const char* buf, int len, int flags);

/**
  Closes the specified dynamic lib

  @datatypes
  void *

  @param[in]    handle    The handle of the dl to be closed 

  @return
  0 -- Success

  @dependencies
  None. @newpage
*/
int posal_dlclose(void* handle);

/**
  Gets the pointer to the symbol within the dnamic lib

  @datatypes
  void *, const char *

  @param[in]    handle    The handle of the dl with the symbol needed 
  @param[in]    name      The name of the symbol

  @return
  Returns a pointer to the requested symbol, or 0 on failure

  @dependencies
  None. @newpage
*/
void* posal_dlsym(void* handle, const char* name);

/**
  Gives the string of the error if there is a problem in one of the dl functions

  @return
  Returns a pointer to error string

  @dependencies
  None. @newpage
*/
char* posal_dlerror(void);

/**
  Gets info about the dynamic lib based on the request

  @datatypes
  void *, int, void *

  @param[in]    handle    The handle of the dl in question
  @param[in]    request   The value of the request
  @param[out]   p         The output of the request

  @return
  0 -- Success

  @dependencies
  None. @newpage
*/
int posal_dlinfo(void* handle, int request, void* p);

#endif // POSAL_RTLD_H
