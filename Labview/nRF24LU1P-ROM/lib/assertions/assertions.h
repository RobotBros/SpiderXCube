/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic 
 * Semiconductor ASA.Terms and conditions of usage are described in detail 
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT. 
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *              
 * $LastChangedRevision: 133 $
 */                                              

/** 
 * @file
 * @brief An assertions library
 * @defgroup lib_assert Assertions
 * @{
 * @ingroup lib
 *
 * @brief Assertion library
 */

#ifndef ASSERT_H
#define ASSERT_H

#include "assertions_setup.h"
#include "stdint.h"

#ifdef ASSERT_ENABLE

/**
Function that is called  whenever an asserion is triggered. 

@param line is an integer holding the line number on which the asserion was called.

@param *file [in] is a string holding the name of the file in which the assertion was called.

The implementation of this function must be customized by the end user.
*/
void assert_handler(uint16_t line, char* file);

/**
Macro for calling an assert. This macro can be called by any firmware module in order
to trigger an assertion.  

@param message should a pointer to a string description of the assertion.
    
*/
#define ASSERT(condition) do{ \
  {                                                    \
    if(!(condition))                                      \
    {                                                  \
      assert_handler(ASSERT_LINE, ASSERT_FILE); \
    }                                                  \
  }                                                    \
} while(0);

#else

#define ASSERT(a)

/** @} */
#endif
#endif