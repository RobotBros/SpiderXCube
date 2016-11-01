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
                                                 
/** @file
 * @brief Definition of compiler specific macros for assertion library
 */

#ifndef ASSERT_SETUP_H
#define ASSERT_SETUP_H

#define ASSERT_LINE __LINE__      // Compiler specific macro returning a string containing the current line number
#define ASSERT_FILE __FILE__      // Compiler specific macro returning a integer containing the current file name

#endif