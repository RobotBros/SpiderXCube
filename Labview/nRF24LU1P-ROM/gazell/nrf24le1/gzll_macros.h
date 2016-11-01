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
 * @brief Gazell Link Layer nRF24LE1 specific macros
 */
 
/** @ingroup nordic_gzll */

/** @name Hardware dependent macros for the Gazell Link Layer.  
@{
*/

#include "nrf24le1.h"

#ifndef GZLL_MACROS_H__
#define GZLL_MACROS_H__

/** @brief Defines the radio interrupt enable bit*/
#define RFEN RF

/** @brief Defines the timer interrupt enable bit*/
#define TIMEREN WUIRQ

#endif // GZLL_MACROS_H__

/** @} */
