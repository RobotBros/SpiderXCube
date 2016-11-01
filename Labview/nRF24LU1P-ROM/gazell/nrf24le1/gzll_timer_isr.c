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
 * @brief Gazell Link Layer nRF24LE1 radio interrupt service routine.
 */

#include "gzll.h"
#include "isrdef24le1.h"

TICK_ISR()
{     
  gzll_timer_isr_function();                         
}