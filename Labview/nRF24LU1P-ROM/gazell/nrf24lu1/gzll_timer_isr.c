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
 * $LastChangedRevision: 230 $
 */

/** @file
 * @brief Gazell Link Layer nRF24LU1+ timer interrupt service routine.
 */

#include "gzll.h"

//lint -esym(552, timer_cnt) "symbol not accessed"
volatile uint16_t xdata timer_cnt = 0;        // General purpose variable to be used for utilizing timer for other purposes

T2_ISR()
{
  TF2 = 0;
  gzll_timer_isr_function();
  timer_cnt++;
}
