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
 * $LastChangedRevision: 2685 $
 */

/** @file
 * @brief Gazell Link Layer nRF24LE1 specific functions implementation
 */

#include "gzll.h"
#include "hal_clk.h"
#include "hal_rtc.h"

void mcu_init(void)
{  
  hal_clklf_set_source(HAL_CLKLF_XOSC16M_SYNTH);    // Synthesize 32 KHz from 16 MHz clock  
  hal_rtc_set_compare_mode(HAL_RTC_COMPARE_MODE_0); // Use 32 KHz timer mode 0
  hal_clk_regret_xosc16m_on(true);                  // Keep XOSC16M on in register retention
  while (hal_clk_get_16m_source() != HAL_CLK_XOSC16M) {}
}

void gzll_set_timer_period(uint16_t period)
{
  hal_rtc_start(false);                             
  hal_rtc_start(true);                              // Start/stop to reset counter  

  period = (int)((float)period * 32768 / 1000000 + 0.5);

  hal_rtc_set_compare_value(period - 1);
}
