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
 * @brief Implementation of Gazell Link Layer nRF24LU1+ specific functions
 */

#include "gzll.h"

void mcu_init()
{
  RF = 1;                                       // Radio IRQ enable
  
  CE_LOW();
  RFCTL = 0x10;                                 // RF SPI Enable 
   
  T2CON = 0x10;                                 // Reload mode 0, osc / 12 
  T2I0 = 1;                                     // Start Timer2
}

void gzll_set_timer_period(uint16_t period)
{
  T2 = CRC = (uint32_t)0x10000 - (int)((float)period * 4 / 3 + 0.5);             // Set up period for timer 2
}
