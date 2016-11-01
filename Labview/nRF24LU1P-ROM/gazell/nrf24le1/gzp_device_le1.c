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
 * @brief
 * Implementation of nRF24LE1 specific Device functions for the Gazell pairing library.
 */  

#include <stdint.h>
#include <stdbool.h>

#include "gzll.h"
#include "gzp.h"
#include "hal_rng.h"

//-----------------------------------------------------------------------------
// MCU specific function implementations
//-----------------------------------------------------------------------------

void gzp_random_numbers_generate(uint8_t* dst, uint8_t n)
{
  uint8_t i;

  hal_rng_power_up(true);
  for(i = 0; i < n; i++) 
  {
    while(!hal_rng_data_ready())
    ;
    *(dst++) = hal_rng_read();
  }
  hal_rng_power_up(false);
}