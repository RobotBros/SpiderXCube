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
 * @brief Implementation of AES encyption library nRF24LE1 specific functions
 */

 //lint -e714
 
#include "hal_rng.h"

void lib_crypt_generate_ls5b(uint8_t * dest_buf)
{ 
  uint8_t i; 
  hal_rng_power_up(true);

  for(i=0;i<5;i++)
	{	
    while(!hal_rng_data_ready())
    {}  
    dest_buf[i] = hal_rng_read();
	} 
  
  hal_rng_power_up(false);
}
    