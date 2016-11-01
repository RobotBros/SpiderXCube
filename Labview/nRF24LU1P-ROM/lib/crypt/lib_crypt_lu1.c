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
 * @brief Implementation of AES encyption library nRF24LU1+ specific functions
 */
 
#include <stdint.h>
#include <stdbool.h>
#include "nrf24lu1p.h"

void lib_crypt_generate_ls5b(uint8_t * dest_buf)
{
  uint8_t i;
  bool wrap = true;
  static xdata uint8_t ls5b[5] = {0, 0, 0, 0, 0};
  
  //Increment LS5B, and write back the new LS5B.
  for(i=0;i<5;i++)
	{		
	 	if(wrap)  //Check if we need to increment position i.
	 	{
	 	  ls5b[i]++;
      if(ls5b[i] != 0x00) wrap = false;	
		}

    //Write out LS5B
    dest_buf[i] = ls5b[i]; 
	} 
}