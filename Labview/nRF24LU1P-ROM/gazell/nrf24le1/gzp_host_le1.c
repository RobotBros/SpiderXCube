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
 * Implementation of nRF24LU1+ specific Device functions for the Gazell pairing library.
 */  

#include "nrf24le1.h"
#include <gzp.h>
#include "hal_rng.h"
#include "hal_flash.h"
#include "memdefs.h"

__no_init xdata uint8_t gzp_session_token[GZP_SESSION_TOKEN_LENGTH];
__no_init xdata uint8_t gzp_dyn_key[GZP_DYN_KEY_LENGTH];

void gzp_host_chip_id_read(uint8_t *dst, uint8_t n)
{
  uint8_t i;
  
  if(hal_flash_byte_read(GZP_PARAMS_STORAGE_ADR + GZP_HOST_ID_LENGTH + 1) == 0xff)
  {
    hal_flash_byte_write((GZP_PARAMS_STORAGE_ADR + GZP_HOST_ID_LENGTH + 1), 0x00);
    hal_rng_power_up(true);
    for(i = 0; i < n; i++) 
    {
      while(!hal_rng_data_ready())
      ;
      hal_flash_byte_write((GZP_PARAMS_STORAGE_ADR + GZP_HOST_ID_LENGTH + 2 + i),  hal_rng_read());
    }
    hal_rng_power_up(false);
  }
  
  for(i = 0; i < n; i++) 
  {
    *(dst++) = hal_flash_byte_read((GZP_PARAMS_STORAGE_ADR + GZP_HOST_ID_LENGTH + 2 + i));
  }
}