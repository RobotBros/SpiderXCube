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

/** @file lib_crypt.c
 * 
 * @brief AES encyption lilbrary implementation
 *
 */

#include "lib_crypt.h"
#include "hal_aes.h"
#include "memdefs.h"

/**
The "aes_counter" is defined as shown below:
@code
15.14...                                             5..4...            ...1..0
|------------------------------------------------------|-----------------------|
|                 MS11B  (11 bytes)                    |    LS5B (5 bytes)     |
|------------------------------------------------------|-----------------------|
  (never incremented)                                   (increment before encr.)

*) MS11B: 11 most  significant bytes
*) LS5B :  5 least significant bytes
@endcode
*/
static uint8_t aes_counter[16];

void lib_crypt_init(uint8_t * key, const uint8_t * init_counter)
{
	hal_aes_setup(false, ECB, key, NULL);

	if(init_counter)
	{
		lib_crypt_set_counter(init_counter);
	}
}

void lib_crypt_set_counter(const uint8_t * counter)
{
	uint8_t i;
	for(i=0;i<16;i++)
	{
		aes_counter[i] = counter[i];
	}
}

void lib_crypt(uint8_t * dest_buf, const uint8_t * src_buf, uint8_t length,const uint8_t * ls5b_value)
{
	uint8_t i;
  uint8_t encr_buffer[16];   //AES buffer. Needed to do XOR

  //Set LS5B
	for(i=0;i<5;i++)
	{
		aes_counter[i] = ls5b_value[i];
	}	

  //Run AES with aes_counter
	hal_aes_crypt(encr_buffer,aes_counter);
	
  //Encrypt data, based on XOR operation in AES counter mode.
	for(i=0;i<length; i++)
	{
		dest_buf[i] = src_buf[i] ^ encr_buffer[i];
	}
}
