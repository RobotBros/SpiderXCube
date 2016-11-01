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
 * $LastChangedRevision: 2471 $
 */

/** @file
 * @brief
 * Implementation of Gazell pairing library common Device and Host functions
 */

#include "gzp.h"
#include "gzll.h"
#include "hal_aes.h"
#include <string.h>
#include <stdint.h>
#include "memdefs.h"

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
/**
  Constant holding "global pairing address".
*/
static code const uint8_t pairing_address[] = {0, GZP_ADDRESS};

/**
  Constant holding pre-defined "validation ID".
*/
static code const uint8_t gzp_validation_id[GZP_VALIDATION_ID_LENGTH] = GZP_VALIDATION_ID;

/**
  Constant holding pre-defined "secret key".
*/
static code const uint8_t gzp_secret_key[16] = GZP_SECRET_KEY;

/**
  Variable used for AES key selection.
*/
static xdata gzp_key_select_t gzp_key_select;

//-----------------------------------------------------------------------------
// Misc. external variables. 
//-----------------------------------------------------------------------------

static xdata uint8_t gzp_session_token[GZP_SESSION_TOKEN_LENGTH];
static xdata uint8_t gzp_dyn_key[GZP_DYN_KEY_LENGTH];

//-----------------------------------------------------------------------------
// Implementation common internal function
//-----------------------------------------------------------------------------
void gzp_update_radio_params(const uint8_t* system_address)
{
  uint8_t i;
  uint8_t channels[GZLL_MAX_CHANNEL_TAB_SIZE];
  uint8_t temp_address[GZLL_ADDRESS_WIDTH];

  // Configure "global" pairing address
  gzll_set_address(HAL_NRF_PIPE0, (uint8_t const*)pairing_address);

  if(system_address != NULL)
  {
    for(i = 0; i < GZP_SYSTEM_ADDRESS_WIDTH; i++)
    {
      temp_address[i + 1] = *(system_address + i);  
    }

    // Configure address for pipe 1 - 5. Address byte set to equal pipe number.
    for(i = 1; i < 6; i++)
    {
      temp_address[0] = i; 
      gzll_set_address((hal_nrf_address_t)i, temp_address);     
    }
  }

  gzp_generate_channels(&channels[0], &temp_address[1], gzll_get_channel_tab_size());                       
  
  // Write generated channel subset to Gazell Link Layer   
  gzll_set_channels(&channels[0], gzll_get_channel_tab_size());
}

void gzp_generate_channels(uint8_t* ch_dst, const uint8_t* system_address, uint8_t channel_tab_size)
{
  uint8_t binsize, spacing, i;

  binsize = (GZP_CHANNEL_MAX - GZP_CHANNEL_MIN) / channel_tab_size;

  ch_dst[0] = GZP_CHANNEL_LOW;
  ch_dst[channel_tab_size - 1] = GZP_CHANNEL_HIGH;

  for(i = 1; i < (channel_tab_size - 1); i++)
  {
    ch_dst[i] = (binsize * i) + (system_address[i % 4] % binsize);  
  }

  for(i = 1; i < channel_tab_size; i++)
  {
    spacing = (ch_dst[i] - ch_dst[i - 1]); 
    if(spacing < GZP_CHANNEL_SPACING_MIN)
    {
      ch_dst[i] += (GZP_CHANNEL_SPACING_MIN - spacing); 
    }
  } 
}

#ifndef GZP_CRYPT_DISABLE

void gzp_xor_cipher(uint8_t* dst, const uint8_t* src, const uint8_t* pad, uint8_t length)
{
  uint8_t i;

  for(i = 0; i < length; i++)
  {
    *dst = *src ^ *pad;
    dst++;
    src++;
    pad++;   
  }
}

bool gzp_validate_id(const uint8_t* id)
{
  return (memcmp(id, (void*)gzp_validation_id, GZP_VALIDATION_ID_LENGTH) == 0);
}

void gzp_add_validation_id(uint8_t* dst)
{
  memcpy(dst, (void const*)gzp_validation_id, GZP_VALIDATION_ID_LENGTH); 
}

void gzp_crypt_set_session_token(const uint8_t * token)
{
  memcpy(gzp_session_token, (void const*)token, GZP_SESSION_TOKEN_LENGTH);
}

void gzp_crypt_set_dyn_key(const uint8_t* key)
{
  memcpy(gzp_dyn_key, (void const*)key, GZP_DYN_KEY_LENGTH); 
}

void gzp_crypt_get_session_token(uint8_t * dst_token)
{
  memcpy(dst_token, (void const*)gzp_session_token, GZP_SESSION_TOKEN_LENGTH);
}

void gzp_crypt_get_dyn_key(uint8_t* dst_key)
{
  memcpy(dst_key, (void const*)gzp_dyn_key, GZP_DYN_KEY_LENGTH); 
}

void gzp_crypt_select_key(gzp_key_select_t key_select)
{
  gzp_key_select = key_select;
}

void gzp_crypt(uint8_t* dst, const uint8_t* src, uint8_t length)
{
  uint8_t i;
  uint8_t key[16];
  uint8_t iv[16];

  // Build AES key based on "gzp_key_select"
  switch(gzp_key_select)
  {
    case GZP_ID_EXCHANGE:
      memcpy(key, (void const*)gzp_secret_key, 16);
      break;
    case GZP_KEY_EXCHANGE:
      memcpy(key, (void const*)gzp_secret_key, 16);
      gzp_get_host_id(key);
      break;
    case GZP_DATA_EXCHANGE:
      memcpy(key, (void const*)gzp_secret_key, 16);
      memcpy(key, (void const*)gzp_dyn_key, GZP_DYN_KEY_LENGTH);
      break;
    default: 
      break;
  }  
  
  // Build init vector from "gzp_session_token"
  for(i = 0; i < 16; i++)
  {
    if(i < GZP_SESSION_TOKEN_LENGTH)
    {
      iv[i] = gzp_session_token[i];
    }
    else
    {
      iv[i] = 0;
    }
  }

  // Set up hal_aes using new key and init vector
  hal_aes_setup(false, ECB, key, NULL); // Note, here we skip the IV as we use ECB mode

  // Encrypt IV using ECB mode
  hal_aes_crypt(iv, iv);

  // Encrypt data by XOR'ing with AES output
  gzp_xor_cipher(dst, src, iv, length);
}

#endif                       