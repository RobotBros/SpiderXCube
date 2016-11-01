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
 * @brief
 */

#include <stdint.h>
#include "nrf24lu1p.h"
#include <gzp.h>
#include "memdefs.h"

__no_init uint8_t gzp_session_token[GZP_SESSION_TOKEN_LENGTH];
__no_init uint8_t gzp_dyn_key[GZP_DYN_KEY_LENGTH];

void gzp_host_chip_id_read(uint8_t *dst, uint8_t n)
{
  uint8_t i;
#ifdef __C51__
  volatile uint8_t xdata *gzp = (uint8_t xdata *)0x0b;							// 0x0b is start of chip ID in info-page
#else
  __no_init volatile uint8_t *gzp;        // 0x0b is start of chip ID in info-page
  *gzp = 0x0b;
#endif
	uint8_t temp_infen;
  temp_infen = INFEN;

  INFEN = 1;
  // Read n bytes of chip ID from info page
  for(i = 0; i < n; i++)
  {
    *(dst + i) = *(gzp + i);
  }
  INFEN = temp_infen;
}
