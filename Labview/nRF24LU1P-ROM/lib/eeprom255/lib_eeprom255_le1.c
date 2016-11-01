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
 * @brief Implementation of EEPROM library (lib_eeprom255) in nRF24LE1.
 */
 
#ifndef LIB_EEPROM255_H__
#define LIB_EEPROM255_H__

#include "nrf24le1.h"
#include "hal_flash.h"

#define BK_BYTE 255
#define PAGE_0_XDATA 0xFA00
#define PAGE_1_XDATA 0xFB00
#define PAGE_0_FLASH_PN 32
#define PAGE_1_FLASH_PN 33
 
void lib_eeprom255_byte_write(uint8_t adr, uint8_t dat)
{
  uint8_t xdata i;
  uint16_t xdata flash_dst_pn, xdata_dst_adr, flash_old_pn, xdata_old_adr; 
  
  if(*((uint8_t xdata *)(PAGE_0_XDATA + BK_BYTE)) == 0xff)
  {
    flash_dst_pn = PAGE_0_FLASH_PN;
    xdata_dst_adr = PAGE_0_XDATA;  
    flash_old_pn = PAGE_1_FLASH_PN;
    xdata_old_adr = PAGE_1_XDATA;   
  }
  else
  {
    flash_dst_pn = PAGE_1_FLASH_PN;
    xdata_dst_adr = PAGE_1_XDATA;  
    flash_old_pn = PAGE_0_FLASH_PN;
    xdata_old_adr = PAGE_0_XDATA;   
  }
  
  if((*((uint8_t xdata *)(xdata_old_adr + adr))) != dat)
  {
    hal_flash_page_erase(flash_dst_pn);
    
    PCON &= ~(1 << 4);
    for(i = 0; i < 255; i++)
    {
      if(i == adr)
      {
        hal_flash_byte_write(xdata_dst_adr + adr, dat);
      }
      else
      {
        hal_flash_byte_write(xdata_dst_adr + i, *((uint8_t xdata *)(xdata_old_adr + i)));
      }
    }
  
    hal_flash_byte_write(xdata_dst_adr + BK_BYTE, 0);
    hal_flash_page_erase(flash_old_pn);
  }
}

uint8_t lib_eeprom255_byte_read(uint8_t adr)
{
  if(*((uint8_t xdata *)(PAGE_0_XDATA + BK_BYTE)) == 0)
  {
    return (*((uint8_t xdata *)(PAGE_0_XDATA + adr)));
  }
  else
  {
    return (*((uint8_t xdata *)(PAGE_1_XDATA + adr)));
  }
}

void lib_eeprom255_bytes_write(uint8_t adr, uint8_t *src, uint8_t n)
{
  uint8_t xdata i;
  uint16_t xdata flash_dst_pn, xdata_dst_adr, flash_old_pn, xdata_old_adr; 
  
  if(*((uint8_t xdata *)(PAGE_0_XDATA + BK_BYTE)) == 0xff)
  {
    flash_dst_pn = PAGE_0_FLASH_PN;
    xdata_dst_adr = PAGE_0_XDATA;  
    flash_old_pn = PAGE_1_FLASH_PN;
    xdata_old_adr = PAGE_1_XDATA;   
  }
  else
  {
    flash_dst_pn = PAGE_1_FLASH_PN;
    xdata_dst_adr = PAGE_1_XDATA;  
    flash_old_pn = PAGE_0_FLASH_PN;
    xdata_old_adr = PAGE_0_XDATA;   
  }

  for(i = 0; i < n; i++)
  {
    if((*((uint8_t xdata *)(xdata_old_adr + adr + i))) != src[i])
    {
      break;
    }
  }

  if(i < n)
  {
    hal_flash_page_erase(flash_dst_pn);
    for(i = 0; i < 255; i++)
    {
      if(i >= adr && i < (adr + n))
      {
        hal_flash_byte_write(xdata_dst_adr + i, src[i - adr]);
      }
      else
      {
        hal_flash_byte_write(xdata_dst_adr + i, *((uint8_t xdata *)(xdata_old_adr + i)));
      }
    }
  
    hal_flash_byte_write(xdata_dst_adr + BK_BYTE, 0);
    hal_flash_page_erase(flash_old_pn);
  }
}

void lib_eeprom255_bytes_read(uint8_t adr, uint8_t *dst, uint8_t n)
{
  uint8_t xdata i;
  uint16_t xdata xdata_src_adr;

  if(*((uint8_t xdata *)(PAGE_0_XDATA + BK_BYTE)) == 0)
  {
    xdata_src_adr = PAGE_0_XDATA;
  }
  else
  {
    xdata_src_adr = PAGE_1_XDATA;
  }
  
  for(i = 0; i < n; i++)
  {
    dst[i] = *((uint8_t xdata *)(xdata_src_adr + adr + i));
  }  
}

#endif // LIB_EEPROM255_H__