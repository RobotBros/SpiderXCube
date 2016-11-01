/* Copyright (c) 2008 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT. 
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 133 $
 */ 

/** @file
 * @author Ole Saether
 */

#include "hal_flash.h"
#include "lib_eeprom.h"

#define EEPROM_SIZE         16

#define EEPROM_PAGES (HAL_FLASH_PAGE_SIZE/EEPROM_SIZE)

static uint8_t xdata *fa;
static uint8_t eeprom_buf[EEPROM_SIZE];

uint8_t xdata *get_eeprom_address(uint8_t a)
{
    uint8_t i;
    uint8_t xdata *p;

    fa = (uint8_t xdata *)HAL_DATA_NV_BASE_ADDRESS;
    p = (uint8_t xdata *)(HAL_DATA_NV_BASE_ADDRESS + EEPROM_SIZE + a);
    for(i=0;i<EEPROM_PAGES;)
    {
        if (*fa == 0xff)
            break;
        i++;
        p = (uint8_t xdata *)(p + EEPROM_SIZE);
        if (*fa == 0xf0)
            break;
        i++;
        p = (uint8_t xdata *)(p + EEPROM_SIZE);
        fa++;
    }
    return p;
}

void lib_eeprom_byte_write(uint8_t adr, uint8_t dat)
{
    uint8_t i;
    uint8_t xdata *ea = get_eeprom_address(adr);
    if (dat == *ea)
        return;
    if (*ea != 0xff)
    {
        ea -= adr;
        for(i=0;i<EEPROM_SIZE;i++)
            eeprom_buf[i] = *ea++;
        eeprom_buf[adr] = dat;
        if (ea == (HAL_DATA_NV_BASE_ADDRESS + HAL_FLASH_PAGE_SIZE))
        {
          hal_flash_page_erase(HAL_DATA_NV_FLASH_PN0);
         
          if(HAL_DATA_NV_FLASH_PAGES == 2)
          {
            hal_flash_page_erase(HAL_DATA_NV_FLASH_PN1);
          }
             
          ea = (uint8_t xdata *)(HAL_DATA_NV_BASE_ADDRESS + EEPROM_SIZE);
        }
        else
        {
            if (*fa == 0x00)
                fa++;
            if (*fa == 0xff)
                hal_flash_byte_write((uint16_t)fa, 0xf0);
            else
                hal_flash_byte_write((uint16_t)fa, 0x00);
        }
        for(i=0;i<EEPROM_SIZE;i++)
        {
            hal_flash_byte_write((uint16_t)ea, eeprom_buf[i]);
            ea++;
        }
    }
    else
    {
        // When the byte at current location is 0xff we write the
        // byte directly to the flash:
        hal_flash_byte_write((uint16_t)ea, dat);
    }
}

void lib_eeprom_bytes_write(uint8_t adr, uint8_t *p, uint8_t n)
{
    while(n--)
        lib_eeprom_byte_write(adr++, *p++);
}

uint8_t lib_eeprom_byte_read(uint8_t adr)
{
    uint8_t xdata *fa = get_eeprom_address(adr);
    return *fa;
}

void lib_eeprom_bytes_read(uint8_t adr, uint8_t *p, uint8_t n)
{
    uint8_t xdata *fa = get_eeprom_address(adr);
    while(n--)
    {
        *p++ = *fa++;
    }
}
