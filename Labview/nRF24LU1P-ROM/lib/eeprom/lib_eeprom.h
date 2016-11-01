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
 * @brief Library emulating a high endurance 16 byte EEPROM
 *      
 * @defgroup lib_eeprom16 16 byte EEPROM emulator
 * @{
 * @ingroup lib
 *
 * @brief Library emulating a 16 byte high endurance EEPROM using one page of
 * on-chip Flash memory.
 */

#ifndef LIB_EEPROM16_H__
#define LIB_EEPROM16_H__

#include "hal_flash.h"

/** Function to write a byte to the EEPROM
 *  @param adr 8 bit address in EEPROM
 *  @param dat byte to write
 */
void lib_eeprom_byte_write(uint8_t adr, uint8_t dat);

/** Function to write n bytes to the EEPROM
 *  @param adr 8 bit address in EEPROM
 *  @param *p pointer to bytes to write
 *  @param n number of bytes to write
 */
void lib_eeprom_bytes_write(uint8_t adr, uint8_t *p, uint8_t n);

/** Function to read a byte from the EEPROM
 *  @param adr 8 bit address in EEPROM
 *  @return the byte read
 */
uint8_t lib_eeprom_byte_read(uint8_t adr);

/** Function to read n bytes from the EEPROM
 *  @param adr 8 bit address in EEPROM
 *  @param *p pointer to bytes to write
 *  @param n number of bytes to read
 */
void lib_eeprom_bytes_read(uint8_t adr, uint8_t *p, uint8_t n);

#endif // LIB_EEPROM_H__
/** @} */
