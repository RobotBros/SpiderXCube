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
 * @brief Library emulating an EEPROM in nRf24LE1
 *      
 * @defgroup lib_eeprom255 256 byte EEPROM emulator
 * @{
 * @ingroup lib
 *
 * @brief Library emulating a 255 byte EEPROM in nRF24LE1 using the non-volatile data memory.
 *
 * The library uses the two pages (total 512 bytes) of high endurance non-volatile data
 * memory for emulating a 255-byte EEPROM. 
 */
 
#ifndef LIB_EEPROM255_H__
#define LIB_EEPROM255_H__

#include <stdint.h>
#include <stdbool.h>
 
/** Function to write a byte to the EEPROM
 *  @param adr 8 bit address in EEPROM
 *  @param dat byte to write
 */
void lib_eeprom255_byte_write(uint8_t adr, uint8_t dat);

/** Function to write n bytes to the EEPROM
 *  @param adr 8 bit address in EEPROM
 *  @param *src pointer to bytes to write
 *  @param n number of bytes to write
 */
void lib_eeprom255_bytes_write(uint8_t adr, uint8_t *src, uint8_t n);

/** Function to read a byte from the EEPROM
 *  @param adr 8 bit address in EEPROM
 *  @return the byte read
 */
uint8_t lib_eeprom255_byte_read(uint8_t adr);

/** Function to read n bytes from the EEPROM
 *  @param adr 8 bit address in EEPROM
 *  @param *dst pointer to bytes to write
 *  @param n number of bytes to read
 */
void lib_eeprom255_bytes_read(uint8_t adr, uint8_t *dst, uint8_t n);

#endif // LIB_EEPROM255_H__
/** @} */
