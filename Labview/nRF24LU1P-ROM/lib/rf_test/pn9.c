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
 * $LastChangedRevision: 5717 $
 */

/** @file
 * 
 * @brief Implementation of a PN9 Generator
 *
 * Uses PN9 to generate a pseudo-random number sequence. The LFSR uses
 * x^9 + x^5 as the primitive polynom. 
 */

#include "pn9.h"

#define pn9_bit5 0x08
#define pn9_bit9 0x80

/** The upper 8 bits of 9 */
static uint8_t bits_9_to_2;
/** The lowest bit */
static bit bit_1;

void pn9_init(void)
{
	bits_9_to_2 = 0xFF;
	bit_1 = 1;
}

uint8_t pn9_get_byte(void)
{
	bit feedback;
	uint8_t i, out_value;

	out_value = bits_9_to_2;
	
	for (i = 0; i < 8; i++)
	{
		// Tap the register
		feedback =  
			((((bits_9_to_2 & pn9_bit9)>>4) ^ (bits_9_to_2 & pn9_bit5)) == 0) ? 0 : 1;
		// Shift
		bits_9_to_2<<=1;
		bits_9_to_2 |= (uint8_t) bit_1;
		// Enter feedback
		bit_1 = feedback;
	} 

	return out_value;
}
