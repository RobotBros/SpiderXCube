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
 * $LastChangedRevision: 5755 $
 */

/** @file
@brief Functions for generating a PN9 sequence

@defgroup lib_pn9 pn9
@{
@ingroup rf_test
@brief PN9 Generator

@details Uses PN9 to generate a pseudo-random number sequence. The LFSR uses
x^9 + x^5 as the primitive polynom. pn9_get_byte() returns the sequence one
byte at the time. The following code shows how to generate 40 bits (5 bytes) of
the PN9 sequence
@code
uint8_t idx;
uint8_t sequence[5];

pn9_init();

for (idx = 0; idx < 5; idx++)
{
  sequence[idx] = pn9_get_byte();
}
@endcode 
 */

 
 
#ifndef PN9_H__
#define PN9_H__

#include <stdint.h>

/** Init CCITT PN9 sequence.
 *	This function sets the starting sequence of CCITT PN9 sequence.
 */
void pn9_init(void);

/** Get the next 8 bits of PN9 sequence.
 *	Each time this function is called the next 8 bit of
 *  the PN9 sequence is calculated and returned.
 */
uint8_t pn9_get_byte(void);

#endif //  PN9_H__

/** @} */
