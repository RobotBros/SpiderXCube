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

#ifndef __LIB_ADNS7530__
#define __LIB_ADNS7530__
#include <stdint.h>
#include <stdbool.h>

// Configuration bits Register Settings
#define ADNS7530_NORMAL_OPERATION   0x00
#define ADNS7530_FORCE_REST1        0x10
#define ADNS7530_FORCE_REST2        0x20
#define ADNS7530_FORCE_REST3        0x30

// Configuration2 bit Register Settings
#define ADNS7530_FORCE_AWAKE        0x08
#define ADNS7530_2MS_RUN_RATE       0x00
#define ADNS7530_3MS_RUN_RATE       0x01
#define ADNS7530_4MS_RUN_RATE       0x02
#define ADNS7530_5MS_RUN_RATE       0x03
#define ADNS7530_6MS_RUN_RATE       0x04
#define ADNS7530_7MS_RUN_RATE       0x05
#define ADNS7530_8MS_RUN_RATE       0x06
#define ADNS7530_RES1600CPI         0x60
#define ADNS7530_RES1200CPI         0x40
#define ADNS7530_RES800CPI          0x20
#define ADNS7530_RES400CPI          0x00

#define ADNS7530_POWERUPRESET_INIT  0x5A

#define ADNS7530_LASERCTRL0_DEBUG   0xC0  // Address 0x1a
#define ADNS7530_LASERCTRL1_DEBUG   0x00  // Address 0x1f
#define ADNS7530_LSRPWRCFG0_DEBUG   0x80  // Address 0x1c, 0x41
#define ADNS7530_LSRPWRCFG1_DEBUG   0x7F  // Address 0x1d, 0xbe

// ADNS-7530 Register Addresses
#define ADNS7530_PRODUCTID_ADDR         0x00
#define ADNS7530_REVISIONID_ADDR        0x01
#define ADNS7530_MOTION_ADDR            0x02
#define ADNS7530_DELTAX_L_ADDR          0x03
#define ADNS7530_DELTAY_L_ADDR          0x04
#define ADNS7530_DELTAXY_H_ADDR         0x05
#define ADNS7530_SQUAL_ADDR             0x06
#define ADNS7530_SHUTTER_UPPER_ADDR     0x07
#define ADNS7530_SHUTTER_LOWER_ADDR     0x08
#define ADNS7530_MAXIMUM_PIXEL_ADDR     0x09
#define ADNS7530_PIXEL_SUM_ADDR         0x0A
#define ADNS7530_MINIMUM_PIXEL_ADDR     0x0B
#define ADNS7530_CRC0_ADDR              0x0C
#define ADNS7530_CRC1_ADDR              0x0D
#define ADNS7530_CRC2_ADDR              0x0E
#define ADNS7530_CRC3_ADDR              0x0F
#define ADNS7530_SELFTEST_ADDR          0x10

#define ADNS7530_CONFIG2_BITS_ADDR 		0x12
#define ADNS7530_RUN_DOWNSHIFT_ADDR 	0x13
#define ADNS7530_REST1_RATE_ADDR	 	0x14
#define ADNS7530_REST1_DOWNSHITF_ADDR	0x15
#define ADNS7530_REST2_RATE_ADDR	 	0x16
#define ADNS7530_REST2_DOWNSHITF_ADDR	0x17
#define ADNS7530_REST3_RATE_ADDR	 	0x18

#define ADNS7530_LASERCTRL0_ADDR        0x1A
#define ADNS7530_LSRPWRCFG0_ADDR        0x1C
#define ADNS7530_LSRPWRCFG1_ADDR        0x1D
#define ADNS7530_LASERCTRL1_ADDR        0x1F

#define ADNS7530_OBSERVATION_ADDR       0x2E

#define ADNS7530_PIXEL_GRAB_ADDR     	  0x35

#define ADNS7530_POWERUPRESET_ADDR      0x3A
#define ADNS7530_SHUTDOWN_ADDR          0x3B

#define ADNS7530_AUTO_RUN_ADDR			0x3D
#define ADNS7530_INVERSEREVISIONID_ADDR 0x3E
#define ADNS7530_INVERSEPRODUCTID_ADDR  0x3F

#define ADNS7530_MOTIONBURST_ADDR       0x42

// ADNS-7530 Read/Write configuration settings
#define ADNS7530_WRITE      0x80
#define ADNS7530_READ       0x00
#define ADNS7530_DUMMYDATA  0xFF
   
void hal_adns7530_enable(void);
void hal_adns7530_disable(void);
uint8_t hal_adns7530_readRegister(uint8_t regAddress);
void hal_adns7530_writeRegister(uint8_t regAddress, uint8_t innData);
void hal_adns7530_init();
bool hal_adns7530_readMotionBurst(int16_t *deltaX, int16_t *deltaY);
void hal_adns7530_mod_cpi();

#endif