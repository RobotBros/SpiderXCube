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

#include "nrf24le1.h"
#include "lib_adns7530.h"
#include "hal_spi.h"
#include "nrfr_mouse.h"

void uSecDelay(uint8_t len)
{
  len += (len>>1);
  while(len--);
}
// Approximately 2 mSec delay
void frameDelay()
{
  uint8_t frameDel = 200;
  while(frameDel--)uSecDelay(98);
}
void hal_adns7530_enable(void)
{
  SPI_CS = 0;
  uSecDelay(15);
}

void hal_adns7530_disable(void)
{
  uSecDelay(10);
  SPI_CS = 1;
}

uint8_t hal_adns7530_readRegister(uint8_t regAddress)
{
  uint8_t returnVal;
  hal_adns7530_enable();
  hal_spi_master_read_write(regAddress); // Supply the register address to read from 	
  uSecDelay(10);
  returnVal = hal_spi_master_read_write(0);
  hal_adns7530_disable();
  return (returnVal);
}

void hal_adns7530_writeRegister(uint8_t regAddress, uint8_t innData)
{
  hal_adns7530_enable();
  hal_spi_master_read_write(ADNS7530_WRITE | regAddress);  // Supply the register address to write to
  uSecDelay(10);
  hal_spi_master_read_write(innData); // Send out the data to be written to that register address
  hal_adns7530_disable();
}

void hal_adns7530_init()
{
  //  One-time initialization for ADNS-7530 optical sensor
  hal_adns7530_writeRegister(ADNS7530_POWERUPRESET_ADDR, ADNS7530_POWERUPRESET_INIT); // Power Up Reset register data
  frameDelay();
  hal_adns7530_writeRegister(ADNS7530_OBSERVATION_ADDR, 0x00); 
  frameDelay();
  hal_adns7530_readRegister(ADNS7530_OBSERVATION_ADDR); 

  frameDelay();
  
  hal_adns7530_readRegister(ADNS7530_MOTION_ADDR);
  hal_adns7530_readRegister(ADNS7530_DELTAX_L_ADDR);
  hal_adns7530_readRegister(ADNS7530_DELTAY_L_ADDR);
  hal_adns7530_readRegister(ADNS7530_DELTAXY_H_ADDR);  

  hal_adns7530_writeRegister(0x3c,0x27);
  hal_adns7530_writeRegister(0x22,0x0a);
  hal_adns7530_writeRegister(0x21,0x01);
  hal_adns7530_writeRegister(0x3c,0x32);
  hal_adns7530_writeRegister(0x23,0x20)	;
  hal_adns7530_writeRegister(0x3c,0x05);

  frameDelay();  // Delay to wait until sensor fully reset

  hal_adns7530_writeRegister(ADNS7530_LSRPWRCFG0_ADDR, 0x7f);  // LSRPWR CFG0 register data
  hal_adns7530_writeRegister(ADNS7530_LSRPWRCFG1_ADDR, 0x80); // LSRPWR CFG1 register data
  hal_adns7530_writeRegister(ADNS7530_LASERCTRL0_ADDR, 0xc0); // LASER CTRL0 register data
  hal_adns7530_writeRegister(ADNS7530_LASERCTRL1_ADDR, 0x00); // LASER CTRL1 register data

  hal_adns7530_writeRegister(ADNS7530_CONFIG2_BITS_ADDR, ADNS7530_RES800CPI | ADNS7530_8MS_RUN_RATE);
}

void hal_adns7530_mod_cpi()
{
  static xdata uint8_t cpi = ADNS7530_RES800CPI;

  cpi += 0x20;
  if(cpi > 0x60)
  {
    cpi = 0;
  }
  hal_adns7530_writeRegister(ADNS7530_CONFIG2_BITS_ADDR, cpi | ADNS7530_8MS_RUN_RATE);
}

bool hal_adns7530_readMotionBurst(int16_t *deltaX, int16_t *deltaY) {
  uint8_t motion;
  int16_t deltaX_l, deltaY_l, deltaX_h, deltaY_h, deltaXY_h;

  hal_adns7530_enable();
  hal_spi_master_read_write(ADNS7530_MOTIONBURST_ADDR);
  uSecDelay(10);

  motion = hal_spi_master_read_write(0);
  deltaX_l = hal_spi_master_read_write(0);   // Low byte
  deltaY_l = hal_spi_master_read_write(0);   // Low byte
  deltaXY_h = hal_spi_master_read_write(0);  // XY High nibble

  deltaX_h = ((int16_t)deltaXY_h << 4) & 0xF00;
  if(deltaX_h & 0x800)
  {
    deltaX_h |= 0xf000;
  }
  
  deltaY_h = ((int16_t)deltaXY_h << 8) & 0xF00;
  if(deltaY_h & 0x800)
  {
    deltaY_h |= 0xf000;
  }

  *deltaX = deltaX_h | deltaX_l;
  *deltaY = deltaY_h | deltaY_l;


  hal_adns7530_disable();

  // If motion occured
  if(motion & 0x80)
  {
    return true;
  }
  else
  {
    return false;
  }
}

