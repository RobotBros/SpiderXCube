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
 * $LastChangedRevision: 2494 $
 */ 

/** @file
 * @brief Implementation of library for using nRFgo display module
 */

#include "nrf24le1.h"
#include <stdint.h>
//#include <stdlib.h>
#include <string.h>
#include "nordic_common.h"
#include "hal_delay.h"
#include "lib_display.h"

#define DDRAM_ADR     0x80      // Write to DDRAM AC
#define DDRAM_WR      0x40      // Write to DDRAM
#define FUNC_SET      0x00      // Enter LCD Function settings
#define LCD_ADDR      0x3E      // LCD display adr
#define JS_ADDR       0x3F      // Joystick adr

#define X 0                     // Contents in joystick array
#define Y 1

void lcd_init(void)
{ 
  hal_w2_configure_master(HAL_W2_400KHZ);
  delay_us(100);
                      
  lcd_set_instruction(0x38);                      // Function set
  lcd_set_instruction(0x39);                      // Choose two-line mode
  lcd_set_instruction(0x14);                      // Internal OSC frequency
  lcd_set_contrast(LCD_CONTRAST_HIGH);            // Contrast set (low byte)
  lcd_set_instruction(0x5F);                      // Power/ICON control/
                                                  // Contrast set (high byte)
  lcd_set_instruction(0x6A);                      // Follower control
  delay_ms(200);

  lcd_on();                                       // Display ON
  lcd_clear();                                    // Clear display
  lcd_set_instruction(0x06);                      // Entry mode set    
}                                                                    

void lcd_write_string(char *text, uint8_t line, uint8_t pos)
{
  char str[18];
  uint8_t buffer[2];
  uint8_t i;
  
  if(line == 0)
  { 
    line = 0x00;                                  // Upper row of LCD display
  }
  else
  {
    line = 0x40;                                  // Lower row of LCD display
  }

  if(pos > 15) pos = 16;                          // Write to visible positions

  buffer[0] = FUNC_SET;                           // Enter function setting
  buffer[1] = DDRAM_ADR + (pos + line);           // LCD adr counter set to pos
  hal_w2_write(LCD_ADDR, buffer, 2);              // Write the settings to the 
                                                  // LCD display
  for(i=0;i<17;i++)                               // Save text in a new string
  {                                               // with space for function
    str[i+1] = text[i];                           // setting
  }
  str[0] = DDRAM_WR;                                  // Enter function setting
  hal_w2_write(LCD_ADDR, (const uint8_t *)str, strlen(text) + 1); // Transmit string to LCD
}

void lcd_clear(void)
{ 
  uint8_t buffer[2];

  delay_ms(10);
  buffer[0] = FUNC_SET; 
  buffer[1] = 0x01;                               // Clear display
  hal_w2_write(LCD_ADDR, buffer, 2);
  delay_ms(10);   
}

void lcd_set_contrast(lib_nrf6350_lcd_contrast_t contrast)
{
  uint8_t buffer[2];
 
  delay_ms(10);
  buffer[0] = FUNC_SET; 
  buffer[1] = 0x70 | contrast;                    // Contrast set (low byte)
  hal_w2_write(LCD_ADDR, buffer, 2);
  delay_ms(10);
}

void lcd_set_instruction(uint8_t instr)
{
  uint8_t buffer[2];
 
  delay_ms(10);
  buffer[0] = FUNC_SET; 
  buffer[1] = instr;                              // Instr. set
  hal_w2_write(LCD_ADDR, buffer, 2);
  delay_ms(10);
}

void lcd_on(void)
{
  uint8_t buffer[2];

  delay_ms(10);
  buffer[0] = FUNC_SET; 
  buffer[1] = 0x0C;                               // Display ON
  hal_w2_write(LCD_ADDR, &buffer[0], 2);
  delay_ms(10);
}

void lcd_off(void)
{
  uint8_t buffer[2];
 
  delay_ms(10);
  buffer[0] = FUNC_SET; 
  buffer[1] = 0x08;                               // Display OFF
  hal_w2_write(LCD_ADDR, buffer, 2);
  delay_ms(10);
}

void js_get_value(int8_t *val)
{
  uint8_t js_data;
  uint8_t rx_buffer[1];

  hal_w2_read(JS_ADDR, rx_buffer, 1);             // Get data from the joystick
  js_data = (~rx_buffer[0] & 0x1D);               // Select the useful bits
 
  if((js_data & BIT_0) == BIT_0)                  // Check joystick position
  {
    val[X] = -1;
  }
  else if((js_data & BIT_4) == BIT_4)
  {
    val[X] = 1;
  } 
  else
  {
    val[X] = 0;
  }
  
  if((js_data & BIT_2) == BIT_2)
  {
    val[Y] = 1;
  }
  else if((js_data & BIT_3) == BIT_3)
  {
    val[Y] = -1;
  }
  else
  {
    val[Y] = 0;
  }
}

bool js_button_pushed(void)
{
  uint8_t js_data;
  uint8_t rx_buffer[1];
 
  hal_w2_read(JS_ADDR, rx_buffer, 1);             // Get data from the joystick
  js_data = (~rx_buffer[0] & BIT_1);              // Mask button bit
         
  return (js_data == BIT_1);                      // Check if button is pushed
}

lib_display_js_states_t js_get_status()
{
  uint8_t js_data;

  hal_w2_read(JS_ADDR, &js_data, 1);              // Get data from the joystick
  js_data = ~js_data;   
  if( js_data & 0x02 ) return JS_BUTTON_PUSH;
  if( js_data & 0x01 ) return JS_BUTTON_LEFT;
  if( js_data & 0x10 ) return JS_BUTTON_RIGHT;
  if( js_data & 0x08 ) return JS_BUTTON_UP;
  if( js_data & 0x04 ) return JS_BUTTON_DOWN;
  return JS_BUTTON_NONE;
} 
