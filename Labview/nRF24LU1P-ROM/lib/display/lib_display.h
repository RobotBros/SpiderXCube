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
 * $LastChangedRevision: 183 $
 */
 
/** @file
 * @brief nRF6350 LCD display and joystick library header file.
 * @defgroup lib_display nRFgo LCD and joystick
 * @{
 * @ingroup lib
 *
 * @brief Library for the nRF6350 LCD display and joystick module
 *
 * The 2 wire interface of the nRF24LE1 on the nRFgo Development Kit modules are routed to the IO port
 * sharing pins with it. To enable connection to a nRFgo Display module fitted in the extension port of the
 * nRFgo Motherboard the 2 wire interface of the mounted nRF24LE1 must also be routed to the 2 wire interface
 * of the nRFgo Development Kit module. This is done by shorting the SDA and SCL solder bridges SB1
 * and SB2 found on the nRF24LE1 development kit
 */


#ifndef LIB_NRF6350_H__
#define LIB_NRF6350_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf24le1.h"
#include "hal_w2.h"

/** Enum describing the contrast of lcd display.
 *
 */  
typedef enum {
  LCD_CONTRAST_LOW     = 0x00,
  LCD_CONTRAST_MEDIUM  = 0x02,
  LCD_CONTRAST_HIGH    = 0x08
} lib_nrf6350_lcd_contrast_t;

/** Enum describing the different states of the joystick.
 *
 */ 
typedef enum {
  JS_BUTTON_NONE   = 0,
  JS_BUTTON_PUSH   = 1,
  JS_BUTTON_LEFT   = 2,
  JS_BUTTON_RIGHT  = 3,
  JS_BUTTON_UP     = 4,
  JS_BUTTON_DOWN   = 5
} lib_display_js_states_t;

/** Function to initialize the LCD display prior to writing.
 */
void lcd_init(void);

/** Function for writing a text string on the LCD-display.
 *
 * @param *text A pointer to the text string to be written
 * @param line The line the text should be written to
 * @param pos The start position of the text on the line
 */
void lcd_write_string(char *text, uint8_t line, uint8_t pos);

/** This function clears the contents of the LCD-display.
 */
void lcd_clear(void);

/** This function adjust the contrast of the LCD-display, select between
 * LCD_CONTRAST_LOW, LCD_CONTRAST_MEDIUM and LCD_CONTRAST_HIGH
 *
 * @param contrast The desired contrast of the lcd display
 */
void lcd_set_contrast(lib_nrf6350_lcd_contrast_t contrast);

/** This function writes instructions to the LCD display
 *
 * @remark See LCD datasheet.
 * @param instr The desired instruction
 */
void lcd_set_instruction(uint8_t instr);

/** Function that turns ON the LCD-display
 */
void lcd_on(void);

/** Function that turns OFF the LCD-display
 */
void lcd_off(void);

/** This function gets the position of the joystick.
 *
 * @param val pointer to a 2 byte array where the X,Y position is stored
 */
void js_get_value(int8_t *val);

/** This function checks if the joystick is pushed down
 *
 * @return True if joystick is pushed down, false if released
 */
bool js_button_pushed(void);

/** This function gets the status of the joystick
 *
 * @return The status of the joystick as defined in the lib_display_js_states_t struct
 */
lib_display_js_states_t js_get_status(void);

#endif  // LIB_NRF6350
/** @} */
