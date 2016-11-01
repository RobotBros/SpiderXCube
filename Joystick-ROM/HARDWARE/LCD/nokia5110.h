#ifndef __NOKIA5110_H
#define __NOKIA5110_H	 
#include "sys.h"

//LED端口定义
#define LCD_BL 			PAout(6)
#define LCD_RST			PCout(13)
#define LCD_CLK			PCout(14)
#define LCD_DIN			PBout(6)
#define LCD_DC			PBout(7)
#define LCD_CE			PBout(8)


#define LCD_BL_SCALE	7199				//100us 10kHz FRQ
#define LCD_BL_DIV		0

void LCD_init(void);
void LCD_Nokia5110_Init(u16 arr,u16 psc);			//初始化	
void LCD_BL_PWM_duty_set(u8 duty);
void LCD_write_byte(u8 data, u8 command);
void LCD_set_XY(u8  x, u8 y);
void LCD_clear(void);
void LCD_write_char(u8 c);
void LCD_write_String(u8 x,u8 y,char *s);
void LCD_write_shu(u8 row, u8 page, u8 c);
void LCD_write_hanzi(u8 row, u8 page,u8 c);
void LCD_write_number(u8 x,u8 y,u8 number,u8 len);
void LCD_logo_write_String(u8 x,u8 y,char *s);
void LCD_power_on_action(void);
void LCD_backlight_set_timer(void);
void LCD_backlight_set(void);
void LCD_fault_display(u8 fault);
void LCD_write_logo(u8 x,u8 y,u8 *s,u8 pix_x,u8 pix_y);

#endif

















