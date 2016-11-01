#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H	 
#include "sys.h"
#include "gui.h"

//------------------LCD 1.44 inch pix-----------------//
#define LCD_X_MAX_PIXEL	       128
#define LCD_Y_MAX_PIXEL	       128
//-------------LCD 1.44 backlight PWM FRQ-------------//
#define LCD_BL_SCALE			7199				//100us 10kHz FRQ
#define LCD_BL_DIV				0
//------------------LCD 1.44 color-------------------// RGB555 5bit 16色
#define BUTTON_UP MAGENTA
#define BUTTON_DOWN RED

//画笔颜色
#define WHITE      				0xFFFF
#define BLACK      				0x0000	  
#define BLUE       					0x001F  
#define BLUE2          				0x051F
#define BRED        				0XF81F
#define GRED 			 		0XFFE0
#define GBLUE			 		0X07FF
#define GREY						0xF7DE
#define RED         					0xF800
#define MAGENTA     				0xF81F
#define GREEN       				0x07E0
#define CYAN        				0x7FFF
#define YELLOW      				0xFFE0
#define BROWN 					0XBC40 //棕色
#define BRRED 					0XFC07 //棕红色
#define GRAY  					0X8430 //灰色
#define COPPYB					0x0001				

//GUI颜色
#define DARKBLUE      	 			0X01CF	//深蓝色
#define LIGHTBLUE      	 		0X7D7C	//浅蓝色  
#define GRAYBLUE       	 		0X5458 	//灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     			0X841F //浅绿色
#define LIGHTGRAY     			0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 		0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE      			0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE          				0X2B12 //浅棕蓝色(选择条目的反色)

////////////////////////////////////////////////////////////////////
//LED端口定义
#define LCD_LED        	PAout(6)		 	 //PA6    连接至TFT - -LED
#define LCD_RS         	PBout(7)			 //PB7    连接至TFT --RS
#define LCD_CS        	PBout(8)			 //PB8    连接至TFT --CS
#define LCD_RST     	PCout(13)		 //PC13  连接至TFT --RST
#define LCD_SCL        	PCout(14)		 //PC14  连接至TFT -- CLK
#define LCD_SDA        	PBout(6)			 //PB6   连接至TFT -- SDI

//#define LCD_DATAOUT(x) LCD_DATA->ODR=x; //数据输出
//#define LCD_DATAIN     LCD_DATA->IDR;   //数据输入

//#define LCD_WR_DATA(data){\
//LCD_RS_SET;\
//LCD_CS_CLR;\
//LCD_DATAOUT(data);\
//LCD_WR_CLR;\
//LCD_WR_SET;\
//LCD_CS_SET;\
//} 

extern u16 Back_color, Point_color ;  
//typedef struct  
//{										    
//	u16 width;			//LCD 宽度
//	u16 height;			//LCD 高度
//	u16 id;				//LCD ID
//	u8  dir;				//横屏还是竖屏控制：0，竖屏；1，横屏。	
//	u16	 wramcmd;		//开始写gram指令
//	u16  setxcmd;		//设置x坐标指令
//	u16  setycmd;		//设置y坐标指令	 
//}_lcd_dev; 	

void Lcd_144inch_IO_Init(u16 arr,u16 psc);
void  SPI_WriteData(u8 Data);
void Lcd_WriteIndex(u8 Index);
void Lcd_WriteData(u8 Data);
void LCD_WriteData_16Bit(u16 Data);
void Lcd_WriteReg(u8 Index,u8 Data);
void Lcd_Reset(void);
void Lcd_144inch_Init(void);
void Lcd_SetRegion(u16 x_start,u16 y_start,u16 x_end,u16 y_end);
void Lcd_SetXY(u16 x,u16 y);
void Lcd_DrawPoint(u16 x,u16 y);
unsigned int Lcd_ReadPoint(u16 x,u16 y);
void Lcd_Clear(u16 Color,u8 x,u8 y);    
void Lcd_Init_Clear(u16 Color);    
void LCD_BL_PWM_duty_set(u8 duty);
void LCD_fault_display(u8 fault);
void LCD_backlight_set_timer(void);
void LCD_backlight_set(void);

#endif


