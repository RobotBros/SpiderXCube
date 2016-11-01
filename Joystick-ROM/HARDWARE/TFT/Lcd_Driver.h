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
//------------------LCD 1.44 color-------------------// RGB555 5bit 16ɫ
#define BUTTON_UP MAGENTA
#define BUTTON_DOWN RED

//������ɫ
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
#define BROWN 					0XBC40 //��ɫ
#define BRRED 					0XFC07 //�غ�ɫ
#define GRAY  					0X8430 //��ɫ
#define COPPYB					0x0001				

//GUI��ɫ
#define DARKBLUE      	 			0X01CF	//����ɫ
#define LIGHTBLUE      	 		0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 		0X5458 	//����ɫ
//������ɫΪPANEL����ɫ 
 
#define LIGHTGREEN     			0X841F //ǳ��ɫ
#define LIGHTGRAY     			0XEF5B //ǳ��ɫ(PANNEL)
#define LGRAY 			 		0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE      			0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE          				0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)

////////////////////////////////////////////////////////////////////
//LED�˿ڶ���
#define LCD_LED        	PAout(6)		 	 //PA6    ������TFT - -LED
#define LCD_RS         	PBout(7)			 //PB7    ������TFT --RS
#define LCD_CS        	PBout(8)			 //PB8    ������TFT --CS
#define LCD_RST     	PCout(13)		 //PC13  ������TFT --RST
#define LCD_SCL        	PCout(14)		 //PC14  ������TFT -- CLK
#define LCD_SDA        	PBout(6)			 //PB6   ������TFT -- SDI

//#define LCD_DATAOUT(x) LCD_DATA->ODR=x; //�������
//#define LCD_DATAIN     LCD_DATA->IDR;   //��������

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
//	u16 width;			//LCD ���
//	u16 height;			//LCD �߶�
//	u16 id;				//LCD ID
//	u8  dir;				//���������������ƣ�0��������1��������	
//	u16	 wramcmd;		//��ʼдgramָ��
//	u16  setxcmd;		//����x����ָ��
//	u16  setycmd;		//����y����ָ��	 
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


