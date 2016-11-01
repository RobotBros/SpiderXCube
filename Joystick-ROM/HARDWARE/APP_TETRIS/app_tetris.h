  /**********************************************************
	*					俄罗斯方块函数文件
	*					@R
	*********************************************************	
    */

#ifndef _TETRIS_H
#define _TETRIS_H

#include "app_tft.h"
#define Vertical_boxs	14 
#define Horizontal_boxs	9
#define MAX_BOX		19

//俄罗斯方块相关宏定义
#define BUTTON_UP MAGENTA
#define BUTTON_DOWN RED

#define BSIZE 		5   	//方块大小
#define SYS_STARX 	2 
#define SYS_STARY 	2
#define SYS_ENDX  	108  //320*240 285  2-113 step 5
#define SYS_ENDY 	98  //185			2-103 step 5


#define able	1	//在判断是否移动函数中用到
#define unable	0

//定义board面板
struct BOARD
{
	u8 var;
	u16 color;
};

//定义box结构体
struct SHAPE
{
	u16 box;
	u16 clor;
	u16 next;
};


void Systick_Init(void);
void Frame_Init (void);
void Show_Box(u16 x ,u16 y,u8 nu);
void Eraser_Box(u16 x ,u16 y,u8 nu);
u8 Mov_Able(u16 x ,u16 y,u8 nu);
u8 Updata_Board(u16 x ,u16 y,u8 nu);
void Show_Point(u8 point);

#endif
