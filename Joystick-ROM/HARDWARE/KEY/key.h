#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"

 
#define	KEY_LEFT  	GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15)	//PC15
#define	KEY_UP	  	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3)		//PB3
#define 	KEY_RIGHT  	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)		//PB4
#define 	KEY_DOWN 	GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9)		//PB9
#define 	KEY_ENTER	GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)	//PA15

#define	KEY_ON		0
#define	KEY_OFF	1

//-------------按键按下键位---------------------
#define	KEY_PRESS_NULL	0
#define	KEY_PRESS_LEFT		1
#define	KEY_PRESS_UP		2
#define	KEY_PRESS_RIGHT	3
#define	KEY_PRESS_DOWN	4
#define	KEY_PRESS_ENTER	5

void Key_init(void);
void Key_detect(void);
void Key_action(void);
void Key_handle_timer(void);

#endif
