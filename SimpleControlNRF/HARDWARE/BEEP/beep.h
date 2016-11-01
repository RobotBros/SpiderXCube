#ifndef __BEEP_H
#define __BEEP_H	 
#include "sys.h"

//beep端口定义

#define BEEP 	PCout(14)	
#define SYS_SW	PCout(15)	

//***************蜂鸣器及舵机电源总开关****************//
#define ON							1
#define OFF							0

void BEEP_Init(void);    //初始化	
void BEEP_flash(u8 times,u16 ms);
void SYS_SW_Init(void);


	 				    
#endif

















