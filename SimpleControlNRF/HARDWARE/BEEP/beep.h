#ifndef __BEEP_H
#define __BEEP_H	 
#include "sys.h"

//beep�˿ڶ���

#define BEEP 	PCout(14)	
#define SYS_SW	PCout(15)	

//***************�������������Դ�ܿ���****************//
#define ON							1
#define OFF							0

void BEEP_Init(void);    //��ʼ��	
void BEEP_flash(u8 times,u16 ms);
void SYS_SW_Init(void);


	 				    
#endif

















