#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

//LED�˿ڶ���
#define LED0 PCout(13)

#define LED_ON		0
#define LED_OFF		1

void LED_Init(void);//��ʼ��	
void LED_flash_short(u8 times);
void LED_flash_fast(u8 times);

#endif

















