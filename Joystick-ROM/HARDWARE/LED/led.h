#ifndef __LED_H
#define __LED_H	 
#include "sys.h"

//LED端口定义
#define LEDR PAout(7)
#define LEDG PBout(0)
#define LEDB PBout(1)

#define LED_SCALE	7199				//100us 10kHz FRQ
#define LED_DIV		0

#define LED_R_PRE	30					//LED RGB 亮度百分比%
#define LED_G_PRE	40
#define LED_B_PRE	15

#define LED0 PAout(6)

void LED_Init(u16 arr,u16 psc);			//初始化	
void LEDR_PWM_duty_set(u8 duty);
void LEDG_PWM_duty_set(u8 duty);
void LEDB_PWM_duty_set(u8 duty);
//void LED_RGB_flash(u8 r_pre, u8 g_pre, u8 b_pre);
void LED_flash_RGB(u8 times);
void LED_R_shadow(u8 precentage);
void LED_G_shadow(u8 precentage);
void LED_B_shadow(u8 precentage);
//void LED_PWM_flash_timer(void);
void LED_rgb_flash_timer(void);
void LED_RGB_turnoff(void);

#endif

















