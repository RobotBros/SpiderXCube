#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"


#define TIM7_TIME_200MS		2
#define TIM7_TIME_300MS		3
#define TIM7_TIME_500MS		5
#define TIM7_TIME_1S			10

void TIM5_PWM_Init(u16 arr,u16 psc);   //PWM初始化
void TIM3_PWM_Init(u16 arr,u16 psc);	 //PWM初始化
void TIM2_PWM_Init(u16 arr,u16 psc);	 //PWM初始化
void TIM8_PWM_Init(u16 arr,u16 psc);	 //PWM初始化
void TIM1_PWM_Init(u16 arr,u16 psc);	 //PWM初始化

void TIM4_PWM_Init(u16 arr,u16 psc);	 //PWM初始化 超声波舵机

void TIM6_Int_Init(u16 arr,u16 psc);	 //定时器6初始化 电池电压采样定时器 主优先级1
void TIM7_Int_Init(u16 arr,u16 psc);	 //定时器7初始化 系统状态展示定时器 主优先级2

void TIM6_IRQHandler(void);						 //定时器6中断入口
void TIM7_IRQHandler(void);						 //定时器7中断入口


#if defined ( __ICCARM__ )  //IAR下使用  
#define TIM6_IRQHandler TIM6_DAC_IRQHandler
#endif

#endif
