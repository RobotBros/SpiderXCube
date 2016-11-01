#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"


#define TIM7_TIME_200MS		2
#define TIM7_TIME_300MS		3
#define TIM7_TIME_500MS		5
#define TIM7_TIME_1S			10

void TIM5_PWM_Init(u16 arr,u16 psc);   //PWM��ʼ��
void TIM3_PWM_Init(u16 arr,u16 psc);	 //PWM��ʼ��
void TIM2_PWM_Init(u16 arr,u16 psc);	 //PWM��ʼ��
void TIM8_PWM_Init(u16 arr,u16 psc);	 //PWM��ʼ��
void TIM1_PWM_Init(u16 arr,u16 psc);	 //PWM��ʼ��

void TIM4_PWM_Init(u16 arr,u16 psc);	 //PWM��ʼ�� ���������

void TIM6_Int_Init(u16 arr,u16 psc);	 //��ʱ��6��ʼ�� ��ص�ѹ������ʱ�� �����ȼ�1
void TIM7_Int_Init(u16 arr,u16 psc);	 //��ʱ��7��ʼ�� ϵͳ״̬չʾ��ʱ�� �����ȼ�2

void TIM6_IRQHandler(void);						 //��ʱ��6�ж����
void TIM7_IRQHandler(void);						 //��ʱ��7�ж����


#if defined ( __ICCARM__ )  //IAR��ʹ��  
#define TIM6_IRQHandler TIM6_DAC_IRQHandler
#endif

#endif
