#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h"

void EXTIX_Init_Nrfirq(void);								//NRF�ⲿ�жϳ�ʼ��		
void EXTIX_Init_curkill(void);								//������ֵ�����ⲿ�жϳ�ʼ��	
void Nrf24l01_Receive_Data_Handle(u8 *data_buf,u8 len);  //�������ݴ���

#endif

