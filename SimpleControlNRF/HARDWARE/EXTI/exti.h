#ifndef __EXTI_H
#define __EXIT_H	 
#include "sys.h"

void EXTIX_Init_Nrfirq(void);								//NRF外部中断初始化		
void EXTIX_Init_curkill(void);								//电流峰值保护外部中断初始化	
void Nrf24l01_Receive_Data_Handle(u8 *data_buf,u8 len);  //接收数据处理

#endif

