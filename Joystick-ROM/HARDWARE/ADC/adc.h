#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"

 #define ADC1_DR_Address    ((u32)0x4001244C)

//��׼��ѹ 3.3V
#define BATTERY_FULL_VALTAGE			      	2605  // 4.2V 		   	 ��ӦADֵ
#define BATTERY_UNDER_VALTAGE		   	       2109  // 3.4V 			 ��ӦADֵ

//************AD1ͨ��***************************//
#define BATTERY_AD_CHANNEL				 	 0 	 //��ز���ͨ��			
#define JOYKIT_LEFT_X_AD_CHANNEL			 1 	 //��ҡ��X�����ͨ��	
#define JOYKIT_LEFT_Y_AD_CHANNEL			 2 	 //��ҡ��Y�����ͨ��	
#define PHOTORESISTOR_AD_CHANNEL			 3 	 //�����������ͨ��	
#define JOYKIT_RIGHT_X_AD_CHANNEL			 4 	 //��ҡ��X�����ͨ��	
#define JOYKIT_RIGHT_Y_AD_CHANNEL			 5	 //��ҡ��Y�����ͨ��	


void Adc_Init(void);
void Adc_DMA_Init(void);
u8 Get_Adc(void);
u8 Get_Percentage_Ad(u16 adc);
u8 Get_Photoresistor_Percentage_Ad(u16 adc);
void Get_all_percentage_ad(void);
void Adc_timer(void);

#endif 
