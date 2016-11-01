#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"

 #define ADC1_DR_Address    ((u32)0x4001244C)

//基准电压 3.3V
#define BATTERY_FULL_VALTAGE			      	2605  // 4.2V 		   	 对应AD值
#define BATTERY_UNDER_VALTAGE		   	       2109  // 3.4V 			 对应AD值

//************AD1通道***************************//
#define BATTERY_AD_CHANNEL				 	 0 	 //电池采样通道			
#define JOYKIT_LEFT_X_AD_CHANNEL			 1 	 //左摇杆X轴采样通道	
#define JOYKIT_LEFT_Y_AD_CHANNEL			 2 	 //左摇杆Y轴采样通道	
#define PHOTORESISTOR_AD_CHANNEL			 3 	 //光敏电阻采样通道	
#define JOYKIT_RIGHT_X_AD_CHANNEL			 4 	 //右摇杆X轴采样通道	
#define JOYKIT_RIGHT_Y_AD_CHANNEL			 5	 //右摇杆Y轴采样通道	


void Adc_Init(void);
void Adc_DMA_Init(void);
u8 Get_Adc(void);
u8 Get_Percentage_Ad(u16 adc);
u8 Get_Photoresistor_Percentage_Ad(u16 adc);
void Get_all_percentage_ad(void);
void Adc_timer(void);

#endif 
