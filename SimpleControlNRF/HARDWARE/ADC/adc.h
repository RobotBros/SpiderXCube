#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"

#ifdef BATTERY_7V4									   
	#define BATTERY_OVER_VALTAGE  				 2637  // 8.5V  
	#define BATTERY_WRRING_VALTAGE		 	 2358  //7.6V 		   	 ??AD?
	#define BATTERY_UNDER_VALTAGE		   		 2234  //7.2V 			 ??AD?
#else
	#define BATTERY_OVER_VALTAGE  				 3910  // 12.6V 	
	#define BATTERY_WRRING_VALTAGE		 	 3537  // 11.4V 		 ??AD?
	#define BATTERY_UNDER_VALTAGE		   		 3351  // 10.8V 		??AD?
#endif


 #define ADC1_DR_Address    ((u32)0x4001244C)

//��׼��ѹ 3.26V ����ʵ��
#define BATTERY_AD_VOL_CHANNEL			 10 	   //��ز���ͨ��
#define BATTERY_AD_CUR_CHANNEL			 11 	   //��ز���ͨ��

#define SAMPLE_LEN	  				 		 5	  //��ѹ/��������������	 ,�������
#define CHANNEL_NUM						 2	  // ADC����˫ͨ��


//#define BATTERY_WRRING_VALTAGE		 	 3183  //7.6V 		   	 ��ӦADֵ
//#define BATTERY_UNDER_VALTAGE		   		 3015  //7.2V 			 ��ӦADֵ

#define BATTERY_OVER_CURRENT		 	 	 846   //15A 		 ��ӦADֵ 1000:17.74A

#define BAT_VOL_CHANNEL					 0	  // ADC voltage channel of buffer
#define BAT_CUR_CHANNEL					 1	  // ADC voltage channel of buffer

void Adc_Init(void);
void Adc_DMA_Init(void);
u16  Get_Adc(u8 ch); 
u16  Get_Adc_Average(u8 ch,u8 times); 
//u16  Adc_recursion_avg(u8 channel);

u8 	Get_AD_Percentage(u16 adc_val);

u16  AD_get_avg(u16* buff, u8 size) ;
void AD_add_queue(u16* buff,u16 ad,u8 len);

u8	Get_Bat_Percentage_Ad(void);						//���ص�ذٷֱ���ֵ
u16	Get_Bat_Ad(void);										//���ص��ADֵ���������Σ���ƽ��ֵ

u8   Get_Left_Joystick_X_Percentage_Ad(void);			//������ҡ��X��ٷֱ���ֵ  
u8 	Get_Left_Joystick_Y_Percentage_Ad(void);			//������ҡ��Y��ٷֱ���ֵ 
u8	Get_Right_Joystick_X_Percentage_Ad(void);  		//������ҡ��X��ٷֱ���ֵ  
u8 	Get_Right_Joystick_Y_Percentage_Ad(void);    		//������ҡ��Y��ٷֱ���ֵ  

#endif 
