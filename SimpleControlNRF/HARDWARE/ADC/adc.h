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

//基准电压 3.26V 板体实测
#define BATTERY_AD_VOL_CHANNEL			 10 	   //电池采样通道
#define BATTERY_AD_CUR_CHANNEL			 11 	   //电池采样通道

#define SAMPLE_LEN	  				 		 5	  //电压/电流缓存器长度	 ,采样深度
#define CHANNEL_NUM						 2	  // ADC采样双通道


//#define BATTERY_WRRING_VALTAGE		 	 3183  //7.6V 		   	 对应AD值
//#define BATTERY_UNDER_VALTAGE		   		 3015  //7.2V 			 对应AD值

#define BATTERY_OVER_CURRENT		 	 	 846   //15A 		 对应AD值 1000:17.74A

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

u8	Get_Bat_Percentage_Ad(void);						//返回电池百分比数值
u16	Get_Bat_Ad(void);										//返回电池AD值，采样两次，求平均值

u8   Get_Left_Joystick_X_Percentage_Ad(void);			//返回左摇杆X轴百分比数值  
u8 	Get_Left_Joystick_Y_Percentage_Ad(void);			//返回左摇杆Y轴百分比数值 
u8	Get_Right_Joystick_X_Percentage_Ad(void);  		//返回右摇杆X轴百分比数值  
u8 	Get_Right_Joystick_Y_Percentage_Ad(void);    		//返回右摇杆Y轴百分比数值  

#endif 
