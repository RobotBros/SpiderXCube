#include "beep.h"
//////////////////////////////////////////////////////////////////////////////////	 
//V01
//蜂鸣器初始化  Date 20150808 By fishcan						  
////////////////////////////////////////////////////////////////////////////////// 	   

//初始化PC14输出口	    
//beep IO初始化
void BEEP_Init(void)
{
	RCC->APB2ENR|=1<<4;    		//使能PORTC时钟	 
   	 
	GPIOC->CRH&=0XF0FFFFFF; 
	GPIOC->CRH|=0X03000000; 	//PC.14 推挽输出   	 
	BEEP = OFF;      					//PC.14 输出LOW									  
}

//蜂鸣器发声，time:次数，ms: 间隔时间毫秒
void BEEP_flash(u8 times,u16 ms)
{
	u8 cnt;
	
	for(cnt=0;cnt<times;cnt++)
	{
		BEEP = ON;
		delay_ms(ms);
		BEEP = OFF;
		delay_ms(ms);
	}

}

void SYS_SW_Init(void)
{
	RCC->APB2ENR|=1<<4;    		//使能PORTC时钟	 
   	 
	GPIOC->CRH&=0X0FFFFFFF; 
	GPIOC->CRH|=0X30000000; 	//PC.15 推挽输出   	 
	SYS_SW = OFF;      				//PC.15 输出LOW	
}




