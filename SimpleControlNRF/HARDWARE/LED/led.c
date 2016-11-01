#include "led.h"  

//初始化PC13
//LED IO初始化
void LED_Init(void)
{
	RCC->APB2ENR|=1<<4;     //使能PORTC时钟	 
	   	 
	GPIOC->CRH&=0XFF0FFFFF; 
	GPIOC->CRH|=0X00300000; //PC.13 推挽输出   	 
	GPIOC->ODR|=1<<13;      //PC.13 输出高	
}

//LED 短闪，1S改变一次灯状态，即闪烁间隔为2S，time:次数
void LED_flash_short(u8 times)
{
	u8 cnt;

	LED0 = LED_OFF;

	for(cnt=0;cnt<times;cnt++)
	{
		LED0 = LED_ON;
		BEEP = ON;
		delay_ms(1000);
		LED0 = LED_OFF;
		BEEP = OFF;
		delay_ms(1000);
	}

}

//LED 快闪，0.5S改变一次灯状态，即闪烁间隔为1S，time:次数
void LED_flash_fast(u8 times)
{
	u8 cnt;

	LED0 = LED_OFF;

	for(cnt=0;cnt<times;cnt++)
	{
		LED0 = LED_ON;
		BEEP = ON;
		delay_ms(500);
		LED0 = LED_OFF;
		BEEP = OFF;
		delay_ms(500);
	}

}



