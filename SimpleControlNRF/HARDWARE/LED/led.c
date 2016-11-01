#include "led.h"  

//��ʼ��PC13
//LED IO��ʼ��
void LED_Init(void)
{
	RCC->APB2ENR|=1<<4;     //ʹ��PORTCʱ��	 
	   	 
	GPIOC->CRH&=0XFF0FFFFF; 
	GPIOC->CRH|=0X00300000; //PC.13 �������   	 
	GPIOC->ODR|=1<<13;      //PC.13 �����	
}

//LED ������1S�ı�һ�ε�״̬������˸���Ϊ2S��time:����
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

//LED ������0.5S�ı�һ�ε�״̬������˸���Ϊ1S��time:����
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



