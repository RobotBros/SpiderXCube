#include "beep.h"
//////////////////////////////////////////////////////////////////////////////////	 
//V01
//��������ʼ��  Date 20150808 By fishcan						  
////////////////////////////////////////////////////////////////////////////////// 	   

//��ʼ��PC14�����	    
//beep IO��ʼ��
void BEEP_Init(void)
{
	RCC->APB2ENR|=1<<4;    		//ʹ��PORTCʱ��	 
   	 
	GPIOC->CRH&=0XF0FFFFFF; 
	GPIOC->CRH|=0X03000000; 	//PC.14 �������   	 
	BEEP = OFF;      					//PC.14 ���LOW									  
}

//������������time:������ms: ���ʱ�����
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
	RCC->APB2ENR|=1<<4;    		//ʹ��PORTCʱ��	 
   	 
	GPIOC->CRH&=0X0FFFFFFF; 
	GPIOC->CRH|=0X30000000; 	//PC.15 �������   	 
	SYS_SW = OFF;      				//PC.15 ���LOW	
}




