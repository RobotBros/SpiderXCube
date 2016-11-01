#include "timer.h"
#include "adc.h"
#include "nokia5110.h"
#include "led.h"
#include "sys.h"
#include "key.h"

//TIM2 ͨ�ö�ʱ����ʼ�� 		NRFͨѶ��ʱ
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM2_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	
	//
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 					//����ʱ��:TDTS = Tck_tim  72MHZ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 			//TIM up counter
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE );													//interrupt enable updata

	//NVIC�ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  					//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 								//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 								  //IRQ�����ж�
	NVIC_Init(&NVIC_InitStructure);  		

	//TIM_Cmd(TIM2, ENABLE);  																				  //ʹ��TIM6					 
}

//TIM4 ͨ�ö�ʱ����ʼ��   ϵͳ�����л���ʱ
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM4_Int_Init(u16 arr,u16 psc)
{
  	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	
	//
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 					//����ʱ��:TDTS = Tck_tim  72MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 			//TIM up counter
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE );													//interrupt enable updata

	//NVIC�ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  				//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 								//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 								  //IRQ�����ж�
	NVIC_Init(&NVIC_InitStructure);  		

	TIM_Cmd(TIM4, ENABLE);  																				  //ʹ��TIM7					 

	System_ini_state.system_timer_ini_state =DONE;
}

//---------------------------------
//TIM2 �ж����  ͨѶ��ʱ����2.4G RX��ʱ
//--------------------------------
void TIM2_IRQHandler(void) 																					  
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 							 		
	{		
		NRF24L01_TX_Mode();									// 2.4G RX timeout turn TX mode
		TIM_Cmd(TIM2, DISABLE); 
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 										
	}
}

//----------------------------------
//TIM4 �ж���� ϵͳ������ʱ��  2msʱ��
//---------------------------------
void TIM4_IRQHandler(void)  
{
	static u8 timer_cnt=0;

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) 	//check interrupt bit falg 
	{
		timer_cnt++;
		
		if(timer_cnt == 2)						// 4ms
		{
			timer_cnt = 0;
			Adc_timer();						// 50ms��ʱ
			LCD_backlight_set_timer();		// 100ms��ʱ	
			Key_detect();					// 48ms �ж�
			Key_handle_timer();				// 60 ms
			Imu_motion_timer();				// 100ms��ʱ	
			LED_rgb_flash_timer();
			buzzer_type_timer();		
			System_fault_check_timer();		// 200ms
			System_nrf_tx_timer();			// 240ms
			System_nrf_rx_timer();			// 300ms
			System_fault_clear_timer();		// 0.5min
		}
	
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update); 	//clear interrupt bit falg
	}
}

