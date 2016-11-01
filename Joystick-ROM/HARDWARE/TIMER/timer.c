#include "timer.h"
#include "adc.h"
#include "nokia5110.h"
#include "led.h"
#include "sys.h"
#include "key.h"

//TIM2 通用定时器初始化 		NRF通讯定时
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM2_Int_Init(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	
	//
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 					//计数时钟:TDTS = Tck_tim  72MHZ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 			//TIM up counter
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE );													//interrupt enable updata

	//NVIC中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  					//主优先级2
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 								//次优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 								  //IRQ允许中断
	NVIC_Init(&NVIC_InitStructure);  		

	//TIM_Cmd(TIM2, ENABLE);  																				  //使能TIM6					 
}

//TIM4 通用定时器初始化   系统函数切换定时
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM4_Int_Init(u16 arr,u16 psc)
{
  	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	
	//
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 					//计数时钟:TDTS = Tck_tim  72MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 			//TIM up counter
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE );													//interrupt enable updata

	//NVIC中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  				//主优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 								//次优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 								  //IRQ允许中断
	NVIC_Init(&NVIC_InitStructure);  		

	TIM_Cmd(TIM4, ENABLE);  																				  //使能TIM7					 

	System_ini_state.system_timer_ini_state =DONE;
}

//---------------------------------
//TIM2 中断入口  通讯定时处理2.4G RX超时
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
//TIM4 中断入口 系统函数定时器  2ms时基
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
			Adc_timer();						// 50ms定时
			LCD_backlight_set_timer();		// 100ms定时	
			Key_detect();					// 48ms 判断
			Key_handle_timer();				// 60 ms
			Imu_motion_timer();				// 100ms定时	
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

