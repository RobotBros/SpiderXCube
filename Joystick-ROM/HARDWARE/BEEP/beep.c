#include "beep.h"
//////////////////////////////////////////////////////////////////////////////////	 
//V01
//蜂鸣器初始化  Date 20150808 By fishcan						  
////////////////////////////////////////////////////////////////////////////////// 	   

static u8 SoundType=0;		//蜂鸣器响声类型
static u8 SoundTypeTemp=0;
   
//beep IO初始化
void BEEP_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	//BEEP1	I/O初始化 复用
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			//beep1 -> TIM1_CH2 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 							
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	//BEEP1	I/O初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			//beep2 I/O	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	 							
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	GPIO_ResetBits(GPIOA,GPIO_Pin_9);
	GPIO_ResetBits(GPIOA,GPIO_Pin_10);

	   //初始化TIM3，频率设置
	TIM_TimeBaseStructure.TIM_Period = arr; 							//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 						//设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 						//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);				//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3  PWM模式设置	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			//选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 			//输出极性:TIM输出比较极性高
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  						//根据T指定的参数初始化外设TIM1_CH2 

	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable); 			//使能TIM1 CH2在CCR2上的预装载寄存器

	TIM_CtrlPWMOutputs(TIM1,ENABLE); 
	TIM_Cmd(TIM1, ENABLE);  										//使能TIM1

	System_ini_state.beep_ini_state =DONE;
}


// BEEP1  占空比设置
void BEEP_PWM_duty_set(u8 beep1_duty)
{
	u16 ccr2;
	
	ccr2 = ((BEEP_SCALE + 1)  * beep1_duty) / 100;
	TIM_SetCompare2(TIM1,ccr2);
}

//-------------------------------
//蜂鸣器响声类型选择
//sound_type:响声类型
//-------------------------------
void buzzer_sound(u8 sound_type)
{
	SoundTypeTemp = sound_type;	
}

//--------------------------------
//音效类型，判断展示
//--------------------------------
void buzzer_type_timer(void)
{
	static u8 sound_finish_flag=1;		
	static u16 temp_4ms=0;			// 4ms定时时基计数器

	if (sound_finish_flag == 1)
	{
		sound_finish_flag = 0;
		SoundType = SoundTypeTemp;
		SoundTypeTemp = 0;
		temp_4ms=0;
	}
	if (temp_4ms < 582)				// 1.4s
	{
		temp_4ms++;
	}
	
	if (SoundType == SOUND_QUIET)	//静音
	{
		
		buzzer_high_Hz(0,0);			//BEEP1 高电平，保证储能电容放电完毕
		BEEP2 = BEEP_OFF;
		temp_4ms = 0;
		sound_finish_flag = 1;
	}

	else if (SoundType == SOUND_POWER_ON1)
	{
		if (temp_4ms == 1)				//3	 // 10ms
		{
			buzzer_high_Hz(5000,4);			// 1.14k 7009	2.88
			BEEP2 = BEEP_ON;		
		}
		else if (temp_4ms == 13)				// 4*12 =  48ms
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms == 42)				// 4*29 =  116ms
		{
			buzzer_high_Hz(4705,4);			// 1.28k 6249	3.06
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms == 54)     			// 12	
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms == 83)				// 29	
		{
			buzzer_high_Hz(4499,4);			// 1.42k 5632	3.2
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms == 95)				//12
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms == 124)				//29
		{
			buzzer_high_Hz(4393,4);			// 1.497k 5343	3.277
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms == 136)				//12
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms == 165)				//29
		{	
			buzzer_high_Hz(4148,4);			// 1.69k 4732	3.47
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms == 177)				//12
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms == 206)				//29
		{
			buzzer_high_Hz(3912,4);			// 1.9k 4208	3.68
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms == 218)				// 12
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms == 247)				//29
		{
			buzzer_high_Hz(3710,4);			// 2.1k 3808	3.88
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms == 259)				//12
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms == 288)				//29
		{
			buzzer_high_Hz(3599,4);			// 2.22k 3635  4k
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms == 300)				//12
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms >= 582)				// 4*350= 1.4s
		{
			buzzer_high_Hz(0,0);
			sound_finish_flag=1;
		}	
	}

	else if (SoundType == SOUND_TURN_ON)
	{
		if (temp_4ms == 1)
		{
			buzzer_high_Hz(5474,4);			// 2.63k
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms == 10)				// 4*9 = 36ms
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms == 43)				// 4*33 = 132ms
		{
			buzzer_high_Hz(6890,4);			// 2.09k
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms == 52)				//9				
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms == 85)				//33
		{
			buzzer_high_Hz(4416,4);			// 3.126k
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms == 94)				//9
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms >= 205)				// 4*205 =820ms
		{
			buzzer_high_Hz(0,0);	
			sound_finish_flag=1;
		}	
	}
	
	else if (SoundType==SOUND_TURN_OFF)
	{
		if (temp_4ms==1)
		{
			buzzer_high_Hz(4416,4);		// 3.126k
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms==10)				//9
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms==36)			//26
		{
			buzzer_high_Hz(5474,4);		// 2.63k
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms==45)			//9
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms==71)			//26
		{
			buzzer_high_Hz(6890,4);		// 2.09k
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms==80)			//9
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms>=220)			// 4*140=560ms
		{
			buzzer_high_Hz(0,0);
			sound_finish_flag=1;
		}		
	}

	else if (SoundType==SOUND_1SHORT)
	{
		if (temp_4ms==1)
		{
			buzzer_high_Hz(5999,4);		// 2.4k
			BEEP2 = BEEP_ON;
		}
		else if (temp_4ms==51)			// 4*51=204ms
		{
			BEEP2 = BEEP_OFF;
		}
		else if (temp_4ms>=193)			// 4*142=568ms
		{
			buzzer_high_Hz(0,0);
			sound_finish_flag=1;
		}		
	}
}

//---------------------------------
//开机音乐
//--------------------------------
void buzzer_power_on(void)
{
	buzzer_sound(SOUND_POWER_ON1);
}

////---------------------------------
////开机音乐
////--------------------------------
//void buzzer_power_on(void)
//{
//	buzzer_high_Hz(5000,4);			// 1.14k 7009	2.88
//	BEEP2 = BEEP_ON;
//	delay_ms(BEEP_PWM_TIME);
//	BEEP2 = BEEP_OFF;
//	delay_ms(BEEP_DISCHARGE);
//	buzzer_high_Hz(4705,4);			// 1.28k 6249	3.06
//	BEEP2 = BEEP_ON;
//	delay_ms(BEEP_PWM_TIME);
//	BEEP2 = BEEP_OFF;
//	delay_ms(BEEP_DISCHARGE);
//	buzzer_high_Hz(4499,4);			// 1.42k 5632	3.2
//	BEEP2 = BEEP_ON;
//	delay_ms(BEEP_PWM_TIME);	
//	BEEP2 = BEEP_OFF;
//	delay_ms(BEEP_DISCHARGE);
//	buzzer_high_Hz(4393,4);			// 1.497k 5343	3.277
//	BEEP2 = BEEP_ON;
//	delay_ms(BEEP_PWM_TIME);	
//	BEEP2 = BEEP_OFF;
//	delay_ms(BEEP_DISCHARGE);
//	buzzer_high_Hz(4148,4);			// 1.69k 4732	3.47
//	BEEP2 = BEEP_ON;
//	delay_ms(BEEP_PWM_TIME);	
//	BEEP2 = BEEP_OFF;
//	delay_ms(BEEP_DISCHARGE);
//	buzzer_high_Hz(3912,4);			// 1.9k 4208	3.68
//	BEEP2 = BEEP_ON;
//	delay_ms(BEEP_PWM_TIME);	
//	BEEP2 = BEEP_OFF;
//	delay_ms(BEEP_DISCHARGE);
//	buzzer_high_Hz(3710,4);			// 2.1k 3808	3.88
//	BEEP2= BEEP_ON;
//	delay_ms(BEEP_PWM_TIME);	
//	BEEP2= BEEP_OFF;
//	delay_ms(BEEP_DISCHARGE);
//	buzzer_high_Hz(3599,4);			// 2.22k 3635  4k
//	BEEP2= BEEP_ON;
//	delay_ms(BEEP_PWM_TIME);	
//	BEEP2= BEEP_OFF;
//	delay_ms(516);
//	buzzer_high_Hz(0,0);
//	
//}

//----------------------------------
//蜂鸣器设置频率，占空比50%
//参数:period:周期,prescaler:分频比
//----------------------------------
void buzzer_high_Hz(u16 period, u16 prescaler)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	if(period == 0)
	{
		TIM_TimeBaseStructure.TIM_Period = 0; 
		TIM_TimeBaseStructure.TIM_Prescaler =0; 
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	}
	else
	{	
		TIM_TimeBaseStructure.TIM_Period = period; 
		TIM_TimeBaseStructure.TIM_Prescaler =prescaler; 
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
		
		TIM_SetCompare2(TIM1, (period >> 1));
	}
}

