#include "timer.h"
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V1.0 20150802
//
//2,增加LED0_PWM_VAL宏定义，控制TIM3_CH2脉宽									  
//////////////////////////////////////////////////////////////////////////////////  

u16 Timer7_base_cnt = 0;			//TIM7时基计数器
u16 Timer7_nrf_base_cnt = 0;		//TIM7 NRF 2.4g 时基计数器
u8 Timer7_nrf_enable = 0;           //TIM7 NRF 2.4G 启用flag
u8 System_tim7_display_flag;

//TIM4 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM4_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);						 						  //使能定时器5时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	
    //	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 			//部分重映射
    
 	//Ultrasonic_DRV ，I/O初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;																					//TIM4_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 																//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);			     																	//初始化GPIO
    
    //初始化TIM4，频率设置
	TIM_TimeBaseStructure.TIM_Period = arr; 											//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 										//设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 									//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//TIM向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 							//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM4  PWM模式设置	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 						//选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//输出极性:TIM输出比较极性高
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
    
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  		  							//根据T指定的参数初始化外设TIM4_CH1
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);  		  							//根据T指定的参数初始化外设TIM4_CH2
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);  		  							//根据T指定的参数初始化外设TIM4_CH3
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);  		  							//根据T指定的参数初始化外设TIM4_CH4
	
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  						//使能TIM4 CH1在CCR1上的预装载寄存器 
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); 						//使能TIM4 CH2在CCR4上的预装载寄存器
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  						//使能TIM4 CH3在CCR3上的预装载寄存器 
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  						//使能TIM4 CH4在CCR2上的预装载寄存器
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
}

//TIM5 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM5_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);						 						  //使能定时器5时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	
    //	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 			//部分重映射
    
 	//L_B_F_DRV ，L_B_M_DRV，L_B_E_DRV，L_M_F_DRV	I/O初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;	//TIM5_CH1/CH2/CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 																//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);			     																	//初始化GPIO
    
    //初始化TIM5，频率设置
	TIM_TimeBaseStructure.TIM_Period = arr; 											//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 										//设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 									//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//TIM向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 							//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM5  PWM模式设置
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 						//选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//输出极性:TIM输出比较极性高
    
	TIM_OC4Init(TIM5, &TIM_OCInitStructure); 											//根据T指定的参数初始化外设TIM5_CH4 
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);  										//根据T指定的参数初始化外设TIM5_CH3 
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  										//根据T指定的参数初始化外设TIM5_CH2 
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  		  							//根据T指定的参数初始化外设TIM5_CH1
    
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable); 						//使能TIM5 CH4在CCR4上的预装载寄存器
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  						//使能TIM5 CH3在CCR3上的预装载寄存器 
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  						//使能TIM5 CH2在CCR2上的预装载寄存器
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  						//使能TIM5 CH1在CCR1上的预装载寄存器 
	
	TIM_Cmd(TIM5, ENABLE);  //使能TIM5
}

//TIM3 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);						 																		 //使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	
    //	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 			//部分重映射
    
 	//L_M_M_DRV ，L_M_E_DRV，L_F_F_DRV，L_F_M_DRV	I/O初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;						//TIM3_CH1/CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 								//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);			     									//初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;						//TIM3_CH3/CH4
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    //初始化TIM3，频率设置
	TIM_TimeBaseStructure.TIM_Period = arr; 													//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 												//设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 											//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  			//TIM向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);										//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM3  PWM模式设置
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 				//输出极性:TIM输出比较极性高
    
	TIM_OC4Init(TIM3, &TIM_OCInitStructure); 													//根据T指定的参数初始化外设TIM3_CH4 
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  												//根据T指定的参数初始化外设TIM3_CH3 
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  												//根据T指定的参数初始化外设TIM3_CH2 
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  		  									//根据T指定的参数初始化外设TIM3_CH1
    
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable); 								//使能TIM3 CH4在CCR4上的预装载寄存器
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  								//使能TIM3 CH3在CCR3上的预装载寄存器 
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); 							  //使能TIM3 CH2在CCR2上的预装载寄存器
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); 							  //使能TIM3 CH1在CCR1上的预装载寄存器 
    
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
}

//TIM2 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM2_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);						 						  //使能定时器2时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE); 												//部分重映射
    
 	//L_F_E_DRV ，R_F_E_DRV	I/O初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;					//TIM2_CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 								//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);			     									//初始化GPIO
    
    //初始化TIM2，频率设置
	TIM_TimeBaseStructure.TIM_Period = arr; 													//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 												//设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 											//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  			//TIM向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);										//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM2  PWM模式设置
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 				//输出极性:TIM输出比较极性高
    
	TIM_OC4Init(TIM2, &TIM_OCInitStructure); 													//根据T指定的参数初始化外设TIM3_CH4 
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  												//根据T指定的参数初始化外设TIM3_CH3 
    
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable); 								//使能TIM3 CH4在CCR4上的预装载寄存器
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  								//使能TIM3 CH3在CCR3上的预装载寄存器 
    
	TIM_Cmd(TIM2, ENABLE);  //使能TIM2
}

//TIM8 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM8_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);						 							//使能定时器8时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	
    //	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 			//部分重映射
    
 	//R_F_M_DRV ，R_F_F_DRV，R_M_E_DRV，R_M_M_DRV	I/O初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;	//TIM8_CH1/CH2/CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 																//复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);			     																	//初始化GPIO
    
    //初始化TIM8，频率设置
	TIM_TimeBaseStructure.TIM_Period = arr; 													//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 												//设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 											//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  			//TIM向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);										//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM8  PWM模式设置
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 				//输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;  //关闭互补输出，防止SPI1_MOSI干扰
    
	TIM_OC4Init(TIM8, &TIM_OCInitStructure); 													//根据T指定的参数初始化外设TIM8_CH4 
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  												//根据T指定的参数初始化外设TIM8_CH3 
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  												//根据T指定的参数初始化外设TIM8_CH2 
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  		  									//根据T指定的参数初始化外设TIM8_CH1
    
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable); 								//使能TIM8 CH4在CCR4上的预装载寄存器
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  								//使能TIM8 CH3在CCR3上的预装载寄存器 
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable); 							  //使能TIM8 CH2在CCR2上的预装载寄存器
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable); 							  //使能TIM8 CH1在CCR1上的预装载寄存器 
    
	TIM_CtrlPWMOutputs(TIM8,ENABLE); 
	TIM_Cmd(TIM8, ENABLE);  //使能TIM8
}

//TIM1 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);						 						 //使能定时器8时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE); 												 //部分重映射
    
 	//R_M_F_DRV ，R_B_E_DRV，R_B_M_DRV，R_B_F_DRV	I/O初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;	//TIM1_CH1/CH2/CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 																  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);			     																	  //初始化GPIO
    
    //初始化TIM1，频率设置
	TIM_TimeBaseStructure.TIM_Period = arr; 													//设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 												//设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 											//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  			//TIM向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);										//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM1  PWM模式设置
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//选择定时器模式:TIM脉冲宽度调制模式1 ,计数时输出为高，计数完成输出低
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 		//比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 				//输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;  //关闭互补输出，防止干扰
    
	TIM_OC4Init(TIM1, &TIM_OCInitStructure); 													//根据T指定的参数初始化外设TIM1_CH4 
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  												//根据T指定的参数初始化外设TIM1_CH3 
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  												//根据T指定的参数初始化外设TIM1_CH2 
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  		  									//根据T指定的参数初始化外设TIM1_CH1
    
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); 								//使能TIM1 CH4在CCR4上的预装载寄存器
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  								//使能TIM1 CH3在CCR3上的预装载寄存器 
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable); 							  //使能TIM1 CH2在CCR2上的预装载寄存器
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 							  //使能TIM1 CH1在CCR1上的预装载寄存器 
    
	TIM_CtrlPWMOutputs(TIM1,ENABLE); 
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
}

//TIM6 通用定时器初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM6_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 
	
	//
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 					//计数时钟:TDTS = Tck_tim  72MHZ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 			//TIM up counter
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); 
    
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE );													//interrupt enable updata
    
	//NVIC中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  				//主优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 								//次优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 								  //IRQ允许中断
	NVIC_Init(&NVIC_InitStructure);  		
    
	TIM_Cmd(TIM6, ENABLE);  																				  //使能TIM6					 
}

//TIM7 通用定时器初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM7_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); 
	
	//
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			//计数时钟:TDTS = Tck_tim  72MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	//TIM up counter
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); 
    
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE );						//interrupt enable updata
    
	//NVIC中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;  			//主优先级4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 				//次优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 				//IRQ允许中断
	NVIC_Init(&NVIC_InitStructure);  		
    
	TIM_Cmd(TIM7, ENABLE);  										//使能TIM7	
	Timer7_base_cnt = 0;										//TIM7时基计数器
	System_tim7_display_flag = SYSTEM_DISPLAY_1S;				//系统状态显示定时标志
}


//TIM6 中断入口  
void TIM6_IRQHandler(void) 																					  //执行时间 97.2 uS
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) 							 		//check interrupt bit falg 
	{		
        System_state_check();
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update); 										//clear interrupt bit falg	
	}
}


//TIM7 中断入口  100ms time base
void TIM7_IRQHandler(void)  
{
    
#ifdef DEBUG
    u8 status, fifo_status;
#endif
    
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) 							   //check interrupt bit falg 
	{
		if(System_fault_state == SYSTEM_FAULT_NULL)
		{
			if(System_tim7_display_flag == SYSTEM_DISPLAY_200MS)
			{
				if(Timer7_base_cnt == TIM7_TIME_200MS)
				{
					LED0 =!LED0;
					Timer7_base_cnt = 0;
				}
			}
			else if(System_tim7_display_flag == SYSTEM_DISPLAY_300MS)
			{
				if(Timer7_base_cnt == TIM7_TIME_300MS)
				{
					LED0 =!LED0;
					Timer7_base_cnt = 0;
				}
			}
			else if(System_tim7_display_flag == SYSTEM_DISPLAY_500MS)
			{
				if(Timer7_base_cnt == TIM7_TIME_500MS)
				{
					LED0 =!LED0;
					Timer7_base_cnt = 0;
				}
			}
			else if(System_tim7_display_flag == SYSTEM_DISPLAY_1S)
			{
				if(Timer7_base_cnt == TIM7_TIME_1S)
				{
					LED0 =!LED0;
					Timer7_base_cnt = 0;
				}
			}
            
            /* 定时200ms 发送NRF数据给上位机 */
            if (Timer7_nrf_enable && Timer7_nrf_base_cnt == TIM7_TIME_200MS) 
            {
                System_nrf_tx_handle();
                Timer7_nrf_base_cnt = 0;
#ifdef DEBUG
                status = NRF24L01_Read_Reg(READ_REG_NRF+STATUS);
                fifo_status = NRF24L01_Read_Reg(READ_REG_NRF+NRF_FIFO_STATUS);
                printf("NRF24L01 STATUS REG: %x; FIFO_STATUS: %x \r\n", status, fifo_status);
#endif
            }
            else
            {
                NRF24L01_RxPacket(Nrf24l01_rx_buff);
                NRF24L01_RX_Mode();
            }
		}
		
		Timer7_base_cnt ++; 
        Timer7_nrf_base_cnt ++;
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update); 								   //clear interrupt bit falg
	}
}

