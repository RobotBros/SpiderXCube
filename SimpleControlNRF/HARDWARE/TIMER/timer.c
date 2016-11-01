#include "timer.h"
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V1.0 20150802
//
//2,����LED0_PWM_VAL�궨�壬����TIM3_CH2����									  
//////////////////////////////////////////////////////////////////////////////////  

u16 Timer7_base_cnt = 0;			//TIM7ʱ��������
u16 Timer7_nrf_base_cnt = 0;		//TIM7 NRF 2.4g ʱ��������
u8 Timer7_nrf_enable = 0;           //TIM7 NRF 2.4G ����flag
u8 System_tim7_display_flag;

//TIM4 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM4_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);						 						  //ʹ�ܶ�ʱ��5ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
    //	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 			//������ӳ��
    
 	//Ultrasonic_DRV ��I/O��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;																					//TIM4_CH1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 																//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);			     																	//��ʼ��GPIO
    
    //��ʼ��TIM4��Ƶ������
	TIM_TimeBaseStructure.TIM_Period = arr; 											//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 										//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 									//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//TIM���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 							//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM4  PWMģʽ����	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 						//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ�������������
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//�������:TIM����Ƚϼ��Ը�
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
    
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  		  							//����Tָ���Ĳ�����ʼ������TIM4_CH1
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);  		  							//����Tָ���Ĳ�����ʼ������TIM4_CH2
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);  		  							//����Tָ���Ĳ�����ʼ������TIM4_CH3
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);  		  							//����Tָ���Ĳ�����ʼ������TIM4_CH4
	
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  						//ʹ��TIM4 CH1��CCR1�ϵ�Ԥװ�ؼĴ��� 
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); 						//ʹ��TIM4 CH2��CCR4�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  						//ʹ��TIM4 CH3��CCR3�ϵ�Ԥװ�ؼĴ��� 
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  						//ʹ��TIM4 CH4��CCR2�ϵ�Ԥװ�ؼĴ���
	
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
}

//TIM5 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM5_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);						 						  //ʹ�ܶ�ʱ��5ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
    //	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 			//������ӳ��
    
 	//L_B_F_DRV ��L_B_M_DRV��L_B_E_DRV��L_M_F_DRV	I/O��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;	//TIM5_CH1/CH2/CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 																//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);			     																	//��ʼ��GPIO
    
    //��ʼ��TIM5��Ƶ������
	TIM_TimeBaseStructure.TIM_Period = arr; 											//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 										//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 									//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//TIM���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 							//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM5  PWMģʽ����
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 						//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ�������������
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;			//�������:TIM����Ƚϼ��Ը�
    
	TIM_OC4Init(TIM5, &TIM_OCInitStructure); 											//����Tָ���Ĳ�����ʼ������TIM5_CH4 
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);  										//����Tָ���Ĳ�����ʼ������TIM5_CH3 
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  										//����Tָ���Ĳ�����ʼ������TIM5_CH2 
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  		  							//����Tָ���Ĳ�����ʼ������TIM5_CH1
    
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable); 						//ʹ��TIM5 CH4��CCR4�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  						//ʹ��TIM5 CH3��CCR3�ϵ�Ԥװ�ؼĴ��� 
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  						//ʹ��TIM5 CH2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  						//ʹ��TIM5 CH1��CCR1�ϵ�Ԥװ�ؼĴ��� 
	
	TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM5
}

//TIM3 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);						 																		 //ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
    //	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 			//������ӳ��
    
 	//L_M_M_DRV ��L_M_E_DRV��L_F_F_DRV��L_F_M_DRV	I/O��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;						//TIM3_CH1/CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 								//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);			     									//��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;						//TIM3_CH3/CH4
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    //��ʼ��TIM3��Ƶ������
	TIM_TimeBaseStructure.TIM_Period = arr; 													//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 												//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 											//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  			//TIM���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);										//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM3  PWMģʽ����
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ�������������
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 		//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 				//�������:TIM����Ƚϼ��Ը�
    
	TIM_OC4Init(TIM3, &TIM_OCInitStructure); 													//����Tָ���Ĳ�����ʼ������TIM3_CH4 
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  												//����Tָ���Ĳ�����ʼ������TIM3_CH3 
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  												//����Tָ���Ĳ�����ʼ������TIM3_CH2 
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  		  									//����Tָ���Ĳ�����ʼ������TIM3_CH1
    
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable); 								//ʹ��TIM3 CH4��CCR4�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  								//ʹ��TIM3 CH3��CCR3�ϵ�Ԥװ�ؼĴ��� 
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); 							  //ʹ��TIM3 CH2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable); 							  //ʹ��TIM3 CH1��CCR1�ϵ�Ԥװ�ؼĴ��� 
    
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
}

//TIM2 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM2_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);						 						  //ʹ�ܶ�ʱ��2ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB  | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE); 												//������ӳ��
    
 	//L_F_E_DRV ��R_F_E_DRV	I/O��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;					//TIM2_CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 								//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);			     									//��ʼ��GPIO
    
    //��ʼ��TIM2��Ƶ������
	TIM_TimeBaseStructure.TIM_Period = arr; 													//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 												//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 											//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  			//TIM���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);										//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM2  PWMģʽ����
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ�������������
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 		//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 				//�������:TIM����Ƚϼ��Ը�
    
	TIM_OC4Init(TIM2, &TIM_OCInitStructure); 													//����Tָ���Ĳ�����ʼ������TIM3_CH4 
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  												//����Tָ���Ĳ�����ʼ������TIM3_CH3 
    
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable); 								//ʹ��TIM3 CH4��CCR4�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  								//ʹ��TIM3 CH3��CCR3�ϵ�Ԥװ�ؼĴ��� 
    
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM2
}

//TIM8 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM8_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);						 							//ʹ�ܶ�ʱ��8ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
    //	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 			//������ӳ��
    
 	//R_F_M_DRV ��R_F_F_DRV��R_M_E_DRV��R_M_M_DRV	I/O��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;	//TIM8_CH1/CH2/CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 																//�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);			     																	//��ʼ��GPIO
    
    //��ʼ��TIM8��Ƶ������
	TIM_TimeBaseStructure.TIM_Period = arr; 													//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 												//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 											//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  			//TIM���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);										//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM8  PWMģʽ����
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ�������������
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 		//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 				//�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;  //�رջ����������ֹSPI1_MOSI����
    
	TIM_OC4Init(TIM8, &TIM_OCInitStructure); 													//����Tָ���Ĳ�����ʼ������TIM8_CH4 
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  												//����Tָ���Ĳ�����ʼ������TIM8_CH3 
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  												//����Tָ���Ĳ�����ʼ������TIM8_CH2 
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  		  									//����Tָ���Ĳ�����ʼ������TIM8_CH1
    
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable); 								//ʹ��TIM8 CH4��CCR4�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  								//ʹ��TIM8 CH3��CCR3�ϵ�Ԥװ�ؼĴ��� 
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable); 							  //ʹ��TIM8 CH2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable); 							  //ʹ��TIM8 CH1��CCR1�ϵ�Ԥװ�ؼĴ��� 
    
	TIM_CtrlPWMOutputs(TIM8,ENABLE); 
	TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM8
}

//TIM1 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
    
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);						 						 //ʹ�ܶ�ʱ��8ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE); 												 //������ӳ��
    
 	//R_M_F_DRV ��R_B_E_DRV��R_B_M_DRV��R_B_F_DRV	I/O��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;	//TIM1_CH1/CH2/CH3/CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 																  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);			     																	  //��ʼ��GPIO
    
    //��ʼ��TIM1��Ƶ������
	TIM_TimeBaseStructure.TIM_Period = arr; 													//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 												//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 											//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  			//TIM���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);										//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM1  PWMģʽ����
    TIM_OCInitStructure.TIM_OCIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNIdleState = 0x0;
    TIM_OCInitStructure.TIM_OCNPolarity = 0x0;
    TIM_OCInitStructure.TIM_OutputNState = 0x0;
    TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ�������������
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 		//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 				//�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;  //�رջ����������ֹ����
    
	TIM_OC4Init(TIM1, &TIM_OCInitStructure); 													//����Tָ���Ĳ�����ʼ������TIM1_CH4 
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);  												//����Tָ���Ĳ�����ʼ������TIM1_CH3 
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  												//����Tָ���Ĳ�����ʼ������TIM1_CH2 
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  		  									//����Tָ���Ĳ�����ʼ������TIM1_CH1
    
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable); 								//ʹ��TIM1 CH4��CCR4�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);  								//ʹ��TIM1 CH3��CCR3�ϵ�Ԥװ�ؼĴ��� 
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable); 							  //ʹ��TIM1 CH2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 							  //ʹ��TIM1 CH1��CCR1�ϵ�Ԥװ�ؼĴ��� 
    
	TIM_CtrlPWMOutputs(TIM1,ENABLE); 
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
}

//TIM6 ͨ�ö�ʱ����ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM6_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); 
	
	//
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 					//����ʱ��:TDTS = Tck_tim  72MHZ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 			//TIM up counter
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); 
    
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE );													//interrupt enable updata
    
	//NVIC�ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  				//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 								//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 								  //IRQ�����ж�
	NVIC_Init(&NVIC_InitStructure);  		
    
	TIM_Cmd(TIM6, ENABLE);  																				  //ʹ��TIM6					 
}

//TIM7 ͨ�ö�ʱ����ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM7_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); 
	
	//
	TIM_TimeBaseStructure.TIM_Period = arr; 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			//����ʱ��:TDTS = Tck_tim  72MHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	//TIM up counter
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); 
    
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE );						//interrupt enable updata
    
	//NVIC�ж����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;  			//�����ȼ�4
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 				//�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 				//IRQ�����ж�
	NVIC_Init(&NVIC_InitStructure);  		
    
	TIM_Cmd(TIM7, ENABLE);  										//ʹ��TIM7	
	Timer7_base_cnt = 0;										//TIM7ʱ��������
	System_tim7_display_flag = SYSTEM_DISPLAY_1S;				//ϵͳ״̬��ʾ��ʱ��־
}


//TIM6 �ж����  
void TIM6_IRQHandler(void) 																					  //ִ��ʱ�� 97.2 uS
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) 							 		//check interrupt bit falg 
	{		
        System_state_check();
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update); 										//clear interrupt bit falg	
	}
}


//TIM7 �ж����  100ms time base
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
            
            /* ��ʱ200ms ����NRF���ݸ���λ�� */
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

