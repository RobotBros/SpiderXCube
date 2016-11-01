#include "led.h"  

//static u8 Cnt_r,Cnt_g,Cnt_b,Flag_r,Flag_g,Flag_b;			//LED flash counter and flag
//static u8 led_pwm_flag_200ms = 0;

//LED IO��ʼ��
void LED_Init(u16 arr,u16 psc)
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
//RGB LED	I/O��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;						//TIM3_CH2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 	 							
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;			//TIM3_CH3/CH4
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA,GPIO_Pin_7);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	GPIO_SetBits(GPIOB,GPIO_Pin_1);	

	   //��ʼ��TIM3��Ƶ������
	TIM_TimeBaseStructure.TIM_Period = arr; 													//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 												//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 											//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  			//TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);										//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��ʼ��TIM3  PWMģʽ����	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;									//ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1 ,����ʱ���Ϊ�ߣ�������������
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 		//�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 				//�������:TIM����Ƚϼ��Ը�
	TIM_OC4Init(TIM3, &TIM_OCInitStructure); 													//����Tָ���Ĳ�����ʼ������TIM3_CH4 
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  												//����Tָ���Ĳ�����ʼ������TIM3_CH3 
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  												//����Tָ���Ĳ�����ʼ������TIM3_CH2 

	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable); 								//ʹ��TIM3 CH4��CCR4�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);  								//ʹ��TIM3 CH3��CCR3�ϵ�Ԥװ�ؼĴ��� 
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable); 							  //ʹ��TIM3 CH2��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3

	System_ini_state.led_ini_state =DONE;
}

// LED R ռ�ձ�����
void LEDR_PWM_duty_set(u8 duty)
{
	u16 ccr;

	ccr = ((LED_SCALE + 1)  * duty) / 100;

	TIM_SetCompare2(TIM3,ccr);
}

// LED G ռ�ձ�����
void LEDG_PWM_duty_set(u8 duty)
{
	u16 ccr;

	ccr = ((LED_SCALE + 1)  * duty) / 100;

	TIM_SetCompare3(TIM3,ccr);
}


// LED B ռ�ձ�����
void LEDB_PWM_duty_set(u8 duty)
{
	u16 ccr;

	ccr = ((LED_SCALE + 1)  * duty) / 100;

	TIM_SetCompare4(TIM3,ccr);
}

//------------------------------------------
//LED flash turn
//����: times:��˸����
//------------------------------------------
void LED_flash_RGB(u8 times)
{
	u8 cnt;
	
	for(cnt=0;cnt<times;cnt++)
	{
		LEDR_PWM_duty_set(50);
		LEDG_PWM_duty_set(0);
		LEDB_PWM_duty_set(0);
		delay_ms(300);
		LEDR_PWM_duty_set(0);
		LEDG_PWM_duty_set(50);
		LEDB_PWM_duty_set(0);
		delay_ms(300);
		LEDR_PWM_duty_set(0);
		LEDG_PWM_duty_set(0);
		LEDB_PWM_duty_set(50);
		delay_ms(300);
		LEDR_PWM_duty_set(0);
		LEDG_PWM_duty_set(0);
		LEDB_PWM_duty_set(0);
	}
	delay_ms(1000);
}

//------------------------------------
//LED R����ɫ
//����: precentage:���Ȱٷֱ� 0~100
//-----------------------------------
void LED_R_shadow(u8 precentage)
{
	u8 cnt;

	LEDG_PWM_duty_set(0);
	LEDB_PWM_duty_set(0);
	for(cnt=0;cnt<precentage;cnt++)
	{
		LEDR_PWM_duty_set(cnt);
		delay_ms(50);
	}
}

//------------------------------------
//LED G����ɫ
//����: precentage:���Ȱٷֱ� 0~100
//-----------------------------------
void LED_G_shadow(u8 precentage)
{
	u8 cnt;

	LEDR_PWM_duty_set(0);
	LEDB_PWM_duty_set(0);
	for(cnt=0;cnt<precentage;cnt++)
	{
		LEDG_PWM_duty_set(cnt);
		delay_ms(50);
	}
}

//------------------------------------
//LED B����ɫ
//����: precentage:���Ȱٷֱ� 0~100
//-----------------------------------
void LED_B_shadow(u8 precentage)
{
	u8 cnt;

	LEDR_PWM_duty_set(0);
	LEDG_PWM_duty_set(0);
	for(cnt=0;cnt<precentage;cnt++)
	{
		LEDB_PWM_duty_set(cnt);
		delay_ms(50);
	}
}

//------------------------------------
//LED all off
//-----------------------------------
void LED_RGB_turnoff(void)
{
	LEDR_PWM_duty_set(0);
	LEDG_PWM_duty_set(0);
	LEDB_PWM_duty_set(0);
}

//-------------------------------------
//LED RGB �����˸��ʱ��  4ms
//-------------------------------------
void LED_rgb_flash_timer(void)
{
	static u16 temp_4ms=0;			// 4ms��ʱʱ��������
	static u8 cnt_r,cnt_g,cnt_b;
	static u8 cnt_r_gap,cnt_g_gap,cnt_b_gap;
	static u8 flag_r,flag_g,flag_b;
	
if(System_fault_flag == SYSTEM_NO_FAULT)
{
	temp_4ms++;
	if(temp_4ms >= 22) {temp_4ms = 0;}
	
	if(temp_4ms == 21)				// 4*21 = 84ms
	{
		if(cnt_r == LED_R_PRE)
			{flag_r = 1;}
		else if(cnt_r == 5)
			{flag_r = 0;}
		if(flag_r)
			{cnt_r--;}
		else
			{cnt_r++;}
		LEDR_PWM_duty_set(cnt_r);

		cnt_r_gap++;					
		if(cnt_r_gap < 2) {return;}
		else {cnt_r_gap = 0;}
			
		if(cnt_g == LED_G_PRE)
			{flag_g = 1;}
		else if(cnt_g == 5)
			{flag_g = 0;}
		if(flag_g)
			{cnt_g--;}
		else
			{cnt_g++;}
		LEDG_PWM_duty_set(cnt_g);

		cnt_g_gap++;					
		if(cnt_g_gap < 2) {return;}
		else {cnt_g_gap = 0;}
		
		if(cnt_b == LED_B_PRE)
			{flag_b = 1;}
		else if(cnt_b == 5)
			{flag_b = 0;}
		if(flag_b)
			{cnt_b--;}
		else
			{cnt_b++;}
		LEDB_PWM_duty_set(cnt_b);

		cnt_b_gap++;					
		if(cnt_b_gap < 2) {return;}
		else {cnt_b_gap = 0;}
	}
}
}

////LED RGB color flash
//void LED_RGB_flash(u8 r_pre, u8 g_pre, u8 b_pre)
//{	
//	if(led_pwm_flag_200ms)
//	{
//		led_pwm_flag_200ms = 0;
//	
//		if(Cnt_r == r_pre)
//			{Flag_r = 1;}
//		else if(Cnt_r == 5)
//			{Flag_r = 0;}
//		if(Flag_r)
//			{Cnt_r--;}
//		else
//			{Cnt_r++;}
//		LEDR_PWM_duty_set(Cnt_r);
//		delay_ms(20);
//		
//		if(Cnt_g == g_pre)
//			{Flag_g = 1;}
//		else if(Cnt_g == 5)
//			{Flag_g = 0;}
//		if(Flag_g)
//			{Cnt_g--;}
//		else
//			{Cnt_g++;}
//		LEDG_PWM_duty_set(Cnt_g);
//		delay_ms(20);

//		if(Cnt_b == b_pre)
//			{Flag_b = 1;}
//		else if(Cnt_b == 5)
//			{Flag_b = 0;}
//		if(Flag_b)
//			{Cnt_b--;}
//		else
//			{Cnt_b++;}
//		LEDB_PWM_duty_set(Cnt_b);
//		delay_ms(20);	
//	}
//}


//------------------------------------
//LED RGB flash ��ʱ��  200ms
//------------------------------------
//void LED_PWM_flash_timer(void)
//{
//	static u8 tick_4ms = 0;
//
//	tick_4ms++;
//	
//	if (tick_4ms >= 50)			// 4*50 = 200ms
//	{	
//		tick_4ms=0;
//		led_pwm_flag_200ms = 1;
//	}
//}

