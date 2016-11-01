#include "action.h"
#include "timer.h"

//����Ƕȿ��ƣ����ƽǶ�Ϊ1�ȣ�����Ϊ90��,���ض�ʱ������ֵ
//�������󷵻�0��
//��������900~4500����ֵ������0��~180��
u16 Steering_engine_ang(u8 ang)	  
{
   u16 counter;
   if(ang > 180)
   {
   		return 0;
   }
   else
   {
   		counter = ang * PWM_DUTY_STEP + PWM_DUTY_ANG_0;
		return counter;
   }

}

//��ߺ��ǰ�˹ؽڶ���Ƕȿ���
void Steering_engine_L_B_F_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM5,counter);
}

//��ߺ���м�˹ؽڶ���Ƕȿ���
void Steering_engine_L_B_M_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM5,counter);
}

//��ߺ��β�˹ؽڶ���Ƕȿ���
void Steering_engine_L_B_E_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM5,counter);
}

//����м��ǰ�˹ؽڶ���Ƕȿ���
void Steering_engine_L_M_F_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM5,counter);
}

//����м���м�ؽڶ���Ƕȿ���
void Steering_engine_L_M_M_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM3,counter);
}

//����м��β�˹ؽڶ���Ƕȿ���
void Steering_engine_L_M_E_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM3,counter);
}

//���ǰ��ǰ�˹ؽڶ���Ƕȿ���
void Steering_engine_L_F_F_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM3,counter);
}

//���ǰ���м�ؽڶ���Ƕȿ���
void Steering_engine_L_F_M_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM3,counter);
}

//���ǰ��β�˹ؽڶ���Ƕȿ���
void Steering_engine_L_F_E_ang(u8 ang) 
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM2,counter);
}

//�ұ�ǰ��β�˹ؽڶ���Ƕȿ���
void Steering_engine_R_F_E_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM2,counter);
}

//�ұ�ǰ���м�ؽڶ���Ƕȿ���
void Steering_engine_R_F_M_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM8,counter);
}

//�ұ�ǰ��ǰ�˹ؽڶ���Ƕȿ���
void Steering_engine_R_F_F_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM8,counter);
}

//�ұ��м��β�˹ؽڶ���Ƕȿ���
void Steering_engine_R_M_E_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM8,counter);
}

//�ұ��м���м�ؽڶ���Ƕȿ���
void Steering_engine_R_M_M_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM8,counter);
}


//�ұ��м��ǰ�˹ؽڶ���Ƕȿ���
void Steering_engine_R_M_F_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM1,counter);
}

//�ұߺ��β�˹ؽڶ���Ƕȿ���
void Steering_engine_R_B_E_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM1,counter);
}

//�ұߺ���м�ؽڶ���Ƕȿ���
void Steering_engine_R_B_M_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM1,counter);
}

//�ұߺ��ǰ�˹ؽڶ���Ƕȿ���
void Steering_engine_R_B_F_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM1,counter);
}

//���������
void Steering_engine_ultrasonic_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM4,counter);
}
	


//���PWM���OFF
void Steering_engine_all_off(void)
{
	TIM_Cmd(TIM5, DISABLE);	   				//TIM5 PWMʹ��OFF
	TIM_Cmd(TIM3, DISABLE);	  			  //TIM3 PWMʹ��OFF
	TIM_Cmd(TIM2, DISABLE);	  				//TIM3 PWMʹ��OFF
	TIM_Cmd(TIM8, DISABLE);	 				  //TIM8 PWMʹ��OFF
	TIM_CtrlPWMOutputs(TIM8, DISABLE);
	TIM_Cmd(TIM1, DISABLE);	 				  //TIM1 PWMʹ��OFF
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	TIM_Cmd(TIM4, DISABLE);	
}

//���PWM���ON
void Steering_engine_all_on(void)
{
	TIM_Cmd(TIM5, ENABLE);	   				//TIM5 PWMʹ��ON
	TIM_Cmd(TIM3, ENABLE);	 				  //TIM3 PWMʹ��ON
	TIM_Cmd(TIM2, ENABLE);	 					//TIM2 PWMʹ��ON
	TIM_Cmd(TIM8, ENABLE);	   				//TIM8 PWMʹ��ON
	TIM_CtrlPWMOutputs(TIM8, ENABLE); //TIM8&&1 ���ⶨʱ������Ҫʹ��PWM
	TIM_Cmd(TIM1, ENABLE);	   				//TIM1 PWMʹ��ON
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

//���PWM�����ʼ��
 void Steering_engine_PWM_initial(void)
{
   TIM5_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV); 	//	L_M_F PWMƵ��=72000/36000=2Khz / 40= 50Hz(20ms) ΢��DIV=39
	 TIM3_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV); 
	 TIM2_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV);
	 TIM8_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV);
   TIM1_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV);
	 TIM4_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV);
}

//�����ʼλ�ö�����λ
void Steering_engine_action_initial(void)
{
	 Steering_engine_L_B_F_ang(90);	     //����ת���Ƿ�Χ 70~110��? , ���е�90��
	 Steering_engine_L_B_M_ang(10);      //����ת���Ƿ�Χ 10~90��   , ���е�90��
   Steering_engine_L_B_E_ang(90);			 //����ת���Ƿ�Χ 20~170��  , ���е�90��
	 
	 Steering_engine_L_M_F_ang(90);			 //����ת���Ƿ�Χ 70~110��? , ���е�90��
	 Steering_engine_L_M_M_ang(10);			 //����ת���Ƿ�Χ 10~90��   , ���е�90��
	 Steering_engine_L_M_E_ang(90);      //����ת���Ƿ�Χ 20~170��  , ���е�90��
	 
	 Steering_engine_L_F_F_ang(90);			 //����ת���Ƿ�Χ 70~110��? , ���е�90��
	 Steering_engine_L_F_M_ang(10);			 //����ת���Ƿ�Χ 10~90��   , ���е�90��
	 Steering_engine_L_F_E_ang(90);			 //����ת���Ƿ�Χ 20~170��  , ���е�90��
	 
	 Steering_engine_R_F_F_ang(90);		 //����ת���Ƿ�Χ 70~175��? , ���е�90��
	 Steering_engine_R_F_M_ang(170);
	 Steering_engine_R_F_E_ang(90);


	
	 Steering_engine_R_M_E_ang(90);
	 Steering_engine_R_M_M_ang(170);
	 Steering_engine_R_M_F_ang(90);
	
	 Steering_engine_R_B_E_ang(90);
	 Steering_engine_R_B_M_ang(170);
	 Steering_engine_R_B_F_ang(90);	 
	 
	 Steering_engine_ultrasonic_ang(90);
}

