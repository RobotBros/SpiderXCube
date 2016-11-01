#ifndef __ACTION_H
#define __ACTION_H
#include "sys.h"

#define PWM_DUTY_ARR				59999	  			//   PWMƵ��=72000/(59999+1)=1.2Khz / 24 = 50Hz(20ms)  
#define PWM_DUTY_DIV				23		  			//   PWMƵ��DIVϵ����΢��DIV=23+1 
#define PWM_DUTY_MAX				7500
#define PWM_DUTY_ANG_180  	7500  	 			//   PWM Duty 12.5% 20ms  2.5/20= 0.125  60000*0.125=7500  
#define PWM_DUTY_ANG_0			1500		  	  //   PWM Duty 2.5%  20ms  0.5/20= 0.025  60000*0.025=1500   	
#define PWM_DUTY_STEP				33					  //   PWM 180��/��7500-1500��=0.03��/��ʱ * 33 Լ���� ÿ��0.99�ȣ�so n�� counter=n*20+900


void Steering_engine_PWM_initial(void);
void Steering_engine_action_initial(void);
void Steering_engine_all_off(void);
void Steering_engine_all_on(void);
u16	 Steering_engine_ang(u8 ang);	 		 		//����Ƕȿ��ƣ����ƽǶ�Ϊ1��	������Ϊ90��,���ض�ʱ������ֵ

void Steering_engine_L_B_F_ang(u8 ang);		//��ߺ��ǰ�˹ؽڶ���Ƕȿ���
void Steering_engine_L_B_M_ang(u8 ang);		//��ߺ���м�˹ؽڶ���Ƕȿ���
void Steering_engine_L_B_E_ang(u8 ang);		//��ߺ��β�˹ؽڶ���Ƕȿ���(��ֺ)

void Steering_engine_L_M_F_ang(u8 ang);		//����м��ǰ�˹ؽڶ���Ƕȿ���
void Steering_engine_L_M_M_ang(u8 ang);		//����м���м�ؽڶ���Ƕȿ���
void Steering_engine_L_M_E_ang(u8 ang);		//����м��β�˹ؽڶ���Ƕȿ���

void Steering_engine_L_F_F_ang(u8 ang);		//���ǰ��ǰ�˹ؽڶ���Ƕȿ���
void Steering_engine_L_F_M_ang(u8 ang);		//���ǰ���м�ؽڶ���Ƕȿ���
void Steering_engine_L_F_E_ang(u8 ang);		//���ǰ��β�˹ؽڶ���Ƕȿ���

void Steering_engine_R_F_E_ang(u8 ang);		//�ұ�ǰ��β�˹ؽڶ���Ƕȿ���
void Steering_engine_R_F_M_ang(u8 ang);		//�ұ�ǰ���м�ؽڶ���Ƕȿ���
void Steering_engine_R_F_F_ang(u8 ang);		//�ұ�ǰ��ǰ�˹ؽڶ���Ƕȿ���

void Steering_engine_R_M_E_ang(u8 ang);		//�ұ��м��β�˹ؽڶ���Ƕȿ���
void Steering_engine_R_M_M_ang(u8 ang);		//�ұ��м���м�ؽڶ���Ƕȿ���
void Steering_engine_R_M_F_ang(u8 ang);		//�ұ��м��ǰ�˹ؽڶ���Ƕȿ���

void Steering_engine_R_B_E_ang(u8 ang);		//�ұߺ��β�˹ؽڶ���Ƕȿ���
void Steering_engine_R_B_M_ang(u8 ang);		//�ұߺ���м�ؽڶ���Ƕȿ���
void Steering_engine_R_B_F_ang(u8 ang);		//�ұߺ��ǰ�˹ؽڶ���Ƕȿ���

void Steering_engine_ultrasonic_ang(u8 ang);		//���������



#endif
