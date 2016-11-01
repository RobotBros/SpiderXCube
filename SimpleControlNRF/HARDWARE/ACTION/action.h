#ifndef __ACTION_H
#define __ACTION_H
#include "sys.h"

#define PWM_DUTY_ARR				54999	  			//   PWM���� = 55000*25/72000 = 19.0972ms
#define PWM_DUTY_DIV				24		  			//   PWMƵ��DIVϵ��25
#define PWM_DUTY_MAX				7200
#define PWM_DUTY_ANG_180  			7200  	 			//   PWM Duty 
#define PWM_DUTY_ANG_0			1440		  	  	//   PWM Duty 	
#define PWM_DUTY_STEP				32					//   PWM 180��/��7200-1440��=0.03125��/��ʱ * 32 Լ���� ÿ��1�ȣ�so n�� counter=angle*32+1440


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


void Steering_engine_DRV1_ang(u8 ang);		//��չ��1����Ƕȿ���
void Steering_engine_DRV2_ang(u8 ang);		//��չ��2����Ƕȿ���
void Steering_engine_DRV3_ang(u8 ang);		//��չ��3����Ƕȿ���
void Steering_engine_DRV4_ang(u8 ang);		//��չ��4����Ƕȿ���

void Steering_engine_action_handle(void);		// �����������


#endif
