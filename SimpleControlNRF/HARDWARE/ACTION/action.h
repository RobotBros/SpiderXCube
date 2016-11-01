#ifndef __ACTION_H
#define __ACTION_H
#include "sys.h"

#define PWM_DUTY_ARR				54999	  			//   PWM周期 = 55000*25/72000 = 19.0972ms
#define PWM_DUTY_DIV				24		  			//   PWM频率DIV系数25
#define PWM_DUTY_MAX				7200
#define PWM_DUTY_ANG_180  			7200  	 			//   PWM Duty 
#define PWM_DUTY_ANG_0			1440		  	  	//   PWM Duty 	
#define PWM_DUTY_STEP				32					//   PWM 180度/（7200-1440）=0.03125度/计时 * 32 约等于 每步1度；so n度 counter=angle*32+1440


void Steering_engine_PWM_initial(void);
void Steering_engine_action_initial(void);
void Steering_engine_all_off(void);
void Steering_engine_all_on(void);
u16	 Steering_engine_ang(u8 ang);	 		 		//舵机角度控制，控制角度为1度	，回中为90度,返回定时器计数值

void Steering_engine_L_B_F_ang(u8 ang);		//左边后脚前端关节舵机角度控制
void Steering_engine_L_B_M_ang(u8 ang);		//左边后脚中间端关节舵机角度控制
void Steering_engine_L_B_E_ang(u8 ang);		//左边后脚尾端关节舵机角度控制(脚趾)

void Steering_engine_L_M_F_ang(u8 ang);		//左边中间脚前端关节舵机角度控制
void Steering_engine_L_M_M_ang(u8 ang);		//左边中间脚中间关节舵机角度控制
void Steering_engine_L_M_E_ang(u8 ang);		//左边中间脚尾端关节舵机角度控制

void Steering_engine_L_F_F_ang(u8 ang);		//左边前脚前端关节舵机角度控制
void Steering_engine_L_F_M_ang(u8 ang);		//左边前脚中间关节舵机角度控制
void Steering_engine_L_F_E_ang(u8 ang);		//左边前脚尾端关节舵机角度控制

void Steering_engine_R_F_E_ang(u8 ang);		//右边前脚尾端关节舵机角度控制
void Steering_engine_R_F_M_ang(u8 ang);		//右边前脚中间关节舵机角度控制
void Steering_engine_R_F_F_ang(u8 ang);		//右边前脚前端关节舵机角度控制

void Steering_engine_R_M_E_ang(u8 ang);		//右边中间脚尾端关节舵机角度控制
void Steering_engine_R_M_M_ang(u8 ang);		//右边中间脚中间关节舵机角度控制
void Steering_engine_R_M_F_ang(u8 ang);		//右边中间脚前端关节舵机角度控制

void Steering_engine_R_B_E_ang(u8 ang);		//右边后脚尾端关节舵机角度控制
void Steering_engine_R_B_M_ang(u8 ang);		//右边后脚中间关节舵机角度控制
void Steering_engine_R_B_F_ang(u8 ang);		//右边后脚前端关节舵机角度控制


void Steering_engine_DRV1_ang(u8 ang);		//扩展口1舵机角度控制
void Steering_engine_DRV2_ang(u8 ang);		//扩展口2舵机角度控制
void Steering_engine_DRV3_ang(u8 ang);		//扩展口3舵机角度控制
void Steering_engine_DRV4_ang(u8 ang);		//扩展口4舵机角度控制

void Steering_engine_action_handle(void);		// 舵机动作处理


#endif
