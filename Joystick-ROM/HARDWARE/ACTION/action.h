#ifndef __ACTION_H
#define __ACTION_H
#include "sys.h"

#define PWM_DUTY_ARR				59999	  			//   PWM频率=72000/(59999+1)=1.2Khz / 24 = 50Hz(20ms)  
#define PWM_DUTY_DIV				23		  			//   PWM频率DIV系数，微调DIV=23+1 
#define PWM_DUTY_MAX				7500
#define PWM_DUTY_ANG_180  	7500  	 			//   PWM Duty 12.5% 20ms  2.5/20= 0.125  60000*0.125=7500  
#define PWM_DUTY_ANG_0			1500		  	  //   PWM Duty 2.5%  20ms  0.5/20= 0.025  60000*0.025=1500   	
#define PWM_DUTY_STEP				33					  //   PWM 180度/（7500-1500）=0.03度/计时 * 33 约等于 每步0.99度；so n度 counter=n*20+900


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

void Steering_engine_ultrasonic_ang(u8 ang);		//超声波舵机



#endif
