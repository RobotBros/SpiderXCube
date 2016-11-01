#include "action.h"
#include "timer.h"

//舵机角度控制，控制角度为1度，回中为90度,返回定时器计数值
//参数错误返回0，
//正常返回900~4500计数值，代表0度~180度
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

//左边后脚前端关节舵机角度控制
void Steering_engine_L_B_F_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM5,counter);
}

//左边后脚中间端关节舵机角度控制
void Steering_engine_L_B_M_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM5,counter);
}

//左边后脚尾端关节舵机角度控制
void Steering_engine_L_B_E_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM5,counter);
}

//左边中间脚前端关节舵机角度控制
void Steering_engine_L_M_F_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM5,counter);
}

//左边中间脚中间关节舵机角度控制
void Steering_engine_L_M_M_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM3,counter);
}

//左边中间脚尾端关节舵机角度控制
void Steering_engine_L_M_E_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM3,counter);
}

//左边前脚前端关节舵机角度控制
void Steering_engine_L_F_F_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM3,counter);
}

//左边前脚中间关节舵机角度控制
void Steering_engine_L_F_M_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM3,counter);
}

//左边前脚尾端关节舵机角度控制
void Steering_engine_L_F_E_ang(u8 ang) 
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM2,counter);
}

//右边前脚尾端关节舵机角度控制
void Steering_engine_R_F_E_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM2,counter);
}

//右边前脚中间关节舵机角度控制
void Steering_engine_R_F_M_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM8,counter);
}

//右边前脚前端关节舵机角度控制
void Steering_engine_R_F_F_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM8,counter);
}

//右边中间脚尾端关节舵机角度控制
void Steering_engine_R_M_E_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM8,counter);
}

//右边中间脚中间关节舵机角度控制
void Steering_engine_R_M_M_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM8,counter);
}


//右边中间脚前端关节舵机角度控制
void Steering_engine_R_M_F_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM1,counter);
}

//右边后脚尾端关节舵机角度控制
void Steering_engine_R_B_E_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM1,counter);
}

//右边后脚中间关节舵机角度控制
void Steering_engine_R_B_M_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM1,counter);
}

//右边后脚前端关节舵机角度控制
void Steering_engine_R_B_F_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM1,counter);
}

//超声波舵机
void Steering_engine_ultrasonic_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM4,counter);
}
	


//舵机PWM输出OFF
void Steering_engine_all_off(void)
{
	TIM_Cmd(TIM5, DISABLE);	   				//TIM5 PWM使能OFF
	TIM_Cmd(TIM3, DISABLE);	  			  //TIM3 PWM使能OFF
	TIM_Cmd(TIM2, DISABLE);	  				//TIM3 PWM使能OFF
	TIM_Cmd(TIM8, DISABLE);	 				  //TIM8 PWM使能OFF
	TIM_CtrlPWMOutputs(TIM8, DISABLE);
	TIM_Cmd(TIM1, DISABLE);	 				  //TIM1 PWM使能OFF
	TIM_CtrlPWMOutputs(TIM1, DISABLE);
	TIM_Cmd(TIM4, DISABLE);	
}

//舵机PWM输出ON
void Steering_engine_all_on(void)
{
	TIM_Cmd(TIM5, ENABLE);	   				//TIM5 PWM使能ON
	TIM_Cmd(TIM3, ENABLE);	 				  //TIM3 PWM使能ON
	TIM_Cmd(TIM2, ENABLE);	 					//TIM2 PWM使能ON
	TIM_Cmd(TIM8, ENABLE);	   				//TIM8 PWM使能ON
	TIM_CtrlPWMOutputs(TIM8, ENABLE); //TIM8&&1 特殊定时器，需要使能PWM
	TIM_Cmd(TIM1, ENABLE);	   				//TIM1 PWM使能ON
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

//舵机PWM输出初始化
 void Steering_engine_PWM_initial(void)
{
   TIM5_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV); 	//	L_M_F PWM频率=72000/36000=2Khz / 40= 50Hz(20ms) 微调DIV=39
	 TIM3_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV); 
	 TIM2_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV);
	 TIM8_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV);
   TIM1_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV);
	 TIM4_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV);
}

//舵机初始位置动作复位
void Steering_engine_action_initial(void)
{
	 Steering_engine_L_B_F_ang(90);	     //输入转动角范围 70~110度? , 回中点90度
	 Steering_engine_L_B_M_ang(10);      //输入转动角范围 10~90度   , 回中点90度
   Steering_engine_L_B_E_ang(90);			 //输入转动角范围 20~170度  , 回中点90度
	 
	 Steering_engine_L_M_F_ang(90);			 //输入转动角范围 70~110度? , 回中点90度
	 Steering_engine_L_M_M_ang(10);			 //输入转动角范围 10~90度   , 回中点90度
	 Steering_engine_L_M_E_ang(90);      //输入转动角范围 20~170度  , 回中点90度
	 
	 Steering_engine_L_F_F_ang(90);			 //输入转动角范围 70~110度? , 回中点90度
	 Steering_engine_L_F_M_ang(10);			 //输入转动角范围 10~90度   , 回中点90度
	 Steering_engine_L_F_E_ang(90);			 //输入转动角范围 20~170度  , 回中点90度
	 
	 Steering_engine_R_F_F_ang(90);		 //输入转动角范围 70~175度? , 回中点90度
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

