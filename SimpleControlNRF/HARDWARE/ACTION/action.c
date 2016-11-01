#include "action.h"
#include "timer.h"


u8 Nrf24l01_rx_handle_flag;

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

//右边后脚前端关节舵机角度控制
void Steering_engine_R_B_F_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM5,counter);
}

//右边后脚中间端关节舵机角度控制
void Steering_engine_R_B_M_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM5,counter);
}

//右边后脚尾端关节舵机角度控制
void Steering_engine_R_B_E_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM5,counter);
}

//右边中间脚前端关节舵机角度控制
void Steering_engine_R_M_F_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM5,counter);
}

//右边中间脚中间关节舵机角度控制
void Steering_engine_R_M_M_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM3,counter);
}

//右边中间脚尾端关节舵机角度控制
void Steering_engine_R_M_E_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM3,counter);
}

//右边前脚前端关节舵机角度控制
void Steering_engine_R_F_F_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM3,counter);
}

//右边前脚中间关节舵机角度控制
void Steering_engine_R_F_M_ang(u8 ang)
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM3,counter);
}

//右边前脚尾端关节舵机角度控制
void Steering_engine_R_F_E_ang(u8 ang) 
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM2,counter);
}

//左边前脚尾端关节舵机角度控制
void Steering_engine_L_F_E_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM2,counter);
}

//左边前脚中间关节舵机角度控制
void Steering_engine_L_F_M_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM8,counter);
}

//左边前脚前端关节舵机角度控制
void Steering_engine_L_F_F_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM8,counter);
}

//左边中间脚尾端关节舵机角度控制
void Steering_engine_L_M_E_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM8,counter);
}

//左边中间脚中间关节舵机角度控制
void Steering_engine_L_M_M_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM8,counter);
}


//左边中间脚前端关节舵机角度控制
void Steering_engine_L_M_F_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM1,counter);
}

//左边后脚尾端关节舵机角度控制
void Steering_engine_L_B_E_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM1,counter);
}

//左边后脚中间关节舵机角度控制
void Steering_engine_L_B_M_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM1,counter);
}

//左边后脚前端关节舵机角度控制
void Steering_engine_L_B_F_ang(u8 ang)  
{
	u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM1,counter);
}

//扩展口1舵机角度控制
void Steering_engine_DRV1_ang(u8 ang)
{
    u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare1(TIM4,counter);
}

//扩展口2舵机角度控制
void Steering_engine_DRV2_ang(u8 ang)
{
    u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare2(TIM4,counter);
}

//扩展口3舵机角度控制
void Steering_engine_DRV3_ang(u8 ang)
{
    u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare3(TIM4,counter);
}

//扩展口4舵机角度控制
void Steering_engine_DRV4_ang(u8 ang)
{
    u16 counter;
	counter = Steering_engine_ang(ang);
	TIM_SetCompare4(TIM4,counter);
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
    TIM5_PWM_Init(PWM_DUTY_ARR,PWM_DUTY_DIV); 	//	50Hz(20ms)
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
    Steering_engine_L_B_M_ang(170);      //输入转动角范围 10~90度   , 回中点90度
    Steering_engine_L_B_E_ang(90);			 //输入转动角范围 20~170度  , 回中点90度
    
    Steering_engine_L_M_F_ang(90);			 //输入转动角范围 70~110度? , 回中点90度
    Steering_engine_L_M_M_ang(170);			 //输入转动角范围 10~90度   , 回中点90度
    Steering_engine_L_M_E_ang(90);      //输入转动角范围 20~170度  , 回中点90度
    
    Steering_engine_L_F_F_ang(90);			 //输入转动角范围 70~110度? , 回中点90度
    Steering_engine_L_F_M_ang(170);			 //输入转动角范围 10~90度   , 回中点90度
    Steering_engine_L_F_E_ang(90);			 //输入转动角范围 20~170度  , 回中点90度
    
    Steering_engine_R_F_F_ang(90);		 //输入转动角范围 70~175度? , 回中点90度
    Steering_engine_R_F_M_ang(10);
    Steering_engine_R_F_E_ang(90);
    
    Steering_engine_R_M_E_ang(90);
    Steering_engine_R_M_M_ang(10);
    Steering_engine_R_M_F_ang(90);
	
    Steering_engine_R_B_E_ang(90);
    Steering_engine_R_B_M_ang(10);
    Steering_engine_R_B_F_ang(90);	 
    
    // 扩展舵机控制口
    Steering_engine_DRV1_ang(90);
    Steering_engine_DRV2_ang(90);
    Steering_engine_DRV3_ang(90);	 
    Steering_engine_DRV4_ang(90);	 
}

// 舵机动作处理
void Steering_engine_action_handle(void)
{
	if(Nrf24l01_rx_handle_flag)
	{
		switch(System_state_flag)
		{
            case SYSTEM_NULL:											//初始化模式
			{
				break;
			}						
            
            case SYSTEM_STAND_BY:										//待机模式
			{
				Steering_engine_action_initial();
				break;
			}		
            
            case SYSTEM_NORMAL:										//遥控控制模式
			{
				//Steering_engine_L_B_E_angle = Joystick_left_x_axis_val;
				//Steering_engine_L_M_E_angle = Joystick_right_x_axis_val;
				Steering_engine_L_B_E_ang(Joystick_left_x_axis_val);
				Steering_engine_L_M_E_ang(Joystick_right_x_axis_val);
				break;
			}	
            
            case SYSTEM_NORMAL_WORRING:
			{
				//Steering_engine_L_B_E_angle = Joystick_left_x_axis_val;
				//Steering_engine_L_M_E_angle = Joystick_right_x_axis_val;
				Steering_engine_L_B_E_ang(Joystick_left_x_axis_val);
				Steering_engine_L_M_E_ang(Joystick_right_x_axis_val);
				break;
			}	
            
            case SYSTEM_CTRL:											//PC控制模式
			{
				Steering_engine_L_B_F_ang(Steering_engine_L_B_F_angle);	     
	 			Steering_engine_L_B_M_ang(Steering_engine_L_B_M_angle);   
   				Steering_engine_L_B_E_ang(Steering_engine_L_B_E_angle);			 
                
	 			Steering_engine_L_M_F_ang(Steering_engine_L_M_F_angle);			
	 			Steering_engine_L_M_M_ang(Steering_engine_L_M_M_angle);			
	 			Steering_engine_L_M_E_ang(Steering_engine_L_M_E_angle);    
                
	 			Steering_engine_L_F_F_ang(Steering_engine_L_F_F_angle);			
	 			Steering_engine_L_F_M_ang(Steering_engine_L_F_M_angle);			 
	 			Steering_engine_L_F_E_ang(Steering_engine_L_F_E_angle);			
                
	 			Steering_engine_R_F_F_ang(Steering_engine_R_F_F_angle);		
	 			Steering_engine_R_F_M_ang(Steering_engine_R_F_M_angle);
	 			Steering_engine_R_F_E_ang(Steering_engine_R_F_E_angle);
                
	 			Steering_engine_R_M_E_ang(Steering_engine_R_M_E_angle);
	 			Steering_engine_R_M_M_ang(Steering_engine_R_M_M_angle);
	 			Steering_engine_R_M_F_ang(Steering_engine_R_M_F_angle);
                
	 			Steering_engine_R_B_E_ang(Steering_engine_R_B_E_angle);
	 			Steering_engine_R_B_M_ang(Steering_engine_R_B_M_angle);
	 			Steering_engine_R_B_F_ang(Steering_engine_R_B_F_angle);	 
                
	 			Steering_engine_DRV1_ang(Steering_engine_DRV1_angle);
                Steering_engine_DRV2_ang(Steering_engine_DRV2_angle);
                Steering_engine_DRV3_ang(Steering_engine_DRV3_angle);	 
                Steering_engine_DRV4_ang(Steering_engine_DRV4_angle);
                
				break;
			}
            
            case SYSTEM_CTRL_WORRING:
			{
                
				Steering_engine_L_B_F_ang(Steering_engine_L_B_F_angle);	     
	 			Steering_engine_L_B_M_ang(Steering_engine_L_B_M_angle);   
   				Steering_engine_L_B_E_ang(Steering_engine_L_B_E_angle);			 
                
	 			Steering_engine_L_M_F_ang(Steering_engine_L_M_F_angle);			
	 			Steering_engine_L_M_M_ang(Steering_engine_L_M_M_angle);			
	 			Steering_engine_L_M_E_ang(Steering_engine_L_M_E_angle);    
                
	 			Steering_engine_L_F_F_ang(Steering_engine_L_F_F_angle);			
	 			Steering_engine_L_F_M_ang(Steering_engine_L_F_M_angle);			 
	 			Steering_engine_L_F_E_ang(Steering_engine_L_F_E_angle);			
                
	 			Steering_engine_R_F_F_ang(Steering_engine_R_F_F_angle);		
	 			Steering_engine_R_F_M_ang(Steering_engine_R_F_M_angle);
	 			Steering_engine_R_F_E_ang(Steering_engine_R_F_E_angle);
                
	 			Steering_engine_R_M_E_ang(Steering_engine_R_M_E_angle);
	 			Steering_engine_R_M_M_ang(Steering_engine_R_M_M_angle);
	 			Steering_engine_R_M_F_ang(Steering_engine_R_M_F_angle);
                
	 			Steering_engine_R_B_E_ang(Steering_engine_R_B_E_angle);
	 			Steering_engine_R_B_M_ang(Steering_engine_R_B_M_angle);
	 			Steering_engine_R_B_F_ang(Steering_engine_R_B_F_angle);	 
                
	 			Steering_engine_DRV1_ang(Steering_engine_DRV1_angle);
                Steering_engine_DRV2_ang(Steering_engine_DRV2_angle);
                Steering_engine_DRV3_ang(Steering_engine_DRV3_angle);	 
                Steering_engine_DRV4_ang(Steering_engine_DRV4_angle);	
                
				break;
			}
            
            case SYSTEM_STOP:											//电池电压异常停止模式
			{
				//Steering_engine_all_off();
				break;
			}
            
            default:
			{
				//Steering_engine_all_off();
				break;
			}
		}
	}
}


