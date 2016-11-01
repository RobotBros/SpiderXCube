#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V1.0 20150808    fishcan
//1、增加AD采样功能		
//2、增加TIM6 TIM7定时器系统时钟
//
//V1.1 20150809    fishcan
//3、系统状态判断、系统状态展示
////////////////////////////////////////////////////////////////////////////////// 

u8 System_state_flag;					 //system run state flag
u8 System_initial_state_flag;  //system initial state flag
u8 Communicate_state_flag;     //system communicate 解锁 flag
u8 Old_system_state_flag;
//u16 Bat_val;

int main(void)
{		
	SysInit();
    
    while(1)
	{
		System_state_display();	
		System_lock_check();
		Steering_engine_action_handle();
		//System_nrf_tx_handle();
		delay_ms(200);
	}	 
}

