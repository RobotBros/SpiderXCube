#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V1.0 20150808    fishcan
//1������AD��������		
//2������TIM6 TIM7��ʱ��ϵͳʱ��
//
//V1.1 20150809    fishcan
//3��ϵͳ״̬�жϡ�ϵͳ״̬չʾ
////////////////////////////////////////////////////////////////////////////////// 

u8 System_state_flag;					 //system run state flag
u8 System_initial_state_flag;  //system initial state flag
u8 Communicate_state_flag;     //system communicate ���� flag
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

