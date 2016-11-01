#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//V1.0 20150808    fishcan
//1、增加AD采样功能		
//2、增加TIM6 TIM7定时器系统时钟
//
//V1.1 20150809    fishcan
//3、系统状态判断、系统状态展示
////////////////////////////////////////////////////////////////////////////////// 
extern u8 System_fault_flag;


int main(void)
{	
	SysInit();

  	while(1)
	{	
		if(System_fault_flag == SYSTEM_NO_FAULT)
		{
			Get_all_percentage_ad();
			LCD_backlight_set();
			Imu_motion_read();
			Key_action();		
			Syterm_master_nrf_tx_handle();
			Syterm_master_nrf_rx_handle();			
		}
		else
		{
			System_fault_check_handle();
			System_fault_clear_handle();
		}
	}	 
}

